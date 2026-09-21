#include "wifi.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "mdns.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "wifi_config.h"

static const char *TAG = "amaran_wifi";
#define CONNECTED_BIT (1 << 0)

static EventGroupHandle_t s_evt;
/* Resolves as amaran-bridge.local via mDNS. See amaran_wifi_start(). */
#define AMARAN_HOSTNAME "amaran-bridge"

static amaran_wifi_got_ip_cb_t s_got_ip_cb;
static int s_retries = 0;
/* LOCAL EDIT (2026-08-24): lifetime disconnect count for /status + the MQTT
 * diagnostic sensors. s_retries resets on every got-IP, so it cannot answer
 * "has the link been flapping since boot?" -- this can. */
static uint32_t s_disconnect_count = 0;

static void event_handler(void *arg, esp_event_base_t base,
                          int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retries++;
        s_disconnect_count++;
        ESP_LOGW(TAG, "wifi disconnected, retry %d (lifetime %u)", s_retries,
                 (unsigned)s_disconnect_count);
        xEventGroupClearBits(s_evt, CONNECTED_BIT);
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip " IPSTR, IP2STR(&e->ip_info.ip));
        s_retries = 0;
        xEventGroupSetBits(s_evt, CONNECTED_BIT);
        if (s_got_ip_cb) {
            s_got_ip_cb();
        }
    }
}

int amaran_wifi_start(amaran_wifi_got_ip_cb_t got_ip_cb)
{
    s_got_ip_cb = got_ip_cb;
    s_evt = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    /* LOCAL EDIT #7 (2026-09-01) — announce a name instead of being an IP.
     *
     * The bridge had no hostname at all: it took whatever DHCP lease was free
     * and appeared in the UniFi client list as the stock "espressif". Anything
     * that wanted to reach it -- the /status health check, and now the HA
     * recovery call to POST /reboot -- had to hardcode 10.30.0.168. A DHCP
     * lease buried in a config file is a silent failure waiting to happen: when
     * it moves, nothing announces it, and the recovery path is exactly the
     * thing you do not want failing quietly.
     *
     * Two separate names are set here and they are not the same mechanism:
     *   - esp_netif_set_hostname() is the DHCP client hostname. The gateway
     *     does not serve DNS for it, so it buys readability in the UniFi client
     *     list, not resolution.
     *   - mDNS is what actually makes amaran-bridge.local resolve. The bridge
     *     is on VLAN 30 and Home Assistant is on Default, so this only works
     *     because mDNS reflection is enabled on both networks (verified on the
     *     gateway 2026-09-01; Cameras is deliberately off and must stay off). */
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, AMARAN_HOSTNAME));

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(AMARAN_HOSTNAME));
    ESP_ERROR_CHECK(mdns_instance_name_set("amaran BLE mesh bridge"));
    /* Advertise the HTTP API so the box is discoverable, not just resolvable. */
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = { 0 };
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD,
            sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /* LOCAL EDIT (not upstream): the default WIFI_FAST_SCAN associates to the
     * first SSID match in channel order and never compares RSSI. With the same
     * SSID on several APs it picks the lowest-numbered channel, not the nearest
     * AP -- here that meant a -61 dBm AP on ch 1 over the -35 dBm one 8 ft away
     * on ch 11. Scan every channel and sort by signal instead. */
    wifi_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* LOCAL EDIT (not upstream), KEPT AS A NO-OP ON PURPOSE. The intent was to
     * drop modem sleep: this is a mains-powered bridge holding a persistent MQTT
     * connection, so power save buys nothing and costs ~70 ms of command latency.
     * The call succeeds, but WiFi/BT coexistence forces modem sleep back on once
     * the BLE Mesh stack starts a few seconds later -- WIFI_PS_NONE is not
     * supported on classic ESP32 while the BT controller is running. Measured:
     * the AP reports powersave off at 5 s uptime and on again by 33 s. Left here
     * so the next person doesn't burn an evening rediscovering it. Do not try to
     * force it; the BLE mesh side is what this box is for. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "wifi sta starting, ssid=%s", WIFI_SSID);
    return 0;
}

bool amaran_wifi_is_connected(void)
{
    if (s_evt == NULL) return false;
    return (xEventGroupGetBits(s_evt) & CONNECTED_BIT) != 0;
}

/* LOCAL EDIT (2026-08-24) — diagnostics accessors. */
uint32_t amaran_wifi_disconnect_count(void)
{
    return s_disconnect_count;
}

int amaran_wifi_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return 0;
    return ap.rssi;
}
