#include "http.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"   /* LOCAL EDIT #8: fragmentation check */
#include "esp_system.h"
#include "esp_timer.h"

#include "mesh_config.h"
#include "wifi.h"
#include "wifi_config.h"

static const char *TAG = "amaran_http";

static const amaran_mqtt_dispatch_t *s_dispatch;
static httpd_handle_t s_server;

/* "all" / "broadcast" / "" → group address. Otherwise match a fixture key
 * or decimal/hex address. Returns 0 on hit, -1 on no match. */
static int resolve_key_or_addr(const char *s, uint16_t *out)
{
    if (s == NULL || *s == '\0' || strcmp(s, "all") == 0 ||
        strcmp(s, "broadcast") == 0) {
        *out = AMARAN_GROUP_ALL;
        return 0;
    }
    char *end;
    long v = strtol(s, &end, 0);
    if (*end == '\0' && v > 0 && v <= 0xffff) {
        *out = (uint16_t)v;
        return 0;
    }
    for (size_t i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        if (strcmp(AMARAN_LIGHTS[i].key, s) == 0) {
            *out = AMARAN_LIGHTS[i].address;
            return 0;
        }
    }
    return -1;
}

static esp_err_t read_body(httpd_req_t *req, char *buf, size_t cap, size_t *out_len)
{
    if (req->content_len == 0) {
        *out_len = 0;
        buf[0] = '\0';
        return ESP_OK;
    }
    if (req->content_len >= cap) return ESP_FAIL;
    int got = httpd_req_recv(req, buf, req->content_len);
    if (got <= 0) return ESP_FAIL;
    buf[got] = '\0';
    *out_len = got;
    return ESP_OK;
}

static esp_err_t send_ok(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

static esp_err_t send_err(httpd_req_t *req, int code, const char *msg)
{
    httpd_resp_set_status(req, code == 404 ? "404 Not Found"
                          : code == 400 ? "400 Bad Request"
                          : "500 Internal Server Error");
    httpd_resp_set_type(req, "application/json");
    char body[128];
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", msg);
    httpd_resp_sendstr(req, body);
    return ESP_OK;
}

/* POST /refresh — broadcast a status-request to every fixture so the
 * iPad/desktop apps catch up. Mirrors Aaron's Shift+R "refresh all". */
static esp_err_t refresh_all_post(httpd_req_t *req)
{
    s_dispatch->refresh(AMARAN_GROUP_ALL);
    return send_ok(req);
}

/* GET /lights — list known fixtures. */
static esp_err_t lights_get(httpd_req_t *req)
{
    cJSON *arr = cJSON_CreateArray();
    for (size_t i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        const amaran_light_t *l = &AMARAN_LIGHTS[i];
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "key", l->key);
        cJSON_AddStringToObject(item, "name", l->name);
        cJSON_AddNumberToObject(item, "address", l->address);
        char mac[18];
        snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 l->mac[0], l->mac[1], l->mac[2], l->mac[3], l->mac[4], l->mac[5]);
        cJSON_AddStringToObject(item, "mac", mac);
        cJSON_AddItemToArray(arr, item);
    }
    char *out = cJSON_PrintUnformatted(arr);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out);
    free(out);
    cJSON_Delete(arr);
    return ESP_OK;
}

/* Split "/lights/<key>/<action>" into its parts. uri excludes the leading
 * slash already (httpd_req_t->uri starts with '/'). Returns 0 on success. */
static int parse_lights_path(const char *uri, char *key, size_t key_cap,
                             char *action, size_t action_cap)
{
    /* Expect: /lights/<key>/<action> */
    if (strncmp(uri, "/lights/", 8) != 0) return -1;
    const char *p = uri + 8;
    const char *slash = strchr(p, '/');
    if (slash == NULL) return -1;
    size_t klen = slash - p;
    if (klen == 0 || klen >= key_cap) return -1;
    memcpy(key, p, klen);
    key[klen] = '\0';
    const char *q = slash + 1;
    const char *qend = strchr(q, '?');
    if (qend == NULL) qend = q + strlen(q);
    size_t alen = qend - q;
    if (alen == 0 || alen >= action_cap) return -1;
    memcpy(action, q, alen);
    action[alen] = '\0';
    return 0;
}

static esp_err_t lights_post(httpd_req_t *req)
{
    char key[32], action[16];
    if (parse_lights_path(req->uri, key, sizeof(key), action, sizeof(action)) != 0) {
        return send_err(req, 400, "bad path");
    }
    uint16_t dst;
    if (resolve_key_or_addr(key, &dst) != 0) {
        return send_err(req, 404, "unknown fixture");
    }

    char body[256];
    size_t bodylen = 0;
    if (read_body(req, body, sizeof(body), &bodylen) != ESP_OK) {
        return send_err(req, 400, "bad body");
    }

    if (strcmp(action, "on") == 0) {
        s_dispatch->on_off(dst, true);
        return send_ok(req);
    }
    if (strcmp(action, "off") == 0) {
        s_dispatch->on_off(dst, false);
        return send_ok(req);
    }
    if (strcmp(action, "refresh") == 0) {
        s_dispatch->refresh(dst);
        return send_ok(req);
    }

    /* Remaining actions need a JSON body. */
    cJSON *root = bodylen ? cJSON_Parse(body) : NULL;
    if (root == NULL) {
        return send_err(req, 400, "body must be JSON");
    }

    esp_err_t rc = ESP_OK;
    if (strcmp(action, "brightness") == 0) {
        cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "value");
        if (!cJSON_IsNumber(v)) {
            rc = send_err(req, 400, "value: number 0-100");
        } else {
            int pct = v->valueint;
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            s_dispatch->brightness(dst, pct);
            rc = send_ok(req);
        }
    } else if (strcmp(action, "cct") == 0) {
        cJSON *k = cJSON_GetObjectItemCaseSensitive(root, "kelvin");
        cJSON *i = cJSON_GetObjectItemCaseSensitive(root, "intensity");
        cJSON *g = cJSON_GetObjectItemCaseSensitive(root, "gm");
        if (!cJSON_IsNumber(k) || !cJSON_IsNumber(i)) {
            rc = send_err(req, 400, "need {kelvin, intensity}");
        } else {
            int pct = i->valueint;
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            int gm = cJSON_IsNumber(g) ? g->valueint : 0;
            s_dispatch->cct(dst, k->valueint, pct, gm);
            rc = send_ok(req);
        }
    } else if (strcmp(action, "hsi") == 0) {
        cJSON *h = cJSON_GetObjectItemCaseSensitive(root, "hue");
        cJSON *sat = cJSON_GetObjectItemCaseSensitive(root, "sat");
        cJSON *i = cJSON_GetObjectItemCaseSensitive(root, "intensity");
        if (!cJSON_IsNumber(h) || !cJSON_IsNumber(sat) || !cJSON_IsNumber(i)) {
            rc = send_err(req, 400, "need {hue, sat, intensity}");
        } else {
            int pct = i->valueint;
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            s_dispatch->hsi(dst, h->valueint, sat->valueint, pct);
            rc = send_ok(req);
        }
    } else {
        rc = send_err(req, 404, "unknown action");
    }
    cJSON_Delete(root);
    return rc;
}

/* LOCAL EDIT (2026-08-24) — GET /status.
 *
 * The 2026-08-24 post-mortem had nothing to work with: the bridge exposed only
 * /lights, kept no counters, and the one way to read its serial console
 * (plugging in USB) resets the ESP32 and destroys the state you came to look
 * at. So the health surface has to be reachable over the network. Everything
 * here is also published to MQTT for the recorder; this endpoint is the
 * zero-dependency version you can curl mid-incident.
 *
 * The interesting field is mesh.silence_s, and mesh.fixtures[] separates "one
 * fixture is deaf" from "the whole mesh is deaf". */
static esp_err_t status_get(httpd_req_t *req)
{
    int64_t now = esp_timer_get_time();
    int64_t last = amaran_mqtt_last_mesh_reply_us();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "uptime_s", (double)(now / 1000000));
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "min_free_heap",
                            esp_get_minimum_free_heap_size());
    /* LOCAL EDIT #8 (2026-09-06). Heap has been flat across every incident, so
     * these exist to keep ruling exhaustion out rather than because it is
     * suspected: largest_free_block catches fragmentation that a healthy
     * free-heap total would hide. reset_reason and bt_controller survive the
     * reboot that the recovery automation performs, which is otherwise the
     * event that destroys the evidence. */
    cJSON_AddNumberToObject(root, "largest_free_block",
                            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    cJSON_AddStringToObject(root, "reset_reason", amaran_reset_reason_name());
    cJSON_AddStringToObject(root, "bt_controller", amaran_bt_controller_status_name());

    cJSON *w = cJSON_CreateObject();
    cJSON_AddBoolToObject(w, "connected", amaran_wifi_is_connected());
    cJSON_AddNumberToObject(w, "rssi", amaran_wifi_rssi());
    cJSON_AddNumberToObject(w, "disconnects", amaran_wifi_disconnect_count());
    cJSON_AddItemToObject(root, "wifi", w);

    cJSON *m = cJSON_CreateObject();
    cJSON_AddBoolToObject(m, "connected", amaran_mqtt_is_connected());
    cJSON_AddNumberToObject(m, "connects", amaran_mqtt_connect_count());
    cJSON_AddItemToObject(root, "mqtt", m);

    cJSON *mesh = cJSON_CreateObject();
    cJSON_AddBoolToObject(mesh, "alive", amaran_mqtt_mesh_is_alive());
    cJSON_AddNumberToObject(mesh, "replies", amaran_mqtt_mesh_reply_count());
    cJSON_AddNumberToObject(mesh, "silence_s",
                            (double)((last ? now - last : now) / 1000000));
    /* LOCAL EDIT #9: never let an injected failure look like a real one. */
    cJSON_AddNumberToObject(mesh, "simulated_deaf_s",
                            amaran_mqtt_simulated_deaf_remaining_s());

    /* LOCAL EDIT #8: transmit side. Compare against `replies` to tell a deaf
     * receiver from a wedged controller from fixtures that stopped answering
     * -- see the note in mqtt.h. */
    {
        uint32_t tx = 0, tx_err = 0, comp = 0, comp_err = 0;
        int32_t last_tx_err = 0, last_comp_err = 0;
        amaran_mqtt_mesh_tx_stats(&tx, &tx_err, &last_tx_err,
                                  &comp, &comp_err, &last_comp_err);
        cJSON_AddNumberToObject(mesh, "tx", tx);
        cJSON_AddNumberToObject(mesh, "tx_err", tx_err);
        cJSON_AddNumberToObject(mesh, "last_tx_err", last_tx_err);
        cJSON_AddNumberToObject(mesh, "send_comp", comp);
        cJSON_AddNumberToObject(mesh, "send_comp_err", comp_err);
        cJSON_AddNumberToObject(mesh, "last_send_comp_err", last_comp_err);
        amaran_mqtt_add_tx_diag(mesh);  /* LOCAL EDIT #10 */
    }

    cJSON *fx = cJSON_CreateArray();
    for (size_t i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        uint32_t replies = 0;
        int64_t last_us = 0;
        bool available = false;
        amaran_mqtt_fixture_stats((int)i, &replies, &last_us, &available);

        cJSON *o = cJSON_CreateObject();
        cJSON_AddStringToObject(o, "key", AMARAN_LIGHTS[i].key);
        cJSON_AddBoolToObject(o, "available", available);
        cJSON_AddNumberToObject(o, "replies", replies);
        /* -1 = has not answered once since boot. */
        cJSON_AddNumberToObject(o, "last_reply_s_ago",
                                last_us ? (double)((now - last_us) / 1000000)
                                        : -1);
        cJSON_AddItemToArray(fx, o);
    }
    cJSON_AddItemToObject(mesh, "fixtures", fx);
    cJSON_AddItemToObject(root, "mesh", mesh);

    char *json = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t rc = httpd_resp_sendstr(req, json ? json : "{}");
    free(json);
    cJSON_Delete(root);
    return rc;
}

/* LOCAL EDIT #6 (2026-08-31) — POST /reboot.
 *
 * Both mesh-death incidents (2026-08-24, 2026-08-31) ended the same way: the
 * BLE side goes deaf while Wi-Fi, MQTT and the CPU stay provably healthy, and
 * the only cure anyone has found is a power cycle. The bridge sits on a shelf
 * on the always-on USB port of a switched strip, so "power cycle" means
 * physically reaching it — three hours passed on 08-31 before anyone did.
 *
 * The saving grace is that this failure leaves the network path completely
 * intact: /status answered normally throughout both outages. So a software
 * restart is reachable exactly when it is needed. This endpoint is what lets
 * Home Assistant do the reaching.
 *
 * POST, not GET, so a browser prefetch or a crawler cannot reboot the bridge.
 * The endpoint is unauthenticated, like the /lights control that is already
 * here — acceptable on the IoT VLAN, and it grants nothing that power-cycling
 * the strip would not.
 *
 * The restart is deferred so the 200 is flushed first: calling esp_restart()
 * inside the handler kills the socket mid-response and HA would log a failed
 * call for a reboot that actually worked.
 *
 * Abrupt restart is safe for mesh sequence numbers — sdkconfig sets
 * CONFIG_BLE_MESH_SEQ_STORE_RATE=0, so SEQ is persisted on every change and
 * cannot fall behind the peers' replay-protection watermark. */
#define REBOOT_DELAY_MS 500

static void reboot_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "restarting now (requested over HTTP)");
    esp_restart();
}

/* LOCAL EDIT #9 (2026-09-06) — POST /simulate_deaf[?seconds=N].
 *
 * Arms the deaf-mesh injection described in mqtt.h. POST-only for the same
 * reason as /reboot: a browser prefetch must not be able to break the studio.
 * Default 420 s, which comfortably spans the whole recovery sequence --
 * 180 s for the bridge to declare the mesh dead, 120 s more for Home Assistant
 * to act, plus margin -- and the reboot clears it long before it expires. */
#define SIMULATE_DEAF_DEFAULT_S 420
#define SIMULATE_DEAF_MAX_S     900

static esp_err_t simulate_deaf_post(httpd_req_t *req)
{
    int seconds = SIMULATE_DEAF_DEFAULT_S;
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "seconds", val, sizeof(val)) == ESP_OK) {
            seconds = atoi(val);
        }
    }
    if (seconds > SIMULATE_DEAF_MAX_S) seconds = SIMULATE_DEAF_MAX_S;

    amaran_mqtt_simulate_deaf(seconds);

    char body[96];
    snprintf(body, sizeof(body), "{\"ok\":true,\"simulated_deaf_s\":%d}",
             amaran_mqtt_simulated_deaf_remaining_s());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static esp_err_t reboot_post(httpd_req_t *req)
{
    static esp_timer_handle_t s_reboot_timer;

    ESP_LOGW(TAG, "reboot requested over HTTP — restarting in %d ms",
             REBOOT_DELAY_MS);

    if (s_reboot_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = reboot_timer_cb,
            .name = "http_reboot",
        };
        if (esp_timer_create(&args, &s_reboot_timer) != ESP_OK) {
            /* Can't defer — answer first, then go down anyway. A reboot that
             * reports failure is still better than one that never happens. */
            ESP_LOGE(TAG, "esp_timer_create failed; restarting immediately");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":true,\"deferred\":false}");
            esp_restart();
        }
    }

    /* Idempotent under a double-tap: a timer already armed is simply re-armed
     * rather than returning ESP_ERR_INVALID_STATE. */
    esp_timer_stop(s_reboot_timer);
    esp_timer_start_once(s_reboot_timer, (uint64_t)REBOOT_DELAY_MS * 1000);

    char body[96];
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"deferred\":true,\"restart_in_ms\":%d,\"uptime_s\":%lld}",
             REBOOT_DELAY_MS, (long long)(esp_timer_get_time() / 1000000));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

int amaran_http_start(const amaran_mqtt_dispatch_t *dispatch)
{
    s_dispatch = dispatch;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = HTTP_PORT;
    cfg.uri_match_fn = httpd_uri_match_wildcard;

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return -1;
    }

    static const httpd_uri_t r_list = {
        .uri = "/lights",
        .method = HTTP_GET,
        .handler = lights_get,
    };
    static const httpd_uri_t r_post = {
        .uri = "/lights/*",
        .method = HTTP_POST,
        .handler = lights_post,
    };
    static const httpd_uri_t r_refresh_all = {
        .uri = "/refresh",
        .method = HTTP_POST,
        .handler = refresh_all_post,
    };
    static const httpd_uri_t r_status = {      /* LOCAL EDIT (2026-08-24) */
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_get,
    };
    static const httpd_uri_t r_sim_deaf = {    /* LOCAL EDIT #9 (2026-09-06) */
        .uri = "/simulate_deaf",
        .method = HTTP_POST,
        .handler = simulate_deaf_post,
    };
    static const httpd_uri_t r_reboot = {      /* LOCAL EDIT #6 (2026-08-31) */
        .uri = "/reboot",
        .method = HTTP_POST,
        .handler = reboot_post,
    };
    httpd_register_uri_handler(s_server, &r_sim_deaf);
    httpd_register_uri_handler(s_server, &r_list);
    httpd_register_uri_handler(s_server, &r_post);
    httpd_register_uri_handler(s_server, &r_refresh_all);
    httpd_register_uri_handler(s_server, &r_status);
    httpd_register_uri_handler(s_server, &r_reboot);

    ESP_LOGI(TAG, "http listening on :%d", HTTP_PORT);
    return 0;
}
