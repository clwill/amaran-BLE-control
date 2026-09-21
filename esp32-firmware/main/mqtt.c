#include "mqtt.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "cJSON.h"
#include "esp_bt.h"          /* LOCAL EDIT #8 */
#include "esp_heap_caps.h"   /* LOCAL EDIT #8 */
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "mesh/buf.h"     /* LOCAL EDIT #10: struct net_buf_pool */

#include <errno.h>

#include "mesh_config.h"
#include "wifi.h"
#include "wifi_config.h"

static const char *TAG = "amaran_mqtt";

static esp_mqtt_client_handle_t s_client;
static bool s_connected;
static const amaran_mqtt_dispatch_t *s_dispatch;

/* ---- LOCAL EDIT (2026-08-24): liveness + diagnostics -------------------
 * See the long note in mqtt.h for why this exists. Two availability topics
 * and one diagnostic blob; all three are compile-time literals so they can be
 * handed straight to esp-mqtt's Last Will config, which keeps the pointer. */
#define AVAIL_TOPIC MQTT_TOPIC_PREFIX "/bridge/availability"
#define MESH_TOPIC  MQTT_TOPIC_PREFIX "/bridge/mesh"
#define DIAG_TOPIC  MQTT_TOPIC_PREFIX "/bridge/diag"
#define PAYLOAD_ONLINE  "online"
#define PAYLOAD_OFFLINE "offline"

static uint32_t s_mesh_replies;        /* decoded mesh status replies, lifetime */
static int64_t  s_last_mesh_reply_us;  /* esp_timer time of the last one */
static uint32_t s_mqtt_connects;       /* lifetime MQTT (re)connections */

/* LOCAL EDIT #8 (2026-09-06) -- transmit side. See the long note in mqtt.h. */
static int64_t  s_deaf_until_us;       /* LOCAL EDIT #9: injection deadline */
static uint32_t s_mesh_tx;             /* send_telink_to calls, lifetime */
static uint32_t s_mesh_tx_err;         /* of which the send call refused */
static int32_t  s_mesh_last_tx_err;    /* last non-OK esp_err_t */
static uint32_t s_mesh_send_comp;      /* SEND_COMP_EVT seen, lifetime */
static uint32_t s_mesh_send_comp_err;  /* of which carried an err_code */
static int32_t  s_mesh_last_comp_err;  /* last non-zero SEND_COMP err_code */

/* LOCAL EDIT #10 (2026-09-21) -- see the note in mqtt.h. */
extern struct net_buf_pool adv_buf_pool;  /* IDF core/adv.c, global symbol */
static uint32_t s_comp_err_ebusy;      /* -EBUSY: seg_tx slot busy */
static uint32_t s_comp_err_enobufs;    /* -ENOBUFS: adv pool empty */
static uint32_t s_comp_err_other;      /* anything else */
static int32_t  s_first_comp_err;      /* first non-zero code since boot */
static int64_t  s_first_comp_err_us;   /* ...and when (esp_timer time) */
static uint16_t s_last_comp_err_dst;   /* destination of the last failure */
static int16_t  s_adv_buf_min = -1;    /* lowest adv pool free count seen */
/* Optimistic at boot: the mesh is almost always fine, and the poll sweep
 * downgrades us within MESH_SILENT_SWEEPS_OFFLINE sweeps if it isn't. Starting
 * "offline" would make every restart flap the entities to unavailable. */
static bool s_mesh_alive = true;

/* LOCAL EDIT (2026-08-24, second pass): per-fixture liveness.
 *
 * The whole-mesh signal above is all-or-nothing -- it only fires when every
 * fixture goes quiet. But the seven fixtures are on SEVEN DIFFERENT OUTLETS, so
 * "the gobo is unplugged" is a real and much more likely condition than "the
 * mesh died", and it produced exactly the same lie: HA shows the light
 * available, the zoom script fires, the command goes nowhere.
 *
 * Threshold set from measurement, not taste. Over a 10-sweep window the
 * per-fixture reply rate was: gobo and accent 100%, tube_right/key_right/
 * tube_left 90%, hair 80%, key_left 70%. So a single fixture legitimately
 * misses up to ~30% of sweeps, and a 5-sweep rule (fine for the aggregate,
 * which needs all seven silent at once) would false-trip a single fixture
 * roughly 16x/day.
 *
 * Since HA SUPPRESSES service calls to unavailable entities, a false positive
 * actively breaks the meeting flow while slow detection merely delays a correct
 * diagnosis -- so this is biased hard toward slow. 180 s is ~14 sweeps.
 *
 * Time-since-last-reply rather than counting consecutive misses: simpler, and
 * immune to misses arriving in clusters (which counting assumes they don't).
 * Recovery is immediate -- any reply updates last_reply_us. */
#define FIXTURE_SILENT_US ((int64_t)180 * 1000000)

/* Topic strings: we build them once per fixture at startup. The G/M topics
 * are for a separate HA `number` entity (HA's light schema has no native
 * green/magenta control). */
typedef struct {
    char set_topic[96];
    char state_topic[96];
    char gm_set_topic[96];
    char gm_state_topic[96];
    char avail_topic[96];   /* LOCAL EDIT (2026-08-24): per-fixture liveness */
} fixture_topics_t;
static fixture_topics_t s_topics[AMARAN_LIGHT_COUNT];

/* Last-known per-fixture state — published to the state topics whenever we
 * accept a command. The desktop/iPad apps remain the ground truth of
 * physical state; this is what HA's UI shows. */
typedef struct {
    bool on;
    int brightness;       /* 0–100 */
    int color_temp_mired; /* exact mireds last set by HA (no round-trip drift) */
    int color_temp_k;     /* derived Kelvin, for the mesh send */
    int hue;              /* 0–360 */
    int sat;              /* 0–100 */
    int gm;               /* -50..+50, 0 = neutral */
    bool is_hs;           /* true: last set was color (hs); false: CCT */
    bool dirty;           /* needs a debounced state publish */
    /* LOCAL EDIT (2026-08-13) — on/off verify-and-retry. Mesh sends are
     * fire-and-forget; a lost on/off PDU leaves the fixture contradicting
     * what we told HA, and the post-command refresh then "corrects" HA to
     * the wrong (stale) state. Track the last commanded on/off so a status
     * reply that contradicts a fresh command triggers a bounded re-send
     * instead of being adopted as an external change. */
    bool cmd_pending;        /* an on/off command awaits status confirmation */
    bool cmd_on;             /* the on/off state we commanded */
    int64_t cmd_deadline_us; /* esp_timer time after which we stop retrying */
    int cmd_retries_left;
    bool retry_due;          /* set in mesh-rx context, consumed by timer */
    /* LOCAL EDIT (2026-08-24): per-fixture reply accounting, surfaced by
     * GET /status. Lets you tell "one fixture is deaf" (bad relay path, unit
     * unplugged) from "the whole mesh is deaf" (the 2026-08-24 failure). */
    uint32_t replies;
    int64_t last_reply_us;
    bool avail;              /* this fixture is answering (own MQTT topic) */
} fixture_state_t;
static fixture_state_t s_state[AMARAN_LIGHT_COUNT];

/* Debounced state publish. HA fires many commands/sec while a slider is
 * dragged; echoing every one back as a retained message makes the slider
 * fight itself and jump to stale values. Instead we mark fixtures dirty and
 * flush ~350ms after the last command. */
static esp_timer_handle_t s_publish_timer;
#define PUBLISH_DEBOUNCE_US 350000

static void publish_state(int idx)
{
    if (!s_connected) return;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state", s_state[idx].on ? "ON" : "OFF");
    cJSON_AddNumberToObject(root, "brightness",
                            (s_state[idx].brightness * 255) / 100);
    /* Report the active color mode + its value so HA's UI tracks it. */
    if (s_state[idx].is_hs) {
        cJSON_AddStringToObject(root, "color_mode", "hs");
        cJSON *color = cJSON_CreateObject();
        cJSON_AddNumberToObject(color, "h", s_state[idx].hue);
        cJSON_AddNumberToObject(color, "s", s_state[idx].sat);
        cJSON_AddItemToObject(root, "color", color);
    } else {
        cJSON_AddStringToObject(root, "color_mode", "color_temp");
        /* Echo the exact mireds HA sent — no kelvin round-trip drift. */
        cJSON_AddNumberToObject(root, "color_temp",
                                s_state[idx].color_temp_mired);
    }
    char *json = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(s_client, s_topics[idx].state_topic,
                            json, 0, 0, /*retain*/ 1);
    free(json);
    cJSON_Delete(root);

    /* G/M is a separate number entity — publish its plain-int state. */
    if (AMARAN_LIGHTS[idx].has_gm) {
        char gm_buf[8];
        snprintf(gm_buf, sizeof(gm_buf), "%d", s_state[idx].gm);
        esp_mqtt_client_publish(s_client, s_topics[idx].gm_state_topic,
                                gm_buf, 0, 0, /*retain*/ 1);
    }
}

/* Flush every dirty fixture — fired by the debounce timer. */
static void publish_flush_cb(void *arg)
{
    for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        if (s_state[i].dirty) {
            s_state[i].dirty = false;
            publish_state(i);
        }
    }
}

static void schedule_publish(int idx)
{
    s_state[idx].dirty = true;
    if (!s_publish_timer) return;
    esp_timer_stop(s_publish_timer);
    esp_timer_start_once(s_publish_timer, PUBLISH_DEBOUNCE_US);
}

/* LOCAL EDIT (2026-08-13) — on/off verify-and-retry, send side.
 * amaran_mqtt_report_state() runs in the BLE mesh rx path, so it must not
 * send mesh messages itself; it marks the fixture retry_due and arms this
 * one-shot timer. Mesh sends from the esp_timer task are already the
 * established pattern here (refresh_timer_cb in main.c does the same).
 * Each re-send goes through disp_on_off, whose schedule_refresh() fetches
 * a fresh status ~600ms later — closing the verify loop. */
static esp_timer_handle_t s_retry_timer;
#define CMD_RETRY_DELAY_US   100000              /* 100ms breather */
#define CMD_VERIFY_WINDOW_US (5 * 1000 * 1000)   /* give up after 5s */
#define CMD_RETRIES          2

static void retry_timer_cb(void *arg)
{
    for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        fixture_state_t *st = &s_state[i];
        if (!st->retry_due) continue;
        st->retry_due = false;
        ESP_LOGW(TAG, "on/off for %s not confirmed — resending on=%d (%d left)",
                 AMARAN_LIGHTS[i].key, st->cmd_on, st->cmd_retries_left);
        s_dispatch->on_off(AMARAN_LIGHTS[i].address, st->cmd_on);
    }
}

/* LOCAL EDIT (2026-08-24). Attach HA's availability declaration to a
 * discovery payload. `with_mesh` additionally requires the mesh to be
 * answering -- correct for the light/number entities, wrong for the
 * diagnostic sensors, which must stay readable precisely when the mesh is
 * down. HA's default payloads are "online"/"offline", so we don't restate
 * them. */
static void add_avail_topic(cJSON *arr, const char *topic)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "topic", topic);
    cJSON_AddItemToArray(arr, o);
}

/* `fixture_avail_topic` NULL => bridge-alive only (the diagnostic sensors,
 * which must stay readable when the mesh or a fixture is down). Otherwise the
 * entity requires all three: bridge on the broker, mesh answering, and this
 * particular fixture answering.
 *
 * Keeping MESH_TOPIC in the list is deliberately redundant now that per-fixture
 * exists -- a total mesh outage takes every per-fixture topic offline anyway.
 * It stays as defense in depth for the failure that actually happened, because
 * it is the proven-tested path and the per-fixture logic is newer. */
static void add_availability(cJSON *root, const char *fixture_avail_topic)
{
    cJSON *arr = cJSON_CreateArray();
    add_avail_topic(arr, AVAIL_TOPIC);
    if (fixture_avail_topic) {
        add_avail_topic(arr, MESH_TOPIC);
        add_avail_topic(arr, fixture_avail_topic);
        cJSON_AddStringToObject(root, "availability_mode", "all");
    }
    cJSON_AddItemToObject(root, "availability", arr);
}

/* Attach the shared device block so everything groups under one HA device. */
static void add_device(cJSON *root)
{
    cJSON *dev = cJSON_CreateObject();
    cJSON *ids = cJSON_CreateArray();
    cJSON_AddItemToArray(ids, cJSON_CreateString(DEVICE_ID));
    cJSON_AddItemToObject(dev, "identifiers", ids);
    cJSON_AddStringToObject(dev, "name", "amaran ESP32 mesh bridge");
    cJSON_AddStringToObject(dev, "manufacturer", "amaran (Aputure)");
    cJSON_AddStringToObject(dev, "model", "Telink BLE Mesh fixture");
    cJSON_AddItemToObject(root, "device", dev);
}

/* LOCAL EDIT (2026-08-24): diagnostic sensors. All eight read from the single
 * DIAG_TOPIC JSON blob via value_template -- one publish, eight entities.
 * `mesh_silence` is the one that matters: it is the number the 2026-08-24
 * post-mortem did not have. With it in the recorder, "when did the mesh stop
 * answering, and what else moved at that moment" is a history query instead of
 * a guess. */
typedef struct {
    const char *key;
    const char *name;
    const char *unit;
    const char *device_class;
    const char *state_class;
    const char *icon;
} diag_sensor_t;

static const diag_sensor_t DIAG_SENSORS[] = {
    { "uptime",           "Bridge uptime",           "s",   "duration",        "measurement",      NULL },
    { "free_heap",        "Bridge free heap",        "B",   "data_size",       "measurement",      NULL },
    { "min_free_heap",    "Bridge min free heap",    "B",   "data_size",       "measurement",      NULL },
    { "rssi",             "Bridge WiFi signal",      "dBm", "signal_strength", "measurement",      NULL },
    { "wifi_disconnects", "Bridge WiFi disconnects", NULL,  NULL,              "total_increasing", "mdi:wifi-off" },
    { "mqtt_connects",    "Bridge MQTT connects",    NULL,  NULL,              "total_increasing", "mdi:transit-connection-variant" },
    { "mesh_replies",     "Mesh replies",            NULL,  NULL,              "total_increasing", "mdi:bluetooth-connect" },
    { "mesh_silence",     "Mesh silence",            "s",   "duration",        "measurement",      "mdi:bluetooth-off" },
    /* LOCAL EDIT #8 (2026-09-06) */
    { "mesh_tx",            "Mesh sends",              NULL,  NULL,              "total_increasing", "mdi:bluetooth-transfer" },
    { "mesh_tx_err",        "Mesh send errors",        NULL,  NULL,              "total_increasing", "mdi:alert-circle-outline" },
    { "mesh_send_comp",     "Mesh send completions",   NULL,  NULL,              "total_increasing", "mdi:check-network-outline" },
    { "mesh_send_comp_err", "Mesh completion errors",  NULL,  NULL,              "total_increasing", "mdi:close-network-outline" },
    { "largest_free_block", "Bridge largest free block", "B", "data_size",      "measurement",      NULL },
    { "reset_reason",       "Bridge last reset reason", NULL, NULL,             NULL,               "mdi:restart" },
    { "bt_controller",      "Bridge BT controller",    NULL,  NULL,             NULL,               "mdi:bluetooth-settings" },
    /* LOCAL EDIT #10 (2026-09-21) */
    { "comp_err_last",         "Mesh last completion error",   NULL, NULL, "measurement",      "mdi:alert-octagon-outline" },
    { "comp_err_first",        "Mesh first completion error",  NULL, NULL, "measurement",      "mdi:alert-octagon-outline" },
    { "comp_err_first_uptime", "Mesh first error at uptime",   "s",  "duration", "measurement", "mdi:timer-alert-outline" },
    { "comp_err_ebusy",        "Mesh EBUSY errors",            NULL, NULL, "total_increasing", "mdi:lock-clock" },
    { "comp_err_enobufs",      "Mesh ENOBUFS errors",          NULL, NULL, "total_increasing", "mdi:tray-alert" },
    { "comp_err_other",        "Mesh other errors",            NULL, NULL, "total_increasing", "mdi:help-network-outline" },
    { "comp_err_last_dst",     "Mesh last error destination",  NULL, NULL, NULL,               "mdi:target" },
    { "adv_buf_free",          "Mesh adv buffers free",        NULL, NULL, "measurement",      "mdi:tray-full" },
    { "adv_buf_min",           "Mesh adv buffers min free",    NULL, NULL, "measurement",      "mdi:tray-minus" },
};
#define DIAG_SENSOR_COUNT (sizeof(DIAG_SENSORS) / sizeof(DIAG_SENSORS[0]))

static void publish_diag_discovery(void)
{
    for (size_t i = 0; i < DIAG_SENSOR_COUNT; i++) {
        const diag_sensor_t *d = &DIAG_SENSORS[i];

        char unique_id[64];
        snprintf(unique_id, sizeof(unique_id), "%s_%s", DEVICE_ID, d->key);
        char config_topic[160];
        snprintf(config_topic, sizeof(config_topic), "%s/sensor/%s/config",
                 HA_DISCOVERY_PREFIX, unique_id);
        char tmpl[64];
        snprintf(tmpl, sizeof(tmpl), "{{ value_json.%s }}", d->key);

        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "name", d->name);
        cJSON_AddStringToObject(root, "unique_id", unique_id);
        cJSON_AddStringToObject(root, "object_id", unique_id);
        cJSON_AddStringToObject(root, "state_topic", DIAG_TOPIC);
        cJSON_AddStringToObject(root, "value_template", tmpl);
        cJSON_AddStringToObject(root, "entity_category", "diagnostic");
        if (d->unit) cJSON_AddStringToObject(root, "unit_of_measurement", d->unit);
        if (d->device_class) cJSON_AddStringToObject(root, "device_class", d->device_class);
        if (d->state_class) cJSON_AddStringToObject(root, "state_class", d->state_class);
        if (d->icon) cJSON_AddStringToObject(root, "icon", d->icon);
        /* Bridge-alive only -- deliberately NOT gated on the mesh. */
        add_availability(root, NULL);
        add_device(root);

        char *json = cJSON_PrintUnformatted(root);
        esp_mqtt_client_publish(s_client, config_topic, json, 0, 0, /*retain*/ 1);
        free(json);
        cJSON_Delete(root);
    }
    ESP_LOGI(TAG, "discovery published: %u diagnostic sensors",
             (unsigned)DIAG_SENSOR_COUNT);
}

/* Publish one HA MQTT-discovery message per fixture. */
static void publish_discovery(void)
{
    for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        const amaran_light_t *l = &AMARAN_LIGHTS[i];

        char unique_id[64];
        snprintf(unique_id, sizeof(unique_id), "%s_%s", DEVICE_ID, l->key);

        char config_topic[160];
        snprintf(config_topic, sizeof(config_topic),
                 "%s/light/%s/config", HA_DISCOVERY_PREFIX, unique_id);

        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "name", l->name);
        cJSON_AddStringToObject(root, "unique_id", unique_id);
        cJSON_AddStringToObject(root, "object_id", unique_id);
        cJSON_AddStringToObject(root, "schema", "json");
        cJSON_AddNumberToObject(root, "brightness_scale", 255);
        /* JSON schema: brightness comes free with any color mode. Always
         * expose color_temp; add hs only for RGB-capable fixtures so
         * CCT-only fixtures (e.g. the Halo) don't show a dead color wheel. */
        cJSON_AddBoolToObject(root, "color_mode", true);
        cJSON *modes = cJSON_CreateArray();
        cJSON_AddItemToArray(modes, cJSON_CreateString("color_temp"));
        if (l->has_color) {
            cJSON_AddItemToArray(modes, cJSON_CreateString("hs"));
        }
        cJSON_AddItemToObject(root, "supported_color_modes", modes);
        /* Per-fixture CCT range, in mireds (mired = 1e6 / kelvin). */
        cJSON_AddNumberToObject(root, "min_mireds", 1000000 / l->cct_max);
        cJSON_AddNumberToObject(root, "max_mireds", 1000000 / l->cct_min);
        cJSON_AddStringToObject(root, "command_topic", s_topics[i].set_topic);
        cJSON_AddStringToObject(root, "state_topic", s_topics[i].state_topic);
        cJSON_AddNumberToObject(root, "qos", 0);
        cJSON_AddBoolToObject(root, "retain", true);
        cJSON_AddBoolToObject(root, "optimistic", false);
        /* LOCAL EDIT (2026-08-24): grey the entity out when the bridge is off
         * the broker OR when the fixtures have stopped answering. */
        add_availability(root, s_topics[i].avail_topic);

        add_device(root);

        char *json = cJSON_PrintUnformatted(root);
        esp_mqtt_client_publish(s_client, config_topic, json, 0, 0,
                                /*retain*/ 1);
        ESP_LOGI(TAG, "discovery published: %s", config_topic);
        free(json);
        cJSON_Delete(root);

        /* G/M: a separate HA `number` entity (the light schema has no
         * green/magenta control). -50..+50, 0 = neutral. */
        if (l->has_gm) {
            char gm_uid[80], gm_topic[176];
            snprintf(gm_uid, sizeof(gm_uid), "%s_gm", unique_id);
            snprintf(gm_topic, sizeof(gm_topic), "%s/number/%s/config",
                     HA_DISCOVERY_PREFIX, gm_uid);
            char gm_name[80];
            snprintf(gm_name, sizeof(gm_name), "%s G/M", l->name);

            cJSON *g = cJSON_CreateObject();
            cJSON_AddStringToObject(g, "name", gm_name);
            cJSON_AddStringToObject(g, "unique_id", gm_uid);
            cJSON_AddStringToObject(g, "object_id", gm_uid);
            cJSON_AddStringToObject(g, "command_topic", s_topics[i].gm_set_topic);
            cJSON_AddStringToObject(g, "state_topic", s_topics[i].gm_state_topic);
            /* -10 = full green, 0 = neutral, +10 = full magenta. */
            cJSON_AddNumberToObject(g, "min", -10);
            cJSON_AddNumberToObject(g, "max", 10);
            cJSON_AddNumberToObject(g, "step", 1);
            cJSON_AddStringToObject(g, "mode", "slider");
            cJSON_AddStringToObject(g, "icon", "mdi:invert-colors");
            add_availability(g, s_topics[i].avail_topic);   /* LOCAL EDIT */
            add_device(g);

            char *gjson = cJSON_PrintUnformatted(g);
            esp_mqtt_client_publish(s_client, gm_topic, gjson, 0, 0, 1);
            ESP_LOGI(TAG, "discovery published: %s", gm_topic);
            free(gjson);
            cJSON_Delete(g);
        } else {
            /* Clear any previously-retained G/M entity for fixtures that
             * don't support it (e.g. the Halo after a capability fix) by
             * publishing an empty retained payload to its config topic. */
            char gm_uid[80], gm_topic[176];
            snprintf(gm_uid, sizeof(gm_uid), "%s_gm", unique_id);
            snprintf(gm_topic, sizeof(gm_topic), "%s/number/%s/config",
                     HA_DISCOVERY_PREFIX, gm_uid);
            esp_mqtt_client_publish(s_client, gm_topic, "", 0, 0, 1);
            ESP_LOGI(TAG, "discovery cleared: %s", gm_topic);
        }
    }
    publish_diag_discovery();   /* LOCAL EDIT (2026-08-24) */
}

/* Parse one HA JSON-schema command for a given fixture and dispatch to
 * the mesh send callbacks. After dispatching, update our local state and
 * republish on the state topic so HA's UI reflects it. */
static void handle_command_for(int idx, const char *payload, int len)
{
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGW(TAG, "bad json on set topic");
        return;
    }
    uint16_t dst = AMARAN_LIGHTS[idx].address;

    cJSON *state = cJSON_GetObjectItemCaseSensitive(root, "state");
    cJSON *bri = cJSON_GetObjectItemCaseSensitive(root, "brightness");
    cJSON *ct = cJSON_GetObjectItemCaseSensitive(root, "color_temp");
    cJSON *color = cJSON_GetObjectItemCaseSensitive(root, "color");

    bool wants_on = s_state[idx].on;
    if (cJSON_IsString(state)) {
        wants_on = (strcmp(state->valuestring, "ON") == 0);
    }

    int wants_brightness = s_state[idx].brightness;
    if (cJSON_IsNumber(bri)) {
        int v = (bri->valueint * 100) / 255;
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        wants_brightness = v;
    }

    /* On/off first — a sleeping fixture won't accept other commands until
     * it's woken. */
    if (wants_on != s_state[idx].on) {
        s_dispatch->on_off(dst, wants_on);
        s_state[idx].on = wants_on;
        /* LOCAL EDIT (2026-08-13): arm verify-and-retry — see
         * amaran_mqtt_report_state(). */
        s_state[idx].cmd_pending = true;
        s_state[idx].cmd_on = wants_on;
        s_state[idx].cmd_deadline_us = esp_timer_get_time()
                                       + CMD_VERIFY_WINDOW_US;
        s_state[idx].cmd_retries_left = CMD_RETRIES;
    }

    if (wants_on) {
        /* A single HA command typically carries exactly one of:
         *   color (hs), color_temp, or brightness.
         * Handle whichever is present; fall through to a brightness-only
         * update otherwise. */
        if (cJSON_IsObject(color)) {
            cJSON *h = cJSON_GetObjectItemCaseSensitive(color, "h");
            cJSON *s = cJSON_GetObjectItemCaseSensitive(color, "s");
            int hue = cJSON_IsNumber(h) ? (int)h->valuedouble : s_state[idx].hue;
            int sat = cJSON_IsNumber(s) ? (int)s->valuedouble : s_state[idx].sat;
            s_dispatch->hsi(dst, hue, sat, wants_brightness);
            s_state[idx].is_hs = true;
            s_state[idx].hue = hue;
            s_state[idx].sat = sat;
            s_state[idx].brightness = wants_brightness;
        } else if (cJSON_IsNumber(ct) && ct->valueint > 0) {
            /* Store the exact mireds HA sent; derive kelvin only for the
             * mesh send. Echoing the exact mireds back stops the slider
             * from drifting/jumping. */
            int mireds = ct->valueint;
            int kelvin = 1000000 / mireds;
            s_dispatch->cct(dst, kelvin, wants_brightness, s_state[idx].gm);
            s_state[idx].is_hs = false;
            s_state[idx].color_temp_mired = mireds;
            s_state[idx].color_temp_k = kelvin;
            s_state[idx].brightness = wants_brightness;
        } else if (wants_brightness != s_state[idx].brightness) {
            /* Pure brightness change — preserves current color/CCT on the
             * fixture (Telink 0x8f intensity packet). */
            s_dispatch->brightness(dst, wants_brightness);
            s_state[idx].brightness = wants_brightness;
        }
    }

    cJSON_Delete(root);
    schedule_publish(idx);
}

/* G/M number command: payload is a plain integer -50..+50. Re-send CCT with
 * the stored kelvin + brightness so the tint takes effect immediately (only
 * meaningful in CCT mode; we still store it for HS→CCT transitions). */
static void handle_gm_for(int idx, const char *payload, int len)
{
    char buf[16];
    int n = len < (int)sizeof(buf) - 1 ? len : (int)sizeof(buf) - 1;
    memcpy(buf, payload, n);
    buf[n] = '\0';
    int gm = atoi(buf);
    if (gm < -10) gm = -10;
    if (gm > 10) gm = 10;
    s_state[idx].gm = gm;

    if (s_state[idx].on && !s_state[idx].is_hs) {
        int kelvin = s_state[idx].color_temp_k > 0
                         ? s_state[idx].color_temp_k : 4700;
        s_dispatch->cct(AMARAN_LIGHTS[idx].address, kelvin,
                        s_state[idx].brightness, gm);
    }
    schedule_publish(idx);
}

static int idx_by_addr(uint16_t addr)
{
    for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        if (AMARAN_LIGHTS[i].address == addr) {
            return i;
        }
    }
    return -1;
}

void amaran_mqtt_report_state(uint16_t addr, bool on, bool is_hs,
                              int brightness, int cct_kelvin, int gm,
                              int hue, int sat)
{
    /* LOCAL EDIT #9 (2026-09-06): failure injection. Drop the reply on the
     * floor BEFORE it is counted, so the bridge observes exactly what it
     * observes during a real deaf-mesh incident. Deliberately placed here
     * rather than in the send path: the fixtures stay powered and answering,
     * and the transmit counters keep climbing, which is what distinguishes a
     * deaf receiver from a wedged controller. */
    if (amaran_mqtt_simulated_deaf_remaining_s() > 0) {
        return;
    }

    /* LOCAL EDIT (2026-08-24): liveness. Reaching this function at all means
     * a fixture answered a status request on the mesh -- count it before any
     * path below can bail out. This counter is the ONLY evidence the bridge
     * has that the mesh works; the poll sweep watches it for silence. */
    s_mesh_replies++;
    s_last_mesh_reply_us = esp_timer_get_time();
    /* Deliberately NOT calling amaran_mqtt_set_mesh_alive(true) here: this runs
     * in the BLE mesh rx callback, and esp_mqtt_client_publish can block. The
     * poll sweep owns the transition and restores availability within one
     * interval -- same reason schedule_publish() defers to a timer. */

    int i = idx_by_addr(addr);
    if (i < 0) {
        return;
    }
    fixture_state_t *st = &s_state[i];
    st->replies++;
    st->last_reply_us = s_last_mesh_reply_us;
    bool changed = false;

    /* LOCAL EDIT (2026-08-13) — on/off verify-and-retry, receive side.
     * If this status contradicts an on/off we commanded within the last
     * few seconds, the mesh PDU was lost (observed twice on zoom-off:
     * fixture stayed on, and this function then flipped HA back to "on"
     * ~1s after the off). Re-send the command instead of adopting the
     * stale state. Once retries are exhausted or the window passes, fall
     * through and adopt — genuine external changes (phone app, knob)
     * still flow to HA as before. */
    if (st->cmd_pending) {
        if (on == st->cmd_on) {
            st->cmd_pending = false;             /* confirmed — all good */
        } else if (st->cmd_retries_left > 0 &&
                   esp_timer_get_time() < st->cmd_deadline_us) {
            st->cmd_retries_left--;
            st->retry_due = true;
            if (s_retry_timer) {
                esp_timer_stop(s_retry_timer);
                esp_timer_start_once(s_retry_timer, CMD_RETRY_DELAY_US);
            }
            return;                              /* don't adopt; retry */
        } else {
            st->cmd_pending = false;             /* give up; adopt truth */
        }
    }

    /* Thresholds suppress the echo of our own commands (the fixture reads
     * back ~exactly what we set) while still catching genuine external
     * changes from the desktop / iOS app or a physical knob. */
    if (on != st->on) {
        st->on = on;
        changed = true;
    }

    if (on) {
        if (abs(brightness - st->brightness) > 2) {
            st->brightness = brightness;
            changed = true;
        }
        if (is_hs != st->is_hs) {
            st->is_hs = is_hs;
            changed = true;
        }
        if (is_hs) {
            if (abs(hue - st->hue) > 3) {
                st->hue = hue;
                changed = true;
            }
            if (abs(sat - st->sat) > 3) {
                st->sat = sat;
                changed = true;
            }
        } else {
            if (cct_kelvin > 0 && abs(cct_kelvin - st->color_temp_k) > 60) {
                st->color_temp_k = cct_kelvin;
                st->color_temp_mired = 1000000 / cct_kelvin;
                changed = true;
            }
            if (gm != st->gm) {
                st->gm = gm;
                changed = true;
            }
        }
    }

    if (changed) {
        ESP_LOGI(TAG, "external change on %s -> publish", AMARAN_LIGHTS[i].key);
        schedule_publish(i);
    }
}

/* ---- LOCAL EDIT (2026-08-24): availability + diagnostic publishing ----- */

/* Retained, so a restarting HA learns the current truth immediately rather
 * than waiting up to a poll interval. */
static void publish_bridge_avail(void)
{
    if (!s_connected) return;
    /* qos 0: this is called from inside the MQTT event handler, where a
     * qos>0 publish can block the client task (see esp-mqtt docs). Retained,
     * so late subscribers still get it. */
    esp_mqtt_client_publish(s_client, AVAIL_TOPIC, PAYLOAD_ONLINE, 0, 0,
                            /*retain*/ 1);
}

static void publish_mesh_avail(void)
{
    if (!s_connected) return;
    esp_mqtt_client_publish(s_client, MESH_TOPIC,
                            s_mesh_alive ? PAYLOAD_ONLINE : PAYLOAD_OFFLINE,
                            0, 0, /*retain*/ 1);   /* qos 0 -- see above */
}

void amaran_mqtt_set_mesh_alive(bool alive)
{
    if (alive == s_mesh_alive) return;      /* idempotent -- publish on edge */
    s_mesh_alive = alive;
    if (alive) {
        ESP_LOGW(TAG, "mesh is answering again -- entities available");
    } else {
        ESP_LOGE(TAG, "mesh has gone silent -- marking all fixtures "
                      "unavailable in HA");
    }
    publish_mesh_avail();

    /* LOCAL EDIT #8 (2026-09-06): publish a diagnostic blob IMMEDIATELY on the
     * death edge, rather than waiting up to 60 s for the next scheduled one.
     *
     * Home Assistant reboots the bridge ~2 min after the fixtures go
     * unavailable, and the reboot destroys everything that would explain the
     * failure. Whatever HA is holding at that moment is the whole record, so
     * it must not be a value that is up to a minute stale -- that is precisely
     * the mistake that made the 08-31 onset look like it coincided with the
     * office wake-up when it actually preceded it by 43 s. Publishing here
     * pins the counters to the moment of detection. */
    if (!alive) {
        amaran_mqtt_publish_diag();
    }
}

bool amaran_mqtt_mesh_is_alive(void)
{
    return s_mesh_alive;
}

void amaran_mqtt_publish_diag(void)
{
    if (!s_connected) return;

    int64_t now = esp_timer_get_time();
    /* Never had a reply: report silence as the full uptime, which is the
     * honest answer and reads correctly on a bridge that came up into a dead
     * mesh. */
    int64_t since = s_last_mesh_reply_us ? (now - s_last_mesh_reply_us) : now;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "uptime", (double)(now / 1000000));
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "min_free_heap",
                            esp_get_minimum_free_heap_size());
    cJSON_AddNumberToObject(root, "rssi", amaran_wifi_rssi());
    cJSON_AddNumberToObject(root, "wifi_disconnects",
                            amaran_wifi_disconnect_count());
    cJSON_AddNumberToObject(root, "mqtt_connects", s_mqtt_connects);
    cJSON_AddNumberToObject(root, "mesh_replies", s_mesh_replies);
    cJSON_AddNumberToObject(root, "mesh_silence", (double)(since / 1000000));
    /* LOCAL EDIT #8: the rx/tx discriminator, published continuously so the
     * next incident is explained by history rather than caught live. */
    cJSON_AddNumberToObject(root, "mesh_tx", s_mesh_tx);
    cJSON_AddNumberToObject(root, "mesh_tx_err", s_mesh_tx_err);
    cJSON_AddNumberToObject(root, "mesh_send_comp", s_mesh_send_comp);
    cJSON_AddNumberToObject(root, "mesh_send_comp_err", s_mesh_send_comp_err);
    /* LOCAL EDIT #8: /status-only fields promoted into the MQTT blob, because
     * HA cannot fetch arbitrary HTTP without YAML config -- and these are the
     * three that do not survive the recovery reboot. */
    cJSON_AddNumberToObject(root, "largest_free_block",
                            heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    cJSON_AddStringToObject(root, "reset_reason", amaran_reset_reason_name());
    cJSON_AddStringToObject(root, "bt_controller",
                            amaran_bt_controller_status_name());
    amaran_mqtt_add_tx_diag(root);  /* LOCAL EDIT #10 */

    char *json = cJSON_PrintUnformatted(root);
    /* NOT retained: a stale "mesh_silence" recovered from the broker after a
     * restart would be actively misleading. HA fills these in within one
     * publish interval. */
    esp_mqtt_client_publish(s_client, DIAG_TOPIC, json, 0, 0, /*retain*/ 0);
    free(json);
    cJSON_Delete(root);
}

static void publish_fixture_avail(int idx)
{
    if (!s_connected) return;
    esp_mqtt_client_publish(s_client, s_topics[idx].avail_topic,
                            s_state[idx].avail ? PAYLOAD_ONLINE : PAYLOAD_OFFLINE,
                            0, 0, /*retain*/ 1);   /* qos 0 -- see above */
}

void amaran_mqtt_update_fixture_availability(void)
{
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        fixture_state_t *st = &s_state[i];
        /* Never answered => age counts from boot, so a fixture that is dark
         * from the start still goes unavailable once past the threshold. */
        int64_t age = now - st->last_reply_us;   /* last_reply_us == 0 => uptime */
        bool alive = age < FIXTURE_SILENT_US;
        if (alive == st->avail) continue;        /* publish on edge only */
        st->avail = alive;
        if (alive) {
            ESP_LOGW(TAG, "%s is answering again", AMARAN_LIGHTS[i].key);
        } else {
            ESP_LOGE(TAG, "%s has not answered in %d s -- marking it "
                          "unavailable in HA (unplugged? outlet off?)",
                     AMARAN_LIGHTS[i].key,
                     (int)(FIXTURE_SILENT_US / 1000000));
        }
        publish_fixture_avail(i);
    }
}

void amaran_mqtt_republish_availability(void)
{
    publish_bridge_avail();
    publish_mesh_avail();
    for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) publish_fixture_avail(i);
}

/* LOCAL EDIT #8 (2026-09-06): the previous boot's reset reason, as a word.
 * A bridge that came back from a panic or a watchdog is a completely different
 * story from one the recovery automation rebooted -- and after that reboot the
 * distinction is otherwise unrecoverable. Lives here rather than in http.c
 * because both the HTTP status page and the MQTT blob need it. */
const char *amaran_reset_reason_name(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";   /* our POST /reboot */
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int_wdt";
    case ESP_RST_TASK_WDT:  return "task_wdt";
    case ESP_RST_WDT:       return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
    }
}

const char *amaran_bt_controller_status_name(void)
{
    switch (esp_bt_controller_get_status()) {
    case ESP_BT_CONTROLLER_STATUS_IDLE:    return "idle";
    case ESP_BT_CONTROLLER_STATUS_INITED:  return "inited";
    case ESP_BT_CONTROLLER_STATUS_ENABLED: return "enabled";
    default:                               return "unknown";
    }
}

void amaran_mqtt_simulate_deaf(int seconds)
{
    if (seconds <= 0) {                 /* explicit cancel */
        s_deaf_until_us = 0;
        ESP_LOGW(TAG, "deaf-mesh injection CANCELLED");
        return;
    }
    s_deaf_until_us = esp_timer_get_time() + (int64_t)seconds * 1000000;
    ESP_LOGE(TAG, "deaf-mesh injection ARMED for %d s -- mesh replies will be "
                  "dropped; this is a TEST, not a fault", seconds);
}

int amaran_mqtt_simulated_deaf_remaining_s(void)
{
    if (!s_deaf_until_us) return 0;
    int64_t left = s_deaf_until_us - esp_timer_get_time();
    if (left <= 0) {
        s_deaf_until_us = 0;
        ESP_LOGW(TAG, "deaf-mesh injection expired -- replies flowing again");
        return 0;
    }
    return (int)(left / 1000000);
}

void amaran_mqtt_note_mesh_tx(int err)
{
    s_mesh_tx++;
    if (err != 0) {
        s_mesh_tx_err++;
        s_mesh_last_tx_err = err;
    }
}

static int16_t adv_buf_free(void)
{
    return adv_buf_pool.avail_count;
}

void amaran_mqtt_note_mesh_send_comp(int err_code, uint16_t dst)
{
    s_mesh_send_comp++;
    /* Sampled here because a send is when the pool is most drawn down. */
    int16_t avail = adv_buf_free();
    if (s_adv_buf_min < 0 || avail < s_adv_buf_min) {
        s_adv_buf_min = avail;
    }
    if (err_code != 0) {
        s_mesh_send_comp_err++;
        s_mesh_last_comp_err = err_code;
        s_last_comp_err_dst = dst;
        if (err_code == -EBUSY) {
            s_comp_err_ebusy++;
        } else if (err_code == -ENOBUFS) {
            s_comp_err_enobufs++;
        } else {
            s_comp_err_other++;
        }
        if (!s_first_comp_err) {
            s_first_comp_err = err_code;
            s_first_comp_err_us = esp_timer_get_time();
        }
    }
}

void amaran_mqtt_add_tx_diag(cJSON *obj)
{
    cJSON_AddNumberToObject(obj, "comp_err_last", s_mesh_last_comp_err);
    cJSON_AddNumberToObject(obj, "comp_err_first", s_first_comp_err);
    cJSON_AddNumberToObject(obj, "comp_err_first_uptime",
                            (double)(s_first_comp_err_us / 1000000));
    cJSON_AddNumberToObject(obj, "comp_err_ebusy", s_comp_err_ebusy);
    cJSON_AddNumberToObject(obj, "comp_err_enobufs", s_comp_err_enobufs);
    cJSON_AddNumberToObject(obj, "comp_err_other", s_comp_err_other);
    cJSON_AddNumberToObject(obj, "comp_err_last_dst", s_last_comp_err_dst);
    cJSON_AddNumberToObject(obj, "tx_err_last", s_mesh_last_tx_err);
    cJSON_AddNumberToObject(obj, "adv_buf_free", adv_buf_free());
    cJSON_AddNumberToObject(obj, "adv_buf_min", s_adv_buf_min);
    cJSON_AddNumberToObject(obj, "adv_buf_total", adv_buf_pool.buf_count);
}

void amaran_mqtt_mesh_tx_stats(uint32_t *tx, uint32_t *tx_err,
                               int32_t *last_tx_err, uint32_t *comp,
                               uint32_t *comp_err, int32_t *last_comp_err)
{
    if (tx)            *tx = s_mesh_tx;
    if (tx_err)        *tx_err = s_mesh_tx_err;
    if (last_tx_err)   *last_tx_err = s_mesh_last_tx_err;
    if (comp)          *comp = s_mesh_send_comp;
    if (comp_err)      *comp_err = s_mesh_send_comp_err;
    if (last_comp_err) *last_comp_err = s_mesh_last_comp_err;
}

uint32_t amaran_mqtt_mesh_reply_count(void)
{
    return s_mesh_replies;
}

int64_t amaran_mqtt_last_mesh_reply_us(void)
{
    return s_last_mesh_reply_us;
}

uint32_t amaran_mqtt_connect_count(void)
{
    return s_mqtt_connects;
}

void amaran_mqtt_fixture_stats(int idx, uint32_t *replies, int64_t *last_us,
                               bool *available)
{
    if (idx < 0 || idx >= AMARAN_LIGHT_COUNT) return;
    if (replies) *replies = s_state[idx].replies;
    if (last_us) *last_us = s_state[idx].last_reply_us;
    if (available) *available = s_state[idx].avail;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        s_connected = true;
        s_mqtt_connects++;                      /* LOCAL EDIT (2026-08-24) */
        /* Clear the Last Will's "offline" before discovery, so entities never
         * appear and immediately grey out on a reconnect. */
        publish_bridge_avail();
        publish_mesh_avail();
        for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) publish_fixture_avail(i);
        publish_discovery();
        for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
            esp_mqtt_client_subscribe(s_client, s_topics[i].set_topic, 0);
            if (AMARAN_LIGHTS[i].has_gm) {
                esp_mqtt_client_subscribe(s_client, s_topics[i].gm_set_topic, 0);
            }
            publish_state(i);
        }
        amaran_mqtt_publish_diag();             /* LOCAL EDIT (2026-08-24) */
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        s_connected = false;
        break;
    case MQTT_EVENT_DATA: {
        for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
            int tlen = strlen(s_topics[i].set_topic);
            if (event->topic_len == tlen &&
                memcmp(event->topic, s_topics[i].set_topic, tlen) == 0) {
                ESP_LOGI(TAG, "cmd %.*s: %.*s",
                         event->topic_len, event->topic,
                         event->data_len, event->data);
                handle_command_for(i, event->data, event->data_len);
                break;
            }
            int glen = strlen(s_topics[i].gm_set_topic);
            if (AMARAN_LIGHTS[i].has_gm && event->topic_len == glen &&
                memcmp(event->topic, s_topics[i].gm_set_topic, glen) == 0) {
                ESP_LOGI(TAG, "gm %.*s: %.*s",
                         event->topic_len, event->topic,
                         event->data_len, event->data);
                handle_gm_for(i, event->data, event->data_len);
                break;
            }
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;
    default:
        break;
    }
}

int amaran_mqtt_start(const amaran_mqtt_dispatch_t *dispatch)
{
    s_dispatch = dispatch;

    for (int i = 0; i < AMARAN_LIGHT_COUNT; i++) {
        snprintf(s_topics[i].set_topic, sizeof(s_topics[i].set_topic),
                 "%s/%s/set", MQTT_TOPIC_PREFIX, AMARAN_LIGHTS[i].key);
        snprintf(s_topics[i].state_topic, sizeof(s_topics[i].state_topic),
                 "%s/%s/state", MQTT_TOPIC_PREFIX, AMARAN_LIGHTS[i].key);
        snprintf(s_topics[i].gm_set_topic, sizeof(s_topics[i].gm_set_topic),
                 "%s/%s/gm/set", MQTT_TOPIC_PREFIX, AMARAN_LIGHTS[i].key);
        snprintf(s_topics[i].gm_state_topic, sizeof(s_topics[i].gm_state_topic),
                 "%s/%s/gm/state", MQTT_TOPIC_PREFIX, AMARAN_LIGHTS[i].key);
        snprintf(s_topics[i].avail_topic, sizeof(s_topics[i].avail_topic),
                 "%s/%s/availability", MQTT_TOPIC_PREFIX, AMARAN_LIGHTS[i].key);
        s_state[i].on = false;
        s_state[i].brightness = 100;
        s_state[i].color_temp_k = 4700;
        s_state[i].color_temp_mired = 1000000 / 4700;
        s_state[i].hue = 0;
        s_state[i].sat = 0;
        s_state[i].gm = 0;
        s_state[i].is_hs = false;   /* start in CCT mode */
        s_state[i].dirty = false;
        s_state[i].avail = true;    /* optimistic at boot, same as the mesh */
    }

    const esp_timer_create_args_t pub_args = {
        .callback = publish_flush_cb,
        .name = "amaran_mqtt_pub",
    };
    esp_timer_create(&pub_args, &s_publish_timer);

    /* LOCAL EDIT (2026-08-13): on/off verify-and-retry timer. */
    const esp_timer_create_args_t retry_args = {
        .callback = retry_timer_cb,
        .name = "amaran_mqtt_retry",
    };
    esp_timer_create(&retry_args, &s_retry_timer);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,
        /* LOCAL EDIT (2026-08-24): Last Will. Without this the entities have
         * no availability at all -- if the bridge falls off the network HA
         * keeps showing the last state forever and accepts clicks that go
         * nowhere. QoS 1 + retained so the broker's "offline" is delivered
         * and survives an HA restart. */
        .session.last_will = {
            .topic = AVAIL_TOPIC,
            .msg = PAYLOAD_OFFLINE,
            .msg_len = sizeof(PAYLOAD_OFFLINE) - 1,
            .qos = 1,
            .retain = 1,
        },
    };
    if (strlen(MQTT_USERNAME) > 0) {
        cfg.credentials.username = MQTT_USERNAME;
        cfg.credentials.authentication.password = MQTT_PASSWORD;
    }

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) return -1;
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
    return 0;
}

bool amaran_mqtt_is_connected(void)
{
    return s_connected;
}
