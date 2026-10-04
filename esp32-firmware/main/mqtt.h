/*
 * MQTT client + Home Assistant discovery.
 *
 * On connect: publishes one HA MQTT-discovery message per fixture under
 *   <ha_prefix>/light/<device_id>_<key>/config
 * with brightness + CCT support, then subscribes to the per-fixture
 * command topic. Each incoming command is parsed as JSON and dispatched
 * to the mesh-send callback registered here.
 */

#ifndef AMARAN_MQTT_H
#define AMARAN_MQTT_H

#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"  /* LOCAL EDIT #10 */

#ifdef __cplusplus
extern "C" {
#endif

/* Mesh command dispatch — provided by main.c. The MQTT layer calls these
 * after parsing an incoming HA command payload. */
typedef struct {
    void (*on_off)(uint16_t dst, bool on);
    void (*brightness)(uint16_t dst, int pct_0_100);
    /* gm: green/magenta tint, -50..+50, 0 = neutral. */
    void (*cct)(uint16_t dst, int kelvin, int pct_0_100, int gm);
    void (*hsi)(uint16_t dst, int hue_0_360, int sat_0_100, int pct_0_100);
    void (*refresh)(uint16_t dst);
} amaran_mqtt_dispatch_t;

int amaran_mqtt_start(const amaran_mqtt_dispatch_t *dispatch);
bool amaran_mqtt_is_connected(void);

/* Report fixture state observed on the mesh (decoded from a 0x26 status
 * reply — e.g. a change made from the desktop / iOS app or a physical
 * knob). Updates HA's view only when a field differs materially from what
 * we last commanded, so the echo of our own commands doesn't churn HA.
 * brightness 0..100, cct in kelvin, gm -10..+10, hue 0..360, sat 0..100. */
void amaran_mqtt_report_state(uint16_t addr, bool on, bool is_hs,
                              int brightness_0_100, int cct_kelvin, int gm,
                              int hue_0_360, int sat_0_100);

/* ---- LOCAL EDIT (2026-08-24): liveness + diagnostics -------------------
 *
 * Background: on 2026-08-24 the bridge sat with a healthy Wi-Fi link and a
 * healthy MQTT session while the BLE mesh side was completely dead. Every HA
 * command was received, echoed back on the state topic, and silently dropped
 * on the floor -- so HA showed a normal, responsive-looking light that did
 * nothing, for hours, with not one line logged. The firmware could detect a
 * status reply that CONTRADICTED a command (the 2026-08-13 verify-and-retry)
 * but had no way to notice the total ABSENCE of replies, because that retry
 * path only runs from the mesh receive path.
 *
 * Two availability topics, deliberately separate:
 *
 *   <prefix>/bridge/availability  — the bridge process is alive and on the
 *                                   broker. Backed by an MQTT Last Will, so
 *                                   the broker publishes "offline" for us if
 *                                   we drop off without saying goodbye.
 *   <prefix>/bridge/mesh          — the fixtures are answering on the mesh.
 *
 * Lights and G/M numbers require BOTH (availability_mode "all") so either
 * failure greys them out in HA. The diagnostic sensors require only the
 * first, because a dead mesh is exactly when you want to still read them.
 */

/* Total decoded mesh status replies since boot. The poll sweep compares this
 * across an interval to decide whether the mesh is answering at all. */
uint32_t amaran_mqtt_mesh_reply_count(void);

/* Declare the mesh reachable/unreachable. Idempotent -- only publishes (and
 * logs) on a transition. */
void amaran_mqtt_set_mesh_alive(bool alive);
bool amaran_mqtt_mesh_is_alive(void);

/* Publish the diagnostic JSON blob (uptime, heap, RSSI, counters, seconds of
 * mesh silence). Called from the poll task. */
void amaran_mqtt_publish_diag(void);

/* Re-evaluate per-fixture availability from time-since-last-reply and publish
 * any changes. Called once per poll sweep. Separate from the whole-mesh signal:
 * the fixtures are on different outlets, so one going dark is a real and
 * distinct condition from the mesh dying. */
void amaran_mqtt_update_fixture_availability(void);

/* Unconditionally republish every availability topic (bridge, mesh, and all
 * per-fixture). The transitions are
 * edge-triggered, so without a periodic restatement a retained value that ever
 * drifted from what the bridge believes would never be corrected -- the
 * entities would sit unavailable with a perfectly healthy bridge. Two tiny
 * retained messages a minute buys a self-healing contract. */
void amaran_mqtt_republish_availability(void);

/* Microseconds since boot at the last decoded mesh reply, 0 if never. */
int64_t amaran_mqtt_last_mesh_reply_us(void);

/* Lifetime count of MQTT (re)connections. */
uint32_t amaran_mqtt_connect_count(void);

/* Per-fixture reply accounting, for /status -- distinguishes "one fixture is
 * deaf" from "the whole mesh is deaf". idx is an AMARAN_LIGHTS index. */
void amaran_mqtt_fixture_stats(int idx, uint32_t *replies, int64_t *last_us,
                               bool *available);

/* ---- LOCAL EDIT #8 (2026-09-06): transmit-side instrumentation ----------
 *
 * Three mesh-death incidents (2026-08-24, 08-31, 09-06) share one fingerprint:
 * the BLE side goes silent while Wi-Fi and MQTT stay provably healthy, heap
 * stays flat, and `mesh_silence` is pinned at its normal value until the
 * instant it isn't -- no precursor, no degradation.
 *
 * The reason none of that identifies the fault is that every counter we keep
 * measures RECEIVE. `mesh_replies` counts what came back. So "the mesh went
 * deaf" is an assumption, and it hides three different bugs:
 *
 *   send returns non-OK          -> the mesh layer is refusing to transmit
 *   send OK but no SEND_COMP     -> the BT controller is wedged
 *   SEND_COMP fine, zero replies -> the rx path is deaf, or the fixtures
 *                                   genuinely stopped answering
 *
 * Those have different fixes, and today we cannot tell them apart. The stack
 * already reports both halves -- `esp_ble_mesh_server_model_send_msg` returns
 * an esp_err_t that the caller discarded, and ESP_BLE_MESH_MODEL_SEND_COMP_EVT
 * already arrives with an err_code that was only logged. Counting them costs
 * nothing and rides the existing 60 s diagnostic publish, so the answer lands
 * in Home Assistant's recorder continuously rather than having to be caught
 * live during a three-minute outage. */
/* Previous-boot reset reason and BT controller state, as short words. Shared
 * by the MQTT diagnostic blob and GET /status. */
const char *amaran_reset_reason_name(void);
const char *amaran_bt_controller_status_name(void);

/* ---- LOCAL EDIT #9 (2026-09-06): deaf-mesh failure injection -------------
 *
 * The recovery path (capture -> reboot -> lights return -> notify) could not be
 * tested end to end, because the fault it recovers from has no known trigger.
 * Cutting power to the fixtures is not the same shape: the reboot cannot revive
 * unpowered lights, so it only ever exercises the "still down" branch.
 *
 * This drops incoming mesh replies for a bounded window while the fixtures stay
 * powered and the bridge keeps transmitting normally. That is exactly the
 * "rx path deaf" shape -- one of the three faults the tx counters were added to
 * tell apart -- so `mesh_sends` and `mesh_send_completions` keep climbing while
 * `mesh_replies` flatlines, mesh silence grows, and every downstream step runs
 * for real. Self-clearing on expiry AND on reboot, so a failed test cannot
 * leave the bridge wedged. */
void amaran_mqtt_simulate_deaf(int seconds);

/* LOCAL EDIT #11 (2026-09-28, made permanent 2026-10-04) -- adv buffer
 * alloc race. ESP-IDF v5.3.2 net_buf_alloc() let two tasks claim the same adv
 * buffer, so the pool's free count leaked ~11/day to 0 (-> -ENOBUFS -> mesh
 * mute every ~6.7 d). Fixed by patches/0002 (backport of upstream 87c0b28459,
 * now unconditional). Diag keeps adv_buf_ref0 (buffers with ref==0, counted
 * directly) and adv_buf_ghost = ref0 - adv_buf_free as a standing tripwire:
 * ghosts should stay 0; any growth means the race (or a cousin) is back. */
int  amaran_mqtt_simulated_deaf_remaining_s(void);

void amaran_mqtt_note_mesh_tx(int err);
/* LOCAL EDIT #10 (2026-09-21): `dst` is the destination of the completed
 * send, so a failure that concentrates on one fixture is visible. */
void amaran_mqtt_note_mesh_send_comp(int err_code, uint16_t dst);
void amaran_mqtt_mesh_tx_stats(uint32_t *tx, uint32_t *tx_err,
                               int32_t *last_tx_err, uint32_t *comp,
                               uint32_t *comp_err, int32_t *last_comp_err);

/* LOCAL EDIT #10 (2026-09-21) -- WHICH error, and the pool it would drain.
 *
 * Incidents 4+5 (2026-09-12, 09-19) proved the bridge goes MUTE, not deaf:
 * every SEND_COMP_EVT carries an err_code after death, and the error rate
 * ramps for 8-19 h beforehand after ~5.5 days at exactly zero. In IDF 5.3 that
 * err_code is bt_mesh_model_send()'s synchronous return -- the transport
 * refusing to queue, not the radio failing. Every send here is SEGMENTED
 * (3-byte vendor opcode + 10-byte payload > 11-byte unsegmented max) and the
 * build allows ONE segmented tx context, so the two live suspects have
 * distinct codes:
 *   -EBUSY   (-16)  "No multi-segment message contexts available" -- the one
 *                   seg_tx slot held busy (stuck, or awaiting a slow ack)
 *   -ENOBUFS (-105) "Out of network buffers" -- the 60-buffer adv pool drained
 * Counting each code, and publishing the adv pool's free count continuously,
 * names the mechanism at the next incident and shows a leak DAYS early. */
void amaran_mqtt_add_tx_diag(cJSON *obj);

#ifdef __cplusplus
}
#endif

#endif /* AMARAN_MQTT_H */
