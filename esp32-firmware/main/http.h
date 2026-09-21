/*
 * Tiny HTTP REST API mirroring the UART CLI:
 *   GET  /status                    — LOCAL EDIT (2026-08-24): health snapshot
 *   GET  /lights
 *   POST /lights/<key>/on
 *   POST /lights/<key>/off
 *   POST /lights/<key>/brightness   { "value": 0-100 }
 *   POST /lights/<key>/cct          { "kelvin": int, "intensity": 0-100, "gm": -50..50 }
 *   POST /lights/<key>/hsi          { "hue": 0-360, "sat": 0-100, "intensity": 0-100 }
 *
 * <key> may also be "all" / "broadcast" for group sends.
 */

#ifndef AMARAN_HTTP_H
#define AMARAN_HTTP_H

#include "mqtt.h"  /* reuse the same dispatch struct */

#ifdef __cplusplus
extern "C" {
#endif

int amaran_http_start(const amaran_mqtt_dispatch_t *dispatch);

#ifdef __cplusplus
}
#endif

#endif /* AMARAN_HTTP_H */
