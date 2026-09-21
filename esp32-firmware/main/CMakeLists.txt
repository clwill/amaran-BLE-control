idf_component_register(
    SRCS
        "main.c"
        "telink.c"
        "wifi.c"
        "mqtt.c"
        "http.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS
        # Internal BLE Mesh headers, used to self-provision with pre-known
        # keys (bt_mesh_provision / bt_mesh_node_local_app_key_add /
        # bt_mesh_node_bind_app_key_to_model). The public API doesn't expose
        # a "load keys without provisioning" path, so we link against these
        # directly.
        "${IDF_PATH}/components/bt/esp_ble_mesh/core/include"
        "${IDF_PATH}/components/bt/esp_ble_mesh/core"
        "${IDF_PATH}/components/bt/esp_ble_mesh/common/include"
    REQUIRES
        bt
        driver
        nvs_flash
        esp_event
        esp_timer
        esp_wifi
        esp_netif
        mdns
        mqtt
        esp_http_server
        json
)
