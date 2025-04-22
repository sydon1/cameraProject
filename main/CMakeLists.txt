idf_component_register(SRCS "cJSON.c" "main.c" "mirf.c"
                        PRIV_INCLUDE_DIRS .
                        PRIV_REQUIRES nvs_flash esp_psram esp_wifi esp_http_server esp32-camera esp_timer) 