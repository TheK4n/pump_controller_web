idf_component_register(SRCS "main.c"
                        PRIV_REQUIRES esp_wifi esp_http_server nvs_flash esp_driver_gpio esp_adc cjson
                       INCLUDE_DIRS ".")
