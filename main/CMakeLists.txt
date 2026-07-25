idf_component_register(SRCS "main.cpp" "OTA_Manager.cpp" "SpindleControl.cpp" "StatusLEDs.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_netif esp_event esp_wifi esp_http_server esp_timer app_update led_strip esp_driver_gpio esp_driver_rmt)