idf_component_register(SRCS "ha_mqtt.c" "esp_device.c" "esp_discovery.c" "esp_serial.c" 
                    INCLUDE_DIRS "include" "../../main" REQUIRES nvs_flash "json")
