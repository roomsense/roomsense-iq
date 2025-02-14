idf_component_register(
	SRCS "ha_mqtt.c" "esp_device.c" "esp_discovery.c" "esp_serial.c"
	INCLUDE_DIRS "include" "../../main"
	REQUIRES hal nvs_flash json esp-tls mqtt ld2410 adafruit-161 pir-sensor direction-detection
)
