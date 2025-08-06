#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "freertos/event_groups.h"
#include <cJSON.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include "tasks_common.h"
#include "ha_mqtt.h"
#include "app_nvs.h"
#include "http_server.h"
#include "esp_timer.h"

#include "rgb_led.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_discovery.h"
#include "wifi_app.h"

#include "ld2410.h"
#include "pir-sensor.h"
#include "adafruit-161.h"
#include "direction-detection.h"

#include "esp_tls.h"
#include "mqtt_client.h"

#include "hal/ledc_types.h"
#include "hal/gpio_types.h"

#define PIR_QOS       2
#define T_H_QOS       0
#define DISTANCE_QOS  0
#define SETTINGS_QOS  0
#define SUBSCRIBE_QOS 2
#define DISCOVERY_QOS 2
#define OCCUPANCY_QOS 2
#define DIRECTION_QOS 0
#define PHOTOCELL_QOS 0

#define CONFIG_MQTT_URI    "mqtt://homeassistant.local:1883"   //"mqtt://10.0.0.203:1883"

#define I2C_BUS 0
#define I2C_SCL_PIN 14
#define I2C_SDA_PIN 13

/*
 * Description of input values
 * brightness: 1..255
 * color_temp: 153..500
 * color: {"r": 0, "g": 255, "b": 0}
 */

#define DEFAULT_BUF_SIZE 4096

#define GPIO_RED_IO GPIO_NUM_27
#define GPIO_GREEN_IO GPIO_NUM_26
#define GPIO_BLUE_IO GPIO_NUM_27
#define LED_FADE_MS 300
#define LED_USE_FADE

#define OCCUPANCY_STATE_MACHINE  100
#define OCCUPANCY_PUBLISH_RATE_MSEC  200
#define DISTANCE_PUBLISH_RATE_MSEC  1000
#define T_H_PUBLISH_RATE_MSEC   2000//30000
#define PHOTOCELL_PUBLISH_RATE_MSEC   2000

#define DHT11_PIN GPIO_NUM_4

#ifdef LED_HAS_WHITE
#define GPIO_N_WHITE_IO GPIO_NUM_23
#define GPIO_W_WHITE_IO GPIO_NUM_22
#endif

#define STORAGE_NAMESPACE "storage"
#define STORAGE_LED_KEY "led"
#define STORAGE_SERIAL_KEY "serial"
#define STORAGE_SERIAL_LENGTH 6

#define PERSIST_DELAY 60000

//extern bool losing_qos;

static EventGroupHandle_t mqtt_event_group, env_event_group, esp_event_group, persist_event_group, discovery_event_group;
static EventGroupHandle_t occupancy_group;

static esp_mqtt_client_handle_t client = NULL;

static const int CONNECTED_BIT = BIT0;
static const int DISCOVERY_BIT = BIT1;
static const int OCCUPANCY_BIT = BIT2;
extern const int MOVEMENT_DIRECTION_BIT;
extern const int PIR_BIT;

static const char *TAG = "MQTT";

#ifdef LED_HAS_WHITE
static const double min_white = 153;
static const double max_white = 500;
static const double wide = max_white - min_white;
#endif

extern EventGroupHandle_t movement_direction_group;
extern EventGroupHandle_t pir_group;

extern EventGroupHandle_t led_event_group;
extern const int RGB_LED_BIT;

extern const uint8_t ca_cert_pem_start[] asm("_binary_ca_crt_start");
extern const uint8_t ca_cert_pem_end[] asm("_binary_ca_crt_end");

const char *STATE_POWER_ON = "ON";
const char *STATE_POWER_OFF = "OFF";

static const char *STATUS_ONLINE = "online";
static const char *STATUS_OFFLINE = "offline";
static char bedsense_switch_status[10];
static char calsense_switch_status[10];

static bool g_mqtt_connection_status = false;

//These global variables are for that a mqtt message published if there is change in the settings

bool start_stop_status_changed = 0;

bool g_calsense_changed = 0;
//bool g_calsense_status = 0;

extern uint16_t raw_detected_distance;

static bool start_stop_status = 1;

extern SemaphoreHandle_t xMutex_g_ld2410_data;
extern SemaphoreHandle_t xMutex_g_light_density_raw;


mqtt_credentials_t *mqtt_credentials = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t mqtt_app_queue_handle;

/**
 * MQTT application event group handle and status bits
 */
static EventGroupHandle_t mqtt_app_event_group;
const int MQTT_APP_CONNECTING_USING_SAVED_CREDS_BIT = BIT0;
const int MQTT_APP_CONNECTING_FROM_HTTP_SERVER_BIT = BIT1;
const int MQTT_APP_USER_REQUESTED_DISCONNECT_BIT = BIT2;
const int MQTT_APP_CONNECTED_BIT = BIT3;

const char *default_state_led = "{\"state\":\"ON\",\"brightness\":255,\"color_temp\":153,\"color\":{\"r\":255,\"g\":255,\"b\":255}}";

static uint8_t serial_data[STORAGE_SERIAL_LENGTH];

esp_serial_t esp_serial =
{ .namespace = STORAGE_NAMESPACE, .key = STORAGE_SERIAL_KEY, .data = serial_data, .length = STORAGE_SERIAL_LENGTH };

esp_device_t esp_device;
//{.manufacturer = "roomsenselabs", .model = "roomsenseiq", .sw_version = "1.0", .idf_version = "4.4", };

esp_discovery_button_t esp_discovery_button_start =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-start", };

esp_discovery_button_t esp_discovery_button_read =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-read", };

esp_discovery_button_t esp_discovery_button_config =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-config", };

esp_discovery_button_t esp_discovery_button_reset =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-reset", };

esp_discovery_switch_t esp_discovery_switch_bedsense =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-bedsense", };

esp_discovery_switch_t esp_discovery_switch_calsense =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-calsense", };

esp_discovery_number_t esp_discovery_number_pir_sensitivity =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"box"
		.device_tag = "roomsense-pirsensitivity", .min = 1, .max = 3, .step = 1, };

esp_discovery_number_t esp_discovery_number_max_macro_range =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"box"
		.device_tag = "roomsense-maxmacrorange", .min = 0, .max = 600, .step = 75, };

esp_discovery_number_t esp_discovery_number_max_micro_range =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"box"
		.device_tag = "roomsense-maxmicrorange", .min = 0, .max = 600, .step = 75, };

esp_discovery_number_t esp_discovery_number_timeout =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-timeout", .min = 0, .max = 300, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_0 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold0", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_1 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold1", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_2 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold2", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_3 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold3", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_4 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold4", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_5 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold5", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_6 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold6", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_7 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold7", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_macro_threshold_8 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-macrothreshold8", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_0 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold0", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_1 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold1", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_2 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold2", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_3 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold3", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_4 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold4", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_5 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold5", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_6 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold6", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_7 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold7", .min = 0, .max = 100, .step = 1, };

esp_discovery_number_t esp_discovery_number_micro_threshold_8 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "slider", //"slider"
		.device_tag = "roomsense-microthreshold8", .min = 0, .max = 100, .step = 1, };

esp_discovery_sensor_t esp_discovery_temperature =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "Temperature-C", .unit_of_measurement = "°C" };

esp_discovery_sensor_t esp_discovery_temperature_f =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "Temperature-F", .unit_of_measurement = "°F" };

esp_discovery_sensor_t esp_discovery_dew_point =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "Dew-Point", .unit_of_measurement = "°C" };

esp_discovery_sensor_t esp_discovery_humidity =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "Humidity", .unit_of_measurement = "%" };

esp_discovery_sensor_t esp_discovery_co2 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "CO2", .unit_of_measurement = "ppm" };

esp_discovery_sensor_t esp_discovery_voc =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "TVOC", .unit_of_measurement = "index" };

esp_discovery_sensor_t esp_discovery_pm1 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "PM1", .unit_of_measurement = "µg/m³" };

esp_discovery_sensor_t esp_discovery_pm25 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "PM2-5", .unit_of_measurement = "µg/m³" };

esp_discovery_sensor_t esp_discovery_pm10 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "PM10", .unit_of_measurement = "µg/m³" };

esp_discovery_sensor_t esp_discovery_co =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "CO", .unit_of_measurement = "ppm" };

esp_discovery_light_sensor_t esp_discovery_light_sensor =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "Light", .unit_of_measurement = "Raw"};

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin0 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin0", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin1 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin1", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin2 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin2", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin3 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin3", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin4 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin4", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin5 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin5", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin6 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin6", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin7 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin7", };

esp_discovery_macro_move_sensor_t esp_discovery_macro_move_bin8 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-macrobin8", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin0 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin0", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin1 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin1", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin2 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin2", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin3 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin3", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin4 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin4", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin5 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin5", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin6 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin6", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin7 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin7", };

esp_discovery_micro_move_sensor_t esp_discovery_micro_move_bin8 =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "roomsense-microbin8", };

esp_discovery_sensor_t esp_discovery_room_presence =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt_room", .schema = "json", .device_tag = "Presence", };

esp_discovery_text_t esp_discovery_movement_direction =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "Movement-Direction", };

esp_discovery_text_t esp_discovery_location =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "Location", };

//esp_discovery_text_t esp_discovery_humidity_status =
//{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "humidity-status", };
//
//esp_discovery_text_t esp_discovery_co2_status =
//{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "co2-status", };
//
//esp_discovery_text_t esp_discovery_voc_status =
//{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "voc-status", };
//
esp_discovery_text_t esp_discovery_co_status =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "co-status", };

esp_discovery_text_t esp_discovery_pm_status =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "pm-status", };

//esp_discovery_text_t esp_discovery_mold_risk =
//{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .mode = "text", .device_tag = "mold-risk", };

esp_discovery_sensor_t esp_discovery_distance_detected =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt_room", .schema = "json", .device_tag = "Distance-cm", .unit_of_measurement = "cm"};

esp_discovery_sensor_t esp_discovery_distance_detected_ft =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt_room", .schema = "json", .device_tag = "Distance-ft", .unit_of_measurement = "ft"};

esp_discovery_room_presence_t esp_discovery_start_stop_status =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt_room", .schema = "json", .device_tag = "start_stop_status", };

esp_discovery_pir_sensor_t esp_discovery_pir_sensor =
{ .esp_serial = &esp_serial, .esp_device = &esp_device, .platform = "mqtt", .schema = "json", .device_tag = "PIR", };

ledc_timer_config_t ledc_timer =
{ .duty_resolution = LEDC_TIMER_8_BIT, .freq_hz = 5000, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0, .clk_cfg = LEDC_AUTO_CLK, };

ledc_channel_config_t ledc_red_channel =
{ .channel = LEDC_CHANNEL_2, .duty = 0, .gpio_num = GPIO_RED_IO, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_0, .intr_type =
		LEDC_INTR_FADE_END };

ledc_channel_config_t ledc_green_channel =
{ .channel = LEDC_CHANNEL_3, .duty = 0, .gpio_num = GPIO_GREEN_IO, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_0, .intr_type =
		LEDC_INTR_FADE_END };

ledc_channel_config_t ledc_blue_channel =
{ .channel = LEDC_CHANNEL_4, .duty = 0, .gpio_num = GPIO_BLUE_IO, .speed_mode = LEDC_LOW_SPEED_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER_0, .intr_type =
		LEDC_INTR_FADE_END };

mqtt_credentials_t* get_mqtt_credentials(void)
{
	return mqtt_credentials;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
	char *topic, *data;
	uint16_t num;

	switch (event->event_id)
	{
	case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

//		esp_mqtt_client_subscribe(event->client, esp_discovery_button_start.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_subscribe(event->client, esp_discovery_button_read.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_subscribe(event->client, esp_discovery_button_config.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_subscribe(event->client, esp_discovery_button_reset.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_subscribe(event->client, esp_discovery_switch_bedsense.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(50 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_subscribe(event->client, esp_discovery_switch_calsense.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(50 / portTICK_PERIOD_MS);

//		esp_mqtt_client_subscribe(event->client, esp_discovery_movement_direction.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(5 / portTICK_PERIOD_MS);

//		esp_mqtt_client_publish(event->client, esp_discovery_button_start.status_topic, STATUS_ONLINE, 0, 2, true);
//		esp_mqtt_client_publish(event->client, esp_discovery_button_read.status_topic, STATUS_ONLINE, 0, 2, true);
//		esp_mqtt_client_publish(event->client, esp_discovery_button_config.status_topic, STATUS_ONLINE, 0, 2, true);
//		esp_mqtt_client_publish(event->client, esp_discovery_button_reset.status_topic, STATUS_ONLINE, 0, 2, true);

//		esp_mqtt_client_subscribe(event->client, esp_discovery_number_pir_sensitivity.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(5 / portTICK_PERIOD_MS);

//		esp_mqtt_client_subscribe(event->client, esp_discovery_number_max_macro_range.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_subscribe(event->client, esp_discovery_number_max_micro_range.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(5 / portTICK_PERIOD_MS);

//		esp_mqtt_client_subscribe(event->client, esp_discovery_number_timeout.set_topic, SUBSCRIBE_QOS);
//		vTaskDelay(5 / portTICK_PERIOD_MS);

//		esp_mqtt_client_publish(event->client, esp_discovery_button_start.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_button_read.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_button_config.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_button_reset.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_switch_bedsense.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true); //STATE_POWER_ON
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_switch_calsense.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_number_pir_sensitivity.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_number_max_macro_range.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_number_max_micro_range.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_number_timeout.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_distance_detected.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_distance_detected_ft.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_humidity.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_light_sensor.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_temperature.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_temperature_f.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		vTaskDelay(100 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_dew_point.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_co2.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_voc.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_pm1.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_pm25.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_pm10.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_co.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_pir_sensor.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_room_presence.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_movement_direction.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_location.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

//		esp_mqtt_client_publish(event->client, esp_discovery_humidity_status.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		//vTaskDelay(500 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_co2_status.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		//vTaskDelay(500 / portTICK_PERIOD_MS);
//
//		esp_mqtt_client_publish(event->client, esp_discovery_voc_status.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(500 / portTICK_PERIOD_MS);
//
		esp_mqtt_client_publish(event->client, esp_discovery_pm_status.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

		esp_mqtt_client_publish(event->client, esp_discovery_co_status.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

//		esp_mqtt_client_publish(event->client, esp_discovery_mold_risk.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
		//vTaskDelay(500 / portTICK_PERIOD_MS);

//		esp_mqtt_client_publish(event->client, esp_discovery_start_stop_status.status_topic, STATUS_ONLINE, 0, DISCOVERY_QOS, true);
//		vTaskDelay(5 / portTICK_PERIOD_MS);



		xEventGroupSetBits(mqtt_event_group, CONNECTED_BIT);
		xEventGroupSetBits(discovery_event_group, CONNECTED_BIT);
		xEventGroupSetBits(env_event_group, CONNECTED_BIT);
		xEventGroupSetBits(esp_event_group, CONNECTED_BIT);
		mqtt_app_send_message(MQTT_APP_MSG_CONNECTED);
		break;
	case MQTT_EVENT_DISCONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");

		xEventGroupClearBits(mqtt_event_group, CONNECTED_BIT);
		mqtt_app_send_message(MQTT_APP_MSG_DISCONNECTED);
		break;

	case MQTT_EVENT_ERROR:
		ESP_LOGE(TAG, "MQTT_EVENT_ERROR");

		// Read the error code
		esp_mqtt_error_type_t err_code = event->error_handle->error_type;
		esp_mqtt_connect_return_code_t connect_return_code = event->error_handle->connect_return_code;
		esp_err_t esp_tls_last_esp_err = event->error_handle->esp_tls_last_esp_err;
		int esp_tls_stack_err = event->error_handle->esp_tls_stack_err;
		int esp_tls_cert_verify_flags = event->error_handle->esp_tls_cert_verify_flags;
		int esp_transport_sock_errno = event->error_handle->esp_transport_sock_errno;

		ESP_LOGE(TAG, "MQTT Error code: %d", err_code);
		ESP_LOGE(TAG, "connect_return_code: %d", connect_return_code);
		ESP_LOGE(TAG, "esp_tls_last_esp_err: %d", esp_tls_last_esp_err);
		ESP_LOGE(TAG, "esp_tls_stack_err: %d", esp_tls_stack_err);
		ESP_LOGE(TAG, "esp_tls_cert_verify_flags: %d", esp_tls_cert_verify_flags);
		ESP_LOGE(TAG, "esp_transport_sock_errno: %d", esp_transport_sock_errno);

		if (esp_transport_sock_errno == 11 || esp_transport_sock_errno == 23)
		{
			mqtt_app_send_message(MQTT_APP_MSG_RESTART_CPU);
		}

		break;
	case MQTT_EVENT_DATA:
		ESP_LOGI(TAG, "MQTT_EVENT_DATA");

		// Short-circuit this entirely for now, as we're not subscribed to any topics and so none of the below set_topics are even allocated.
		break;

		// Check to make sure required fields contain data.
		if (event->topic == NULL || event->topic_len == 0 || event->data == NULL || event->data_len == 0) {
			ESP_LOGE(TAG, "topic and/or data are empty; discarding message.");
			break;
		}

		if ((topic = (char *)calloc(event->topic_len + 1, 1)) == NULL) {
			ESP_LOGE(TAG, "calloc of %i bytes for MQTT topic failed", event->topic_len + 1);
			break;
		}

		if ((data = (char *)calloc(event->data_len + 1, 1)) == NULL) {
			free(topic);
			ESP_LOGE(TAG, "calloc of %i bytes for MQTT data failed", event->data_len + 1);
			break;
		}

		// We're expecting null-terminated strings in the comparisons below; so, copy the MQTT values into the freshly allocated (and zeroed!) buffers.
		memcpy(topic, event->topic, event->topic_len);
		memcpy(data, event->data, event->data_len);

		//ESP_LOGI(TAG, ">>>>>----------------------MQTT_EVENT_DATA----------------------->>>>[MQTT] Data %s with %s", topic, data);

		if (strcmp(topic, esp_discovery_button_start.set_topic) == 0)
		{
			start_stop_status = toggle_value(start_stop_status);
			start_stop_status_changed = true;
			if (start_stop_status)
			{
				ESP_LOGI(TAG, "start");
			}

			else
			{
				//ld2410_app_send_message(LD2410_APP_MSG_CONNECT);
				ESP_LOGI(TAG, "stop");
			}

		}

		else if (strcmp(topic, esp_discovery_button_read.set_topic) == 0)
		{
			//ESP_LOGI(TAG, "[BUTTON] ------Button Read-------%s--------", data);
			roomsense_iq_shared.buttons_shared.g_button_get_config = true;
			//ld2410_app_send_message(LD2410_APP_MSG_GET_CONFIG);
		}

		else if (strcmp(topic, esp_discovery_button_config.set_topic) == 0)
		{
			//ESP_LOGI(TAG, "[BUTTON] ------Button Config-------%s--------", data);
			roomsense_iq_shared.buttons_shared.g_button_set_config = true;
			roomsense_iq_shared.buttons_shared.g_button_config_to_pir = true;
			//ld2410_app_send_message(LD2410_APP_MSG_SET_CONFIG);
		}

		else if (strcmp(topic, esp_discovery_button_reset.set_topic) == 0)
		{
			//ESP_LOGI(TAG, "[BUTTON] ------Reset-------%s--------", data);
			roomsense_iq_shared.buttons_shared.g_button_factory_reset = true;
		}

		else if (strcmp(topic, esp_discovery_switch_bedsense.set_topic) == 0)
		{
			memset(bedsense_switch_status, 0, sizeof(bedsense_switch_status));
			strncpy(bedsense_switch_status, data, sizeof(bedsense_switch_status) - 1);
			if (strcmp(bedsense_switch_status, "ON") == 0)
			{
				roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status = 1;
			}
			else
			{
				roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status = 0;
			}
			roomsense_iq_shared.ha_mqtt_shared.g_bedsense_changed = 1;
		}

		else if (strcmp(topic, esp_discovery_switch_calsense.set_topic) == 0)
		{
			memset(calsense_switch_status, 0, sizeof(calsense_switch_status));
			strncpy(calsense_switch_status, data, sizeof(calsense_switch_status) - 1);
			if (strcmp(calsense_switch_status, "ON") == 0)
			{
				roomsense_iq_shared.ha_mqtt_shared.g_calsense_status = 1;
			}
			if (strcmp(calsense_switch_status, "OFF") == 0)
			{
				roomsense_iq_shared.ha_mqtt_shared.g_calsense_status = 0;
			}
			g_calsense_changed = 1;
		}

		else if (strcmp(topic, esp_discovery_number_pir_sensitivity.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.pir_sensor_shared.pir_sensitivity = (pir_sensitivity_e) num;
			roomsense_iq_shared.ha_mqtt_shared.g_pir_sensitivity_changed = 1;
		}

		else if (strcmp(topic, esp_discovery_number_max_macro_range.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.max_macro_range = (uint16_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_max_macro_range_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------Number--macro range-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_max_micro_range.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.max_micro_range = (uint16_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_max_micro_range_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------Number--micro range-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_timeout.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.absence_time_out = (uint16_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_timeout_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------timeout-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_0.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[0] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_0-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_1.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[1] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_1-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_2.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[2] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_2-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_3.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[3] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_3-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_4.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[4] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_4-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_5.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[5] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_5-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_6.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[6] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_6-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_7.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[7] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_7-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_macro_threshold_8.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.macro_threshold[8] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------macro threshold_8-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_0.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[0] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_0-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_1.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[1] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_1-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_2.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[2] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_2-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_3.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[3] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_3-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_4.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[4] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_4-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_5.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[5] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_5-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_6.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[6] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_6-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_7.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[7] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_7-----%d----------", num);
		}

		else if (strcmp(topic, esp_discovery_number_micro_threshold_8.set_topic) == 0)
		{
			num = atoi(data);
			roomsense_iq_shared.ld2410_config_shared.micro_threshold[8] = (uint8_t) num;
			roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
			//ESP_LOGI(TAG, "[Number] ------micro threshold_8-----%d----------", num);
		}

		free(topic);
		free(data);

		break;
	default:
		ESP_LOGV(TAG, "[MQTT] Event ID %d", event->event_id);

		break;
	}
	return ESP_OK;
}

void task_discovery_publish(void *param)
{
	char *json;

	while (true)
	{
		xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
		xEventGroupWaitBits(discovery_event_group, CONNECTED_BIT, true, true, portMAX_DELAY);
//		json = esp_discovery_serialize_button(&esp_discovery_button_start);
//
//		esp_mqtt_client_publish(client, esp_discovery_button_start.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_button(&esp_discovery_button_read);
//		esp_mqtt_client_publish(client, esp_discovery_button_read.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_button(&esp_discovery_button_config);
//		esp_mqtt_client_publish(client, esp_discovery_button_config.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_button(&esp_discovery_button_reset);
//		esp_mqtt_client_publish(client, esp_discovery_button_reset.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_switch(&esp_discovery_switch_bedsense);
//		esp_mqtt_client_publish(client, esp_discovery_switch_bedsense.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_switch(&esp_discovery_switch_calsense);
//		esp_mqtt_client_publish(client, esp_discovery_switch_calsense.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);

		// send discovery for number entities
//		json = esp_discovery_serialize_number(&esp_discovery_number_pir_sensitivity);
//		esp_mqtt_client_publish(client, esp_discovery_number_pir_sensitivity.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_number(&esp_discovery_number_max_macro_range);
//		esp_mqtt_client_publish(client, esp_discovery_number_max_macro_range.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_number(&esp_discovery_number_max_micro_range);
//		esp_mqtt_client_publish(client, esp_discovery_number_max_micro_range.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_number(&esp_discovery_number_timeout);
//		esp_mqtt_client_publish(client, esp_discovery_number_timeout.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);

		// send discovery for temperature
		json = esp_discovery_serialize_sensor(&esp_discovery_temperature);
		esp_mqtt_client_publish(client, esp_discovery_temperature.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_sensor(&esp_discovery_temperature_f);
		esp_mqtt_client_publish(client, esp_discovery_temperature_f.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		// send discovery for dew point
		json = esp_discovery_serialize_sensor(&esp_discovery_dew_point);
		esp_mqtt_client_publish(client, esp_discovery_dew_point.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		//send discovery for CO2
		json = esp_discovery_serialize_sensor(&esp_discovery_co2);
		esp_mqtt_client_publish(client, esp_discovery_co2.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_sensor(&esp_discovery_voc);
		esp_mqtt_client_publish(client, esp_discovery_voc.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_sensor(&esp_discovery_pm1);
		esp_mqtt_client_publish(client, esp_discovery_pm1.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_sensor(&esp_discovery_pm25);
		esp_mqtt_client_publish(client, esp_discovery_pm25.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_sensor(&esp_discovery_pm10);
		esp_mqtt_client_publish(client, esp_discovery_pm10.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_sensor(&esp_discovery_co);
		esp_mqtt_client_publish(client, esp_discovery_co.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		// send discovery for pir
		json = esp_discovery_serialize_pir_sensor(&esp_discovery_pir_sensor);
		esp_mqtt_client_publish(client, esp_discovery_pir_sensor.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		// send discovery for target status
		json = esp_discovery_serialize_sensor(&esp_discovery_room_presence);
		esp_mqtt_client_publish(client, esp_discovery_room_presence.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		// send discovery for movement direction
		json = esp_discovery_serialize_text(&esp_discovery_movement_direction);
		esp_mqtt_client_publish(client, esp_discovery_movement_direction.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_text(&esp_discovery_location);
		esp_mqtt_client_publish(client, esp_discovery_location.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

//		json = esp_discovery_serialize_text(&esp_discovery_humidity_status);
//		esp_mqtt_client_publish(client, esp_discovery_humidity_status.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_text(&esp_discovery_co2_status);
//		esp_mqtt_client_publish(client, esp_discovery_co2_status.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
//		json = esp_discovery_serialize_text(&esp_discovery_voc_status);
//		esp_mqtt_client_publish(client, esp_discovery_voc_status.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);
//
		json = esp_discovery_serialize_text(&esp_discovery_co_status);
		esp_mqtt_client_publish(client, esp_discovery_co_status.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		json = esp_discovery_serialize_text(&esp_discovery_pm_status);
		esp_mqtt_client_publish(client, esp_discovery_pm_status.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

//		json = esp_discovery_serialize_text(&esp_discovery_mold_risk);
//		esp_mqtt_client_publish(client, esp_discovery_mold_risk.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);

		vTaskDelay(100 / portTICK_PERIOD_MS);
		// send discovery distance_detected
		json = esp_discovery_serialize_sensor(&esp_discovery_distance_detected);
		esp_mqtt_client_publish(client, esp_discovery_distance_detected.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		vTaskDelay(100 / portTICK_PERIOD_MS);
		json = esp_discovery_serialize_sensor(&esp_discovery_distance_detected_ft);
		esp_mqtt_client_publish(client, esp_discovery_distance_detected_ft.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		// send discovery for esp_discovery_start_stop_status
//		json = esp_discovery_serialize_room_presence(&esp_discovery_start_stop_status);
//		esp_mqtt_client_publish(client, esp_discovery_start_stop_status.discovery_topic, json, 0, DISCOVERY_QOS, true);
//		free(json);

		// send discovery for humidity
		vTaskDelay(100 / portTICK_PERIOD_MS);
		json = esp_discovery_serialize_sensor(&esp_discovery_humidity);
		esp_mqtt_client_publish(client, esp_discovery_humidity.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		// send discovery for photocell
		vTaskDelay(100 / portTICK_PERIOD_MS);
		json = esp_discovery_serialize_sensor(&esp_discovery_light_sensor);
		esp_mqtt_client_publish(client, esp_discovery_light_sensor.discovery_topic, json, 0, DISCOVERY_QOS, true);
		free(json);

		vTaskDelay(5000 / portTICK_RATE_MS);
		xEventGroupSetBits(mqtt_event_group, DISCOVERY_BIT);
	}
}
void task_occupancy_publish(void *param)
{
	char sensor_data[20];

	//set the g_occupancy_status based on the status before restart.
	if (roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy)
	{
		roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status = OCCUPIED;
	}
	else
	{
		roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status = VACANT;
	}

	xEventGroupSetBits(occupancy_group, OCCUPANCY_BIT);

	while (true)
	{
		xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
		xEventGroupWaitBits(occupancy_group, OCCUPANCY_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
		sprintf(sensor_data, "%d", roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status);
		if (!g_ws_pause)
		{
			printf("occupancy status=%d\n", roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status);
			esp_mqtt_client_publish(client, esp_discovery_room_presence.state_topic, sensor_data, 0, OCCUPANCY_QOS, true);
		}
	}

}

void task_movement_direction_publish(void *param)
{
	char sensor_data[MAX_MQTT_PUBLISH_LENGTH];

	while (true)
	{
		xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
		xEventGroupWaitBits(movement_direction_group, MOVEMENT_DIRECTION_BIT, pdTRUE, pdTRUE, portMAX_DELAY);

		if (roomsense_iq_shared.direction_detection_shared.g_movement_direction == 0)
		{
			sprintf(sensor_data, "%s", "...");
		}

		if (roomsense_iq_shared.direction_detection_shared.g_movement_direction == 1)
		{
			sprintf(sensor_data, "%s", "Away");
		}

		if (roomsense_iq_shared.direction_detection_shared.g_movement_direction == 2)
		{
			sprintf(sensor_data, "%s", "Towards");
		}
		if (!g_ws_pause)
		{
			esp_mqtt_client_publish(client, esp_discovery_movement_direction.state_topic, sensor_data, 0, DIRECTION_QOS, true);
		}

		//sprintf(sensor_data, "%s", roomsense_iq_shared.location);
		if (!g_ws_pause)
		{
			//we also send the location entity. TBD: The value should set from the web interface
			esp_mqtt_client_publish(client, esp_discovery_location.state_topic, roomsense_iq_shared.location, 0, DIRECTION_QOS, true);
		}
	}

}

void task_distance_publish(void *param)
{
	char sensor_data[20];
	uint16_t distance_ft;

	while (true)
	{
		vTaskDelay(DISTANCE_PUBLISH_RATE_MSEC / portTICK_RATE_MS);

		xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

		if (!g_ws_pause)
		{
			sprintf(sensor_data, "%d", roomsense_iq_shared.ld2410_data_shared.detected_distance);
			esp_mqtt_client_publish(client, esp_discovery_distance_detected.state_topic, sensor_data, 0, DISTANCE_QOS, true);

			distance_ft = (uint16_t) roomsense_iq_shared.ld2410_data_shared.detected_distance / 30.48;
			sprintf(sensor_data, "%d", distance_ft);
			esp_mqtt_client_publish(client, esp_discovery_distance_detected_ft.state_topic, sensor_data, 0, DISTANCE_QOS, true);
		}
	}

}

void task_pir_publish(void *param)
{
	char sensor_data[10];

	while (true)
	{
		xEventGroupWaitBits(pir_group, PIR_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
		if (roomsense_iq_shared.pir_sensor_shared.g_pir_status && roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status == 0)
		{
			sprintf(sensor_data, "%s", "ON");
			if(roomsense_iq_shared.rgb_led_state == VACANT_LED) // It goes occupy only if the system's been in vacant mode.
			{
				roomsense_iq_shared.rgb_led_state = OCCUPIED_LED;
				xEventGroupSetBits(led_event_group, RGB_LED_BIT);
			}
		}
		else
		{
			sprintf(sensor_data, "%s", "OFF");
		}
		if (!g_ws_pause)
		{
			esp_mqtt_client_publish(client, esp_discovery_pir_sensor.state_topic, sensor_data, 0, PIR_QOS, true);
		}
	}
}

void task_climatesense_alarm_publish(void *param)
{
	char sensor_data[100];

	vTaskDelay(30000 / portTICK_RATE_MS); //Let the climatesense task finishes scanning all sensors

	while (true)
	{
	   xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

	   if (roomsense_iq_shared.climate_alarm == true && roomsense_iq_shared.climatesense_connection == true)
	   {
	       printf("-------> Alarm\n");
		   sprintf(sensor_data, "%s", roomsense_iq_shared.pm_status);
		   esp_mqtt_client_publish(client, esp_discovery_pm_status.state_topic, sensor_data, 0, PIR_QOS, true);
		   vTaskDelay(1000 / portTICK_RATE_MS);
		   sprintf(sensor_data, "%s", roomsense_iq_shared.co_status);
		   esp_mqtt_client_publish(client, esp_discovery_co_status.state_topic, sensor_data, 0, PIR_QOS, true);
	   }
	   else
	   {
		  sprintf(sensor_data, "%s", "No Alarm");
		  esp_mqtt_client_publish(client, esp_discovery_pm_status.state_topic, sensor_data, 0, PIR_QOS, true);
		  vTaskDelay(1000 / portTICK_RATE_MS);
		  sprintf(sensor_data, "%s", "No Alarm");
		  esp_mqtt_client_publish(client, esp_discovery_co_status.state_topic, sensor_data, 0, PIR_QOS, true);
	   }
	   vTaskDelay(4000 / portTICK_RATE_MS);
	}
}

void task_occupancy_state(void *param)
{
	char sensor_data[10];
	occupancy_state_e occupancy_state = VACANT;

//loads the occupancy state before restart
	occupancy_state = (occupancy_state_e) roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy;

//we reverse it so it goes through the state change and runs the related codes
	occupancy_state = !occupancy_state;

	while (true)
	{
		//The occupancy status gets updated as soon as there is a ap or wifi connection.
		if (roomsense_iq_shared.wifi_app_shared.g_wifi_connection_status == true || roomsense_iq_shared.wifi_app_shared.g_ap_connection_status ==true)
		{
		switch (occupancy_state)
		{
		case VACANT:
			if (roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status == 0)
			{
				if ((roomsense_iq_shared.pir_sensor_shared.g_pir_status == 1 && (roomsense_iq_shared.ld2410_data_shared.target_status == 1 || roomsense_iq_shared.ld2410_data_shared.target_status == 2 || roomsense_iq_shared.ld2410_data_shared.target_status == 3)) || roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy)
				//if (roomsense_iq_shared.pir_sensor_shared.g_pir_status == 1 || roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy)
				{
					occupancy_state = OCCUPIED;
					roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy = (uint8_t) occupancy_state;
					roomsense_iq_shared.rgb_led_state = OCCUPIED_LED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);
					roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status = OCCUPIED;
					xEventGroupSetBits(occupancy_group, OCCUPANCY_BIT);
				}
			}
			else //we are in the bedsense mode and should ignore the PIR signal
			{
				if ((roomsense_iq_shared.ld2410_data_shared.target_status == 1 || roomsense_iq_shared.ld2410_data_shared.target_status == 2
						|| roomsense_iq_shared.ld2410_data_shared.target_status == 3) || roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy)
				{
					occupancy_state = OCCUPIED;
					roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy = (uint8_t) occupancy_state;
					roomsense_iq_shared.rgb_led_state = OCCUPIED_LED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);
					roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status = OCCUPIED;
					xEventGroupSetBits(occupancy_group, OCCUPANCY_BIT);
				}
			}

			break;

		case OCCUPIED:
			if (roomsense_iq_shared.ha_mqtt_shared.g_bedsense_status == 0)
			{
				if ((roomsense_iq_shared.pir_sensor_shared.g_pir_status == 0 && roomsense_iq_shared.ld2410_data_shared.target_status == 0)
						|| roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy == 0)
				{
					occupancy_state = VACANT;
					roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy = (uint8_t) occupancy_state;
					roomsense_iq_shared.rgb_led_state = VACANT_LED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);
					roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status = VACANT;
					xEventGroupSetBits(occupancy_group, OCCUPANCY_BIT);
				}
			}
			else //we are in the bedsense mode and should ignore the PIR signal
			{
				if (roomsense_iq_shared.ld2410_data_shared.target_status == 0 || roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy == 0)
				{
					occupancy_state = VACANT;
					roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy = (uint8_t) occupancy_state;
					roomsense_iq_shared.rgb_led_state = VACANT_LED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);
					roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status = VACANT;
					xEventGroupSetBits(occupancy_group, OCCUPANCY_BIT);
				}
			}
			break;
		}
		}
		vTaskDelay(OCCUPANCY_STATE_MACHINE / portTICK_RATE_MS);
	}
}

void task_photocell_publish(void *param)
{
	int retry_counter = 0;
	char sensor_data[10];

	while (true)
	{
		xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
		vTaskDelay(PHOTOCELL_PUBLISH_RATE_MSEC / portTICK_RATE_MS);
		if (xMutex_g_light_density_raw != NULL)
		{
			if (xSemaphoreTake(xMutex_g_light_density_raw, (TickType_t) 10) == pdTRUE)
			{
				if (!g_ws_pause)
				{
					sprintf(sensor_data, "%d", roomsense_iq_shared.adafruit_161_shared.light_density_raw);
					esp_mqtt_client_publish(client, esp_discovery_light_sensor.state_topic, sensor_data, 0, PHOTOCELL_QOS, true);
				}
				xSemaphoreGive(xMutex_g_light_density_raw);
				retry_counter = 0;
			}
			else
			{
				retry_counter++;
				if (retry_counter >= 5)
				{
					ESP_LOGI(TAG, "Unable to take xMutex_g_light_density_raw.");
					retry_counter = 0;
				}
			}
		}
		else
		{
			ESP_LOGE(TAG, "Error: xMutex_g_light_density_raw not initialized");
		}
	}
}
void task_climate_sense_publish(void *param)
{
	char sensor_data[100];
	float temperature = 21;
	float temperature_f = 70;
	float dew_point = 21;
	float humidity = 30;
	uint16_t co2 = 400;
	uint16_t pm1 = 1;
	uint16_t pm25 = 2;
	uint16_t pm10 = 1;
	uint16_t co = 1;
	uint32_t voc_index = 1;

	int counter = 0;

	vTaskDelay(30000 / portTICK_RATE_MS); //Let the climatesense task finishes scanning all sensors
	ESP_LOGI(TAG, "task_climate_sense_publish");

	while (true)
	{
		xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
		vTaskDelay(50 / portTICK_RATE_MS);

		temperature = roomsense_iq_shared.scd40_shared.temperature;
		temperature_f = (temperature * 1.8) + 32;
		humidity = roomsense_iq_shared.scd40_shared.humidity;
		dew_point = roomsense_iq_shared.scd40_shared.dew_point;
		co2 = roomsense_iq_shared.scd40_shared.co2;
		voc_index = roomsense_iq_shared.sgp40_shared.voc_index;
		pm1 = roomsense_iq_shared.mpm10_shared.pm1;
		pm25 = roomsense_iq_shared.mpm10_shared.pm25;
		pm10 = roomsense_iq_shared.mpm10_shared.pm10;
		co = roomsense_iq_shared.tgs5141_shared.co;

		if (!g_ws_pause)
		{
			// publish state for temperature
			sprintf(sensor_data, "%.1f", temperature);
			esp_mqtt_client_publish(client, esp_discovery_temperature.state_topic, sensor_data, 0, T_H_QOS, true);

			sprintf(sensor_data, "%.1f", temperature_f);
			esp_mqtt_client_publish(client, esp_discovery_temperature_f.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for humidity
			sprintf(sensor_data, "%.0f", humidity);
			esp_mqtt_client_publish(client, esp_discovery_humidity.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for dew_point
			sprintf(sensor_data, "%.0f", dew_point);
			esp_mqtt_client_publish(client, esp_discovery_dew_point.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for co2
			sprintf(sensor_data, "%d", co2);
			esp_mqtt_client_publish(client, esp_discovery_co2.state_topic, sensor_data, 0, 2, true);

			// publish state for voc
			sprintf(sensor_data, "%d", voc_index);
			esp_mqtt_client_publish(client, esp_discovery_voc.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for pm1
			sprintf(sensor_data, "%d", pm1);
			esp_mqtt_client_publish(client, esp_discovery_pm1.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for pm2.5
			sprintf(sensor_data, "%d", pm25);
			esp_mqtt_client_publish(client, esp_discovery_pm25.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for pm10
			sprintf(sensor_data, "%d", pm10);
			esp_mqtt_client_publish(client, esp_discovery_pm10.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for CO
			sprintf(sensor_data, "%d", co);
			esp_mqtt_client_publish(client, esp_discovery_co.state_topic, sensor_data, 0, T_H_QOS, true);

			// publish state for CO
			sprintf(sensor_data, "%d", co);
			esp_mqtt_client_publish(client, esp_discovery_co.state_topic, sensor_data, 0, T_H_QOS, true);

//			sprintf(sensor_data, "%s", roomsense_iq_shared.humidity_status);
//			esp_mqtt_client_publish(client, esp_discovery_humidity_status.state_topic, sensor_data, 0, DIRECTION_QOS, true);
//
//			sprintf(sensor_data, "%s", roomsense_iq_shared.co2_status);
//			esp_mqtt_client_publish(client, esp_discovery_co2_status.state_topic, sensor_data, 0, DIRECTION_QOS, true);
//
//			sprintf(sensor_data, "%s", roomsense_iq_shared.voc_status);
//			esp_mqtt_client_publish(client, esp_discovery_voc_status.state_topic, sensor_data, 0, DIRECTION_QOS, true);
//
//			sprintf(sensor_data, "%s", roomsense_iq_shared.mold_risk_status);
//			esp_mqtt_client_publish(client, esp_discovery_mold_risk.state_topic, sensor_data, 0, DIRECTION_QOS, true);
		}

		vTaskDelay(CLIMATE_SENSE_REFRESH_RATE_MS / portTICK_RATE_MS);
	}

}
void task_settings_publish(void *param)
{
	char sensor_data[10];

	bool g_init_ha_settings = 1;

	while (true)
	{
		xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

		vTaskDelay(1000 / portTICK_RATE_MS);

		if (start_stop_status_changed || g_init_ha_settings)
		{
			sprintf(sensor_data, "%d", start_stop_status);
			esp_mqtt_client_publish(client, esp_discovery_start_stop_status.state_topic, sensor_data, 0, SETTINGS_QOS, true);
			start_stop_status_changed = 0;
			ESP_LOGI(TAG, "start_stop_status_changed");
		}

		if (roomsense_iq_shared.ha_mqtt_shared.g_pir_sensitivity_changed || g_init_ha_settings)
		{
			sprintf(sensor_data, "%d", roomsense_iq_shared.pir_sensor_shared.pir_sensitivity);
			esp_mqtt_client_publish(client, esp_discovery_number_pir_sensitivity.state_topic, sensor_data, 0, SETTINGS_QOS, true);
			roomsense_iq_shared.ha_mqtt_shared.g_pir_sensitivity_changed = 0;
		}

		if (roomsense_iq_shared.ha_mqtt_shared.g_max_macro_range_changed || g_init_ha_settings)
		{
			sprintf(sensor_data, "%d", roomsense_iq_shared.ld2410_config_shared.max_macro_range);
			esp_mqtt_client_publish(client, esp_discovery_number_max_macro_range.state_topic, sensor_data, 0, SETTINGS_QOS, true);
			roomsense_iq_shared.ha_mqtt_shared.g_max_macro_range_changed = 0;
		}

		if (roomsense_iq_shared.ha_mqtt_shared.g_max_micro_range_changed || g_init_ha_settings)
		{
			sprintf(sensor_data, "%d", roomsense_iq_shared.ld2410_config_shared.max_micro_range);
			esp_mqtt_client_publish(client, esp_discovery_number_max_micro_range.state_topic, sensor_data, 0, SETTINGS_QOS, true);
			roomsense_iq_shared.ha_mqtt_shared.g_max_micro_range_changed = 0;
		}

		if (roomsense_iq_shared.ha_mqtt_shared.g_timeout_changed || g_init_ha_settings)
		{
			sprintf(sensor_data, "%d", roomsense_iq_shared.ld2410_config_shared.absence_time_out);
			esp_mqtt_client_publish(client, esp_discovery_number_timeout.state_topic, sensor_data, 0, SETTINGS_QOS, true);
			roomsense_iq_shared.ha_mqtt_shared.g_timeout_changed = 0;
		}

		if (roomsense_iq_shared.ha_mqtt_shared.g_bedsense_changed || g_init_ha_settings)
		{
			esp_mqtt_client_publish(client, esp_discovery_switch_bedsense.state_topic, bedsense_switch_status, 0, SETTINGS_QOS, true);
			roomsense_iq_shared.ha_mqtt_shared.g_bedsense_changed = 0;
		}

		if (g_calsense_changed || g_init_ha_settings)
		{
			esp_mqtt_client_publish(client, esp_discovery_switch_calsense.state_topic, calsense_switch_status, 0, SETTINGS_QOS, true);
			g_calsense_changed = 0;
			//ESP_LOGI(TAG, "g_calsense_changed");
			g_init_ha_settings = 0;
		}
	}
}

/**
 * Main task for the MQTT application
 * @param pvParameters parameter which can be passed to the task
 */
static void mqtt_app_task(void *pvParameters)
{
	char mqtt_uri[MAX_MQTT_HOST_LENGTH];
	mqtt_app_queue_message_t msg;
	EventBits_t eventBits;

	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_MQTT_URI, /*!< Complete MQTT broker URI */
		.event_handle = mqtt_event_handler, /*!< handle for MQTT events as a callback in legacy mode */
		.lwt_topic = esp_discovery_number_max_macro_range.status_topic, /*!< LWT (Last Will and Testament) message topic  to specify the actions to be taken after a client goes offline unexpectedly */
		.lwt_msg = STATUS_OFFLINE, /*!< LWT message (NULL by default) */
		.lwt_qos = 1, /*!< LWT message qos */
		.lwt_retain = true, /*!< LWT retained message flag */
		.keepalive = 10 /*!< mqtt keepalive, default is 120 seconds */
	};


	// Send first event message
	mqtt_app_send_message(MQTT_APP_MSG_LOAD_SAVED_CREDENTIALS);

	for (;;)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);

		if (xQueueReceive(mqtt_app_queue_handle, &msg, portMAX_DELAY))
		{
			switch (msg.msgID)
			{
			case MQTT_APP_MSG_LOAD_SAVED_CREDENTIALS:
				ESP_LOGI(TAG, "MQTT_APP_MSG_LOAD_SAVED_CREDENTIALS");

				if (client) {
					ESP_LOGI(TAG, "Existing client found, stopping and destroying before proceeding");
					ESP_ERROR_CHECK(esp_mqtt_client_stop(client));
					ESP_ERROR_CHECK(esp_mqtt_client_destroy(client));
					client = NULL;
				}

				if (app_nvs_load_mqtt_creds())
				{
					ESP_LOGI(TAG, "Loaded mqtt credentials");
					strcpy(mqtt_uri, "");
					strcat(mqtt_uri, "mqtt://");
					strcat(mqtt_uri, mqtt_credentials->mqtt_host);
					strcat(mqtt_uri, ":");
					strcat(mqtt_uri, mqtt_credentials->mqtt_port);
					ESP_LOGI(TAG, "Loaded mqtt credentials URI =%s", mqtt_uri);
					mqtt_cfg.uri = mqtt_uri;
					mqtt_cfg.username = mqtt_credentials->mqtt_username;
					mqtt_cfg.password = mqtt_credentials->mqtt_password;
					xEventGroupSetBits(mqtt_app_event_group, MQTT_APP_CONNECTING_USING_SAVED_CREDS_BIT);
				}
				else
				{
					ESP_LOGI(TAG, "Unable to load mqtt credentials, using defaults");
				}

				ESP_LOGI(TAG, "[MQTT] Connecting to %s...", mqtt_cfg.uri);
				client = esp_mqtt_client_init(&mqtt_cfg); /*Creates mqtt client handle based on the configuration.*/
				ESP_ERROR_CHECK(esp_mqtt_client_start(client)); /*Starts mqtt client with already created client handle.*/

				mqtt_app_send_message(MQTT_APP_MSG_START_HTTP_SERVER);

				break;

			case MQTT_APP_MSG_START_HTTP_SERVER:
				ESP_LOGI(TAG, "MQTT_APP_MSG_START_HTTP_SERVER");

				http_server_start();

				break;

			case MQTT_APP_MSG_CONNECTING_FROM_HTTP_SERVER:

				ESP_LOGI(TAG, "MQTT_APP_MSG_CONNECTING_FROM_HTTP_SERVER");
				ESP_LOGI(TAG, "mqtt-host: %s mqtt-port: %s mqtt-user: %s mqtt-pass:%s", mqtt_credentials->mqtt_host, mqtt_credentials->mqtt_port,
						mqtt_credentials->mqtt_username, mqtt_credentials->mqtt_password);
				xEventGroupSetBits(mqtt_app_event_group, MQTT_APP_CONNECTING_FROM_HTTP_SERVER_BIT);
				roomsense_iq_shared.rgb_led_state = MQTT_DISCONNECTED;
				xEventGroupSetBits(led_event_group, RGB_LED_BIT);

				//ESP_LOGI(TAG, "xPortGetFreeHeapSize() %d", xPortGetFreeHeapSize());
				// Let the HTTP server know about the connection attempt
				http_server_monitor_send_message(HTTP_MSG_MQTT_CONNECT_INIT);

				if (client) {
					ESP_LOGI(TAG, "Existing client found, stopping and destroying before proceeding");
					ESP_ERROR_CHECK(esp_mqtt_client_stop(client));
					ESP_ERROR_CHECK(esp_mqtt_client_destroy(client));
					client = NULL;
				}

				strcpy(mqtt_uri, "");
				strcat(mqtt_uri, "mqtt://");
				strcat(mqtt_uri, mqtt_credentials->mqtt_host);
				strcat(mqtt_uri, ":");
				strcat(mqtt_uri, mqtt_credentials->mqtt_port);
				ESP_LOGI(TAG, "URI =%s", mqtt_uri);

				mqtt_cfg.uri = mqtt_uri;
				mqtt_cfg.username = mqtt_credentials->mqtt_username;
				mqtt_cfg.password = mqtt_credentials->mqtt_password;

				client = esp_mqtt_client_init(&mqtt_cfg);
				ESP_ERROR_CHECK(esp_mqtt_set_config(client, &mqtt_cfg));
				ESP_ERROR_CHECK(esp_mqtt_client_start(client));

				break;
			case MQTT_APP_MSG_NEW_CREDENTIALS:

				break;

			case MQTT_APP_MSG_CONNECTED:

				ESP_LOGI(TAG, "MQTT_APP_MSG_CONNECTED_TO_BROKER");
				g_mqtt_connection_status = true;

				if (roomsense_iq_shared.ha_mqtt_shared.g_occupancy_status == OCCUPIED)
				{
					roomsense_iq_shared.rgb_led_state = OCCUPIED_LED;
				}
				else
				{
					roomsense_iq_shared.rgb_led_state = VACANT_LED;
				}

				xEventGroupSetBits(led_event_group, RGB_LED_BIT);
				xEventGroupSetBits(mqtt_app_event_group, MQTT_APP_CONNECTED_BIT);

				http_server_monitor_send_message(HTTP_MSG_MQTT_CONNECT_SUCCESS);

				eventBits = xEventGroupGetBits(mqtt_app_event_group);
				if (eventBits & MQTT_APP_CONNECTING_USING_SAVED_CREDS_BIT) ///> Save creds only if connecting from the http server (not loaded from NVS)
				{
					ESP_LOGI(TAG, "Connected using saved Credentials.");
				}
				else
				{
					ESP_LOGI(TAG, "saved Credentials.");
					app_nvs_save_mqtt_creds();
					xEventGroupSetBits(mqtt_app_event_group, MQTT_APP_CONNECTING_USING_SAVED_CREDS_BIT);
				}
				if (eventBits & MQTT_APP_CONNECTING_FROM_HTTP_SERVER_BIT)
				{
					xEventGroupClearBits(mqtt_app_event_group, MQTT_APP_CONNECTING_FROM_HTTP_SERVER_BIT);
				}

				break;

			case MQTT_APP_MSG_DISCONNECTED:
				ESP_LOGI(TAG, "MQTT_APP_MSG_DISCONNECTED");
				g_mqtt_connection_status = false;
				xEventGroupClearBits(mqtt_app_event_group, MQTT_APP_CONNECTING_FROM_HTTP_SERVER_BIT);

				if (roomsense_iq_shared.wifi_app_shared.g_wifi_connection_status == true)
				{
					roomsense_iq_shared.rgb_led_state = MQTT_DISCONNECTED;
					xEventGroupSetBits(led_event_group, RGB_LED_BIT);
				}

				http_server_monitor_send_message(HTTP_MSG_MQTT_DISCONNECT);

				break;

			case MQTT_APP_MSG_USER_REQUESTED_DISCONNECT:
				ESP_LOGI(TAG, "MQTT_APP_MSG_USER_REQUESTED_DISCONNECT");
				g_mqtt_connection_status = false;
				roomsense_iq_shared.rgb_led_state = MQTT_DISCONNECTED;
				xEventGroupSetBits(led_event_group, RGB_LED_BIT);

				eventBits = xEventGroupGetBits(mqtt_app_event_group);

				if (eventBits & MQTT_APP_CONNECTED_BIT)
				{
					xEventGroupSetBits(mqtt_app_event_group, MQTT_APP_USER_REQUESTED_DISCONNECT_BIT);

					//ESP_ERROR_CHECK(esp_mqtt_client_disconnect(client));
					app_nvs_clear_mqtt_creds();

					mqtt_cfg.username = NULL;
					mqtt_cfg.password = NULL;

					xEventGroupClearBits(mqtt_app_event_group, MQTT_APP_CONNECTING_USING_SAVED_CREDS_BIT);

					//mqtt_app_send_message(MQTT_APP_MSG_LOAD_SAVED_CREDENTIALS);

					// Send a message to disconnect Wifi and clear credentials
					//wifi_app_send_message(WIFI_APP_MSG_USER_REQUESTED_STA_DISCONNECT);

				}

				break;

			case MQTT_APP_MSG_RESTART_CPU:

				app_nvs_save_occupancy_status();
				esp_restart();

				break;

			default:
				break;

				//TODO: ADD case MQTT_APP_MSG_USER_REQUESTED_DISCONNECT:
			}
		}
	}
}

BaseType_t mqtt_app_send_message(mqtt_app_message_e msgID)
{
	mqtt_app_queue_message_t msg;
	msg.msgID = msgID;
	return xQueueSend(mqtt_app_queue_handle, &msg, portMAX_DELAY);
}

void ha_mqtt_start()
{
	ESP_LOGI(TAG, "[APP] Startup..");
//ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
//ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

	esp_log_level_set("*", ESP_LOG_INFO);
	esp_log_level_set("esp-tls", ESP_LOG_INFO);
	esp_log_level_set("MQTT_CLIENT", ESP_LOG_INFO);
	esp_log_level_set(TAG, ESP_LOG_INFO);

	app_nvs_load_occupancy_status();

	roomsense_iq_shared.ha_mqtt_shared.g_macro_threshold_changed = 1;
	roomsense_iq_shared.ha_mqtt_shared.g_micro_threshold_changed = 1;
	roomsense_iq_shared.ha_mqtt_shared.g_pir_sensitivity_changed = 1;
	roomsense_iq_shared.ha_mqtt_shared.g_max_macro_range_changed = 1;
	roomsense_iq_shared.ha_mqtt_shared.g_max_micro_range_changed = 1;
	roomsense_iq_shared.ha_mqtt_shared.g_timeout_changed = 1;
	roomsense_iq_shared.ha_mqtt_shared.g_bedsense_changed = 1;
	g_calsense_changed = 1;

	strcpy(bedsense_switch_status, "OFF");
	strcpy(calsense_switch_status, "OFF");

// Create message queue
	mqtt_app_queue_handle = xQueueCreate(3, sizeof(mqtt_app_queue_message_t));

// Allocate memory for the mqtt credentials
	mqtt_credentials = (mqtt_credentials_t*) malloc(sizeof(mqtt_credentials_t));
	memset(mqtt_credentials, 0x00, sizeof(mqtt_credentials_t));

// Create Wifi application event group
	mqtt_app_event_group = xEventGroupCreate();

	mqtt_event_group = xEventGroupCreate();
	env_event_group = xEventGroupCreate();
	esp_event_group = xEventGroupCreate();
	discovery_event_group = xEventGroupCreate();
	persist_event_group = xEventGroupCreate();
	occupancy_group = xEventGroupCreate();
	movement_direction_group = xEventGroupCreate();
	pir_group = xEventGroupCreate();

//reads the MAC address to
	ESP_ERROR_CHECK(get_mac_address(&esp_serial));

//load manufacturer, model and firmware version
	ESP_ERROR_CHECK(esp_device_init(&esp_device));

//init buttons
//	ESP_ERROR_CHECK(esp_discovery_init_button(&esp_discovery_button_start));
//	ESP_ERROR_CHECK(esp_discovery_init_button(&esp_discovery_button_read));
//	ESP_ERROR_CHECK(esp_discovery_init_button(&esp_discovery_button_config));
//	ESP_ERROR_CHECK(esp_discovery_init_button(&esp_discovery_button_reset));

//	ESP_ERROR_CHECK(esp_discovery_init_switch(&esp_discovery_switch_bedsense));
//	ESP_ERROR_CHECK(esp_discovery_init_switch(&esp_discovery_switch_calsense));

//init numbers
//	ESP_ERROR_CHECK(esp_discovery_init_number(&esp_discovery_number_pir_sensitivity));
//	ESP_ERROR_CHECK(esp_discovery_init_number(&esp_discovery_number_max_macro_range));
//	ESP_ERROR_CHECK(esp_discovery_init_number(&esp_discovery_number_max_micro_range));
//	ESP_ERROR_CHECK(esp_discovery_init_number(&esp_discovery_number_timeout));

//Sensors
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_temperature));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_temperature_f));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_humidity));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_dew_point));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_co2));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_voc));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_pm1));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_pm25));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_pm10));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_co));


	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_movement_direction));
	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_location));
//	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_humidity_status));
//	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_co2_status));
//	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_voc_status));
	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_co_status));
	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_pm_status));
//	ESP_ERROR_CHECK(esp_discovery_init_text(&esp_discovery_mold_risk));

	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_light_sensor));
	ESP_ERROR_CHECK(esp_discovery_init_pir_sensor(&esp_discovery_pir_sensor));

	vTaskDelay(100 / portTICK_PERIOD_MS);
//Room Presence
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_room_presence));
	vTaskDelay(100 / portTICK_PERIOD_MS);
	//ESP_ERROR_CHECK(esp_discovery_init_room_presence(&esp_discovery_distance_detected));
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_distance_detected));
	vTaskDelay(100 / portTICK_PERIOD_MS);
	ESP_ERROR_CHECK(esp_discovery_init_sensor(&esp_discovery_distance_detected_ft));

//	ESP_ERROR_CHECK(esp_discovery_init_room_presence(&esp_discovery_start_stop_status));

	ledc_timer_config(&ledc_timer);
	ledc_channel_config(&ledc_red_channel);
	ledc_channel_config(&ledc_green_channel);
	ledc_channel_config(&ledc_blue_channel);

#ifdef LED_HAS_WHITE

    ledc_channel_config(&ledc_n_white_channel);
    ledc_channel_config(&ledc_w_white_channel);

#endif

	ledc_fade_func_install(0);

	xTaskCreatePinnedToCore(&mqtt_app_task, "mqtt_app_task", MQTT_APP_TASK_STACK_SIZE, NULL, MQTT_APP_TASK_PRIORITY, NULL, MQTT_APP_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_discovery_publish, "task_discovery_publish", MQTT_APP_DISCOVERY_TASK_STACK_SIZE, NULL, MQTT_APP_DISCOVERY_TASK_PRIORITY, NULL,
			MQTT_APP_DISCOVERY_TASK_CORE_ID);
//	xTaskCreatePinnedToCore(task_settings_publish, "task_settings_publish", 20 * 1024, NULL, MQTT_APP_TASK_PRIORITY, NULL, MQTT_APP_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_occupancy_publish, "task_occupancy_publish", OCCUPANCY_PUBLISH_TASK_STACK_SIZE, NULL, OCCUPANCY_PUBLISH_TASK_PRIORITY, NULL,
			OCCUPANCY_PUBLISH_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_movement_direction_publish, "task_movement_direction_publish", 3 * 1024, NULL, MQTT_APP_TASK_PRIORITY, NULL,
			MQTT_APP_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_distance_publish, "task_distance_publish", 6 * 1024, NULL, MQTT_APP_TASK_PRIORITY, NULL, MQTT_APP_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_climate_sense_publish, "task_climate_sense_publish", 5 * 1024, NULL, MQTT_APP_TASK_PRIORITY, NULL, MQTT_APP_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_photocell_publish, "task_photocell_publish", 8 * 1024, NULL, MQTT_APP_TASK_PRIORITY, NULL, MQTT_APP_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_pir_publish, "task_pir_publish", MQTT_PIR_APP_TASK_STACK_SIZE, NULL, MQTT_PIR_APP_TASK_PRIORITY, NULL,
			MQTT_PIR_APP_TASK_CORE_ID);

	xTaskCreatePinnedToCore(task_climatesense_alarm_publish, "task_climatesense_alarm_publish", 3 * 1024, NULL, MQTT_PIR_APP_TASK_PRIORITY,
			NULL, MQTT_PIR_APP_TASK_CORE_ID);
	xTaskCreatePinnedToCore(task_occupancy_state, "task_occupancy_state", OCCUPANCY_STATE_TASK_STACK_SIZE, NULL, OCCUPANCY_STATE_TASK_PRIORITY,
			&task_occupancy_state_handle, OCCUPANCY_STATE_TASK_CORE_ID);
}

bool toggle_value(bool trigger)
{

	if (trigger)
		trigger = 0;
	else
		trigger = 1;

	return trigger;
}
