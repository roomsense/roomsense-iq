/*
 * tasks_common.h
 *
 *  Created on: Oct 17, 2021
 *      Author: builder
 */

#ifndef MAIN_TASKS_COMMON_H_
#define MAIN_TASKS_COMMON_H_

#include "esp_task_wdt.h"
#include <freertos/semphr.h>

//#define ENABLE_ACCESS_POINT_SWITCH

#define CLIMATE_SENSE_REFRESH_RATE_MS       5*60*1000

#define WDT_TIMEOUT_SEC   40
#define BUFFER_SIZE 96

#define PRESENCE_ENABLE_PIN      18
#define PRESENCE_ENABLE_PIN_SEL  (1ULL << PRESENCE_ENABLE_PIN)

// WiFi application task
#define WIFI_APP_TASK_STACK_SIZE			4096
#define WIFI_APP_TASK_PRIORITY				6
#define WIFI_APP_TASK_CORE_ID				0

// HTTP Server task
#define HTTP_SERVER_TASK_STACK_SIZE			8192
#define HTTP_SERVER_TASK_PRIORITY			5
#define HTTP_SERVER_TASK_CORE_ID			1

// HTTP Server Monitor task
#define HTTP_SERVER_MONITOR_STACK_SIZE		4*1024
#define HTTP_SERVER_MONITOR_PRIORITY		4
#define HTTP_SERVER_MONITOR_CORE_ID			0


#define MQTT_APP_TASK_STACK_SIZE			10*1024
#define MQTT_APP_TASK_PRIORITY				3
#define MQTT_APP_TASK_CORE_ID				1

#define MQTT_APP_DISCOVERY_TASK_STACK_SIZE	5*1024
#define MQTT_APP_DISCOVERY_TASK_PRIORITY	3
#define MQTT_APP_DISCOVERY_TASK_CORE_ID		1

#define MQTT_PIR_APP_TASK_STACK_SIZE		2*1024
#define MQTT_PIR_APP_TASK_PRIORITY			5
#define MQTT_PIR_APP_TASK_CORE_ID			0

#define OCCUPANCY_PUBLISH_TASK_STACK_SIZE	3*1024
#define OCCUPANCY_PUBLISH_TASK_PRIORITY		6
#define OCCUPANCY_PUBLISH_TASK_CORE_ID		1

#define OCCUPANCY_STATE_TASK_STACK_SIZE     2*1024
#define OCCUPANCY_STATE_TASK_PRIORITY		6
#define OCCUPANCY_STATE_TASK_CORE_ID		0

// PIR task
#define PIR_TASK_STACK_SIZE			        4*1024
#define PIR_TASK_PRIORITY			        6
#define PIR_TASK_CORE_ID				    0

//wifi Reset Button task
#define WIFI_RESET_BUTTON_TASK_STACK_SIZE   1048
#define WIFI_RESET_BUTTON_TASK_PRIORITY     4
#define WIFI_RESET_BUTTON_TASK_CORE_ID      0

// LD2410 task
#define LD2410_TASK_STACK_SIZE			    4192
#define LD2410_TASK_PRIORITY			    6
#define LD2410_TASK_CORE_ID				    0

// ADA161 task
#define ADA161_TASK_STACK_SIZE			    4096
#define ADA161_TASK_PRIORITY			    5
#define ADA161_TASK_CORE_ID				    0

#define CO_SENSOR_TASK_STACK_SIZE		    2048
#define CO_SENSOR_TASK_PRIORITY			    5
#define CO_SENSOR_TASK_CORE_ID			    0

// SHT3X Sensor task
#define SHT3X_TASK_STACK_SIZE				4096
#define SHT3X_TASK_PRIORITY					5
#define SHT3X_TASK_CORE_ID					0

#define SCD4X_TASK_STACK_SIZE				4096
#define SCD4X_TASK_PRIORITY					5
#define SCD4X_TASK_CORE_ID					0

// SHT21 Sensor task
#define SHT21_TASK_STACK_SIZE				4096
#define SHT21_TASK_PRIORITY					5
#define SHT21_TASK_CORE_ID					0

// SGP40 Sensor task
#define SGP40_TASK_STACK_SIZE				4096
#define SGP40_TASK_PRIORITY					5
#define SGP40_TASK_CORE_ID					0

// MPM10 Sensor task
#define MPM10_TASK_STACK_SIZE				4096
#define MPM10_TASK_PRIORITY					5
#define MPM10_TASK_CORE_ID					0

// ClimateSense task
#define CLIMATESENSE_TASK_STACK_SIZE		4096
#define CLIMATESENSE_TASK_PRIORITY			5
#define CLIMATESENSE_TASK_CORE_ID			0


#define RGB_LED_TASK_STACK_SIZE			    4096
#define RGB_LED_TASK_PRIORITY				6
#define RGB_LED_TASK_CORE_ID				0

#define DIRECTION_DETECTION_TASK_STACK_SIZE			    4096
#define DIRECTION_DETECTION_TASK_PRIORITY				2
#define DIRECTION_DETECTION_TASK_CORE_ID				0

#define WATCHDOG_TASK_STACK_SIZE            2048
#define WATCHDOG_TASK_PRIORITY              2
#define WATCHDOG_TASK_CORE_ID               0

#define MAX_MQTT_PUBLISH_LENGTH             20

#define MAC_ADDRESS_LENGTH                  15
#define LOCATION_SIZE                       32

extern SemaphoreHandle_t baseline_semaphore;
extern SemaphoreHandle_t blindspot_semaphore;

TaskHandle_t task_occupancy_state_handle;


#define HUMIDITY_ALERT (1 << 0)
#define VOC_ALERT      (1 << 1)
#define CO2_ALERT      (1 << 2)
#define PM_ALERT       (1 << 3)
#define CO_ALERT       (1 << 4)
#define MOLD_ALERT     (1 << 5)


typedef enum rgb_led_state
{
	WIFI_DISCONNECTED = 0,
	WIFI_CONNECTED,
	HTPP_DISCONNECTED,
	MQTT_DISCONNECTED,
	MQTT_CONNECTED,
	VACANT_LED,
	OCCUPIED_LED,
} rgb_led_state_e;

typedef enum rgb_led_alert
{
	HARDWARE_FAILING_LED = 0,
	Elevated_AIRBORNE_POLLUTION,
	EXTRME_AIRBORNE_POLLUTION,
} rgb_led_alert_e;

typedef enum pir_sensitivity
{
       low = 1,
       medium,
       high,
} pir_sensitivity_e;

typedef struct {
	unsigned long time_us;
	uint16_t macro_distance;
	uint16_t micro_distance;
	uint16_t detected_distance;
	uint8_t  macro_level;
	uint8_t  micro_level;
	uint8_t  macro_bin[9];
	uint8_t  micro_bin[9];
	uint8_t  target_status;
} ld2410_data_t;


typedef struct {
	bool     start_stop;
	char     valid;
    uint16_t  max_macro_range;
    uint16_t  max_micro_range;
    int8_t  macro_threshold[9];
    int8_t  micro_threshold[9];
    uint16_t absence_time_out;
} ld2410_config_t;

typedef struct {
	uint32_t light_density_raw;
} adafruit_161_t;

typedef struct {
	bool    macro_movement;
	uint8_t g_movement_direction;
} direction_detection_t;

typedef struct {
	bool g_pir_status;
	pir_sensitivity_e      pir_sensitivity;
} pir_sensor_t;

typedef struct {
	float g_temperature;
	float g_humidity;
} sht30_t;

typedef struct {
	float    temperature;
	float    humidity;
	float    dew_point;
	uint16_t co2;
} g_scd40_t;

typedef struct {
	uint16_t pm1;
	uint16_t pm25;
	uint16_t pm10;
} g_mpm10_t;

typedef struct {
	uint32_t voc_index;
} g_sgp40_t;

typedef struct {
	uint32_t co;
} g_tgs5141_t;

typedef struct {
	bool g_wifi_connection_status;
	bool g_ap_connection_status;
} wifi_app_t;

typedef struct {
	bool reset_sw;
} rgb_led_t;

typedef struct {
	bool g_macro_threshold_changed;
	bool g_micro_threshold_changed;
	bool g_pir_sensitivity_changed;
	bool g_max_macro_range_changed;
	bool g_max_micro_range_changed;
	bool g_timeout_changed;
	bool g_bedsense_changed;
	bool g_bedsense_status;
	bool g_calsense_status;
	uint8_t nvs_occupancy;
	uint8_t g_occupancy_status;
} ha_mqtt_t;

typedef struct {
bool g_button_get_config;
bool g_button_set_config;
bool g_button_factory_reset;
bool g_button_config_to_pir;
} buttons_t;

typedef struct {
	bool                  led_enable;
	bool                  ap_enable;
	bool                  climatesense_connection;
	bool                  t_h_co2_connection;
	bool                  voc_connection;
	bool                  pm_connection;
	bool                  alert_flag;
	bool                  climate_alarm;
	bool                  forced_calibration;
	char                  mac_address[MAC_ADDRESS_LENGTH];
	char                  location[100];
	char                  mold_risk_status[100];
	char                  humidity_status[100];
	char                  co2_status[100];
	char                  voc_status[100];
	char                  pm_status[100];
	char                  co_status[100];
	uint8_t               ap_key[32];
	ld2410_config_t       ld2410_config_shared;
	ld2410_data_t         ld2410_data_shared;
	buttons_t             buttons_shared;
	adafruit_161_t        adafruit_161_shared;
	direction_detection_t direction_detection_shared;
	pir_sensor_t          pir_sensor_shared;
	ha_mqtt_t             ha_mqtt_shared;
	sht30_t               sht30_shared;
	g_scd40_t             scd40_shared;
	g_mpm10_t             mpm10_shared;
	g_sgp40_t             sgp40_shared;
	g_tgs5141_t           tgs5141_shared;
	wifi_app_t            wifi_app_shared;
	rgb_led_t             rgb_led_shared;
	rgb_led_state_e       rgb_led_state;
	rgb_led_alert_e       rgb_led_alert;
} roomsense_iq;

extern roomsense_iq roomsense_iq_shared;
extern bool g_ws_pause;

typedef struct {
	int	buffer[BUFFER_SIZE];	// Empty circular buffer
	int	readIndex;	// Index of the read pointer
	int	writeIndex;	// Index of the write pointer
	int	bufferLength;	// Number of values in circular buffer
	int valid_data_len;
} CircularBuffer;

extern CircularBuffer buffer_co2;
extern CircularBuffer buffer_voc;
extern CircularBuffer buffer_pm1;
extern CircularBuffer buffer_pm2_5;
extern CircularBuffer buffer_pm10;
extern CircularBuffer buffer_co;
extern CircularBuffer buffer_temperature;
extern CircularBuffer buffer_humidity;
void init_buffer(CircularBuffer *buffer);
bool write_to_buffer(CircularBuffer *buffer, int value);
bool read_from_buffer(CircularBuffer *buffer, int *value);

void send_config_setting(uint32_t max_macro_range, uint32_t max_micro_range, uint32_t timeout,
						 uint32_t pir_sensistivity, uint32_t bedsense_state, int8_t*set_macro_th, int8_t* set_micro_th);

#endif /* MAIN_TASKS_COMMON_H_ */
