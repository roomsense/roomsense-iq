/**
 * Application entry point.
 */

#include "esp_log.h"
#include "tasks_common.h"
#include "nvs_flash.h"
#include "wifi_app.h"
#include "wifi_reset_button.h"
#include "co_sensor.h"
#include "pir-sensor.h"
#include "ld2410.h"
#include "adafruit-161.h"
#include "scd4x.h"
#include "sht30.h"
#include "sht21.h"
#include "sgp40.h"
#include "mpm10.h"
#include "climatesense.h"
#include "ha_mqtt.h"
#include "rgb_led.h"
#include "direction-detection.h"
#include "watchdog.h"
#include "app_nvs.h"
#include <string.h>

#define MAC_SIZE 6

static const char *TAG = "MAIN";

roomsense_iq roomsense_iq_shared =
{
	 .ap_key                     = "password",  // Initialize with "password"
	 .location                   = {0},
	 .led_enable                 = true,
	 .ap_enable                  = true,
	 .alert_flag                 = false,
	 .climate_alarm              = false,
	 .forced_calibration         = false,
	 .climatesense_connection    = false,
	 .t_h_co2_connection         = false,
	 .voc_connection             = false,
	 .pm_connection              = false,
	 .ld2410_config_shared       = {0},
	 .ld2410_data_shared         = {0},
	 .buttons_shared             = { .g_button_get_config = false, .g_button_set_config = false, .g_button_factory_reset = false, .g_button_config_to_pir = false },
	 .adafruit_161_shared        = { .light_density_raw = 0 },
	 .direction_detection_shared = { .macro_movement = 0, .g_movement_direction = 0 },
	 .pir_sensor_shared          = { .g_pir_status = 0, .pir_sensitivity = high },
	 .ha_mqtt_shared             = { .g_macro_threshold_changed = 0, .g_micro_threshold_changed = 0, .g_pir_sensitivity_changed = 0, .g_max_macro_range_changed = 0,
		                             .g_max_micro_range_changed = 0, .g_timeout_changed = 0, .g_bedsense_changed = 0, .g_bedsense_status = 0, .g_calsense_status = false, .nvs_occupancy = 0,
		                             .g_occupancy_status = 0 },
	 .sht30_shared               = { .g_temperature = 20, .g_humidity = 40 },
	 .scd40_shared               = { .temperature = 0, .humidity = 0, .dew_point = 0, .co2 = 0 },
	 .mpm10_shared               = { .pm1 = 0, .pm25 = 0, .pm10 = 0 },
	 .sgp40_shared               = { .voc_index = 0, },
	 .tgs5141_shared             = { .co = 0 },
	 .wifi_app_shared            = { .g_wifi_connection_status = false,  .g_ap_connection_status = false},
	 .rgb_led_shared             = { .reset_sw = false },
     .rgb_led_state              =  WIFI_DISCONNECTED,
	 .rgb_led_alert              =  HARDWARE_FAILING_LED
};

EventGroupHandle_t led_event_group = NULL;
EventGroupHandle_t event_group_alert;
const int RGB_LED_BIT = BIT5;

extern EventGroupHandle_t dashboard_event_group;
extern const int TEMPERATUR_READY_BIT_R;


esp_err_t load_location();
esp_err_t load_mac();

void app_main(void)
{

	uint32_t free_heap;

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
	}
	ESP_ERROR_CHECK(ret);

	printf("RoomSense © Production image version 20240903\n");

	baseline_semaphore =  xSemaphoreCreateBinary();
    if (baseline_semaphore == NULL) {
        printf("Failed to create baseline_semaphore\n");
        vTaskDelete(NULL);
    }

	blindspot_semaphore =  xSemaphoreCreateBinary();
    if (blindspot_semaphore == NULL) {
        printf("Failed to create blindspot_semaphore\n");
        vTaskDelete(NULL);
    }

	dashboard_event_group = xEventGroupCreate();
	led_event_group       = xEventGroupCreate();
	event_group_alert     = xEventGroupCreate();


	xEventGroupSetBits(dashboard_event_group, TEMPERATUR_READY_BIT_R);

	//printf("ESP-IDF Version %s", esp_get_idf_version());

#ifdef ENABLE_ACCESS_POINT_SWITCH
	app_nvs_load_ap_enable();
#endif

	app_nvs_load_led_enable();
	app_nvs_load_ap_key();
    load_location();
	load_mac();

	esp_reset_reason_t reason = esp_reset_reason();
	if (reason == ESP_RST_SW)
	{
		roomsense_iq_shared.rgb_led_shared.reset_sw = true;
	}

	watchdog_start();
	rgb_led_start();
	climatesense_task_start();
	wifi_app_start();
	pir_task_start();
	direction_detection_task_start();
	ld2410_task_start();
	ada161_task_start();
	ha_mqtt_start();

	vTaskDelay(25000 / portTICK_PERIOD_MS);

	roomsense_iq_shared.rgb_led_shared.reset_sw = false;

	while (1)
	{
		vTaskDelay(2000 / portTICK_PERIOD_MS);
		free_heap = esp_get_free_heap_size();
		printf("Current free heap size: %d bytes\n", free_heap);


		if(free_heap < 60000)
		{
			app_nvs_save_occupancy_status();
			esp_restart();
		}

	}
}



esp_err_t load_location()
{
	esp_err_t err;
	nvs_handle handle;
	char loc[LOCATION_SIZE + 1] = {0};
	size_t loc_sz = LOCATION_SIZE;

	err = nvs_open("net_config", NVS_READWRITE, &handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error in opening NVS for loading location: %s", esp_err_to_name(err));
		return err;
	}


	err = nvs_get_blob(handle, "dev_loc_tag", loc, &loc_sz);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error in reading location from NVS: %s", esp_err_to_name(err));
		return err;
	}
	nvs_close(handle);
    strcpy(roomsense_iq_shared.location, loc);

    return err;
}
esp_err_t load_mac()
{
	uint8_t mac[MAC_SIZE];

	esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);

	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "MAC address reading failed: %s", esp_err_to_name(err));

	}
	else
	{
		snprintf(roomsense_iq_shared.mac_address, sizeof(roomsense_iq_shared.mac_address), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4],mac[5]);
		ESP_LOGI(TAG, "MAC address: %s", roomsense_iq_shared.mac_address);
	}

	return err;
}
