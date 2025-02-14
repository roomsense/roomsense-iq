/*
 * climatesense.c
 *
 *  Created on: Jan. 11, 2024
 *      Author: builder
 */

#include <inttypes.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "freertos/event_groups.h"
#include <sht3x.h>
#include <sgp40.h>
#include <scd4x.h>
#include <mpm10.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <math.h>
#include "tasks_common.h"
#include "climatesense.h"
#include "co_sensor.h"
#include "esp_timer.h"
#include "dashboard.h"
/* float is used in printf(). you need non-default configuration in
 * sdkconfig for ESP8266, which is enabled by default for this
 * example. see sdkconfig.defaults.esp8266
 */

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define MAX_SGP40_RETRIES    5
#define ALERT_WARM_UP_PERIOD 10

static const char *TAG = "climatesense";

static uint16_t alert_counter = 0;

extern CircularBuffer buffer_temperature;
extern CircularBuffer buffer_humidity;
extern EventGroupHandle_t event_group_alert;
extern EventGroupHandle_t led_event_group;

extern const int RGB_LED_BIT;

// Define time_keeping_instace_t structure if it's not already defined
typedef struct time_keeping_instace
{
    uint32_t last_tick;
    uint32_t current_tick;
    uint8_t generic_flag1;
    uint8_t generic_flag2;
    uint8_t generic_flag3;
    uint8_t generic_flag4;
    uint8_t disable_auto_update_last_tick;
} time_keeping_instace_t;



void update_last_tick(time_keeping_instace_t* time_keeping) {
    time_keeping->last_tick = esp_timer_get_time() / 1000; // Convert microseconds to milliseconds
}

bool check_time_elapsed(time_keeping_instace_t* time_keeping, int interval_ms) {
    uint32_t current_tick = esp_timer_get_time() / 1000; // Convert microseconds to milliseconds
    if ((current_tick - time_keeping->last_tick) > interval_ms) {
        if (time_keeping->disable_auto_update_last_tick == false)
            update_last_tick(time_keeping);
        return true;
    }
    else {
        return false;
    }
}

void disable_auto_update_last_tick(time_keeping_instace_t* time_keeping) {
    time_keeping->disable_auto_update_last_tick = true;
}

time_keeping_instace_t climatesense_pkt_tx_timer;

float calculate_dew_point(float temperature, float humidity) {
    // Constants for Magnus formula
    float a = 17.27;
    float b = 237.7;

    // Calculate intermediate values
    float alpha = ((a * temperature) / (b + temperature)) + log(humidity / 100.0);

    // Calculate dew point
    float dewPoint = (b * alpha) / (a - alpha);

    return dewPoint;
}

static const char* dew_point_actionable_insight(float temperature, float dew_point)
{
	if (temperature  <= dew_point)
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
		    xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		    xEventGroupSetBits(event_group_alert, MOLD_ALERT);
		    roomsense_iq_shared.rgb_led_alert = Elevated_AIRBORNE_POLLUTION;
		}
		return "High";
	}
	else if (temperature  > dew_point)
	{
		xEventGroupClearBits(event_group_alert, MOLD_ALERT);
		return "Minimal";
	}

	xEventGroupClearBits(event_group_alert, MOLD_ALERT);
	return " ";
}

static const char* humidity_actionable_insight(float h)
{
	if (h  <= 25)
	{
		xEventGroupClearBits(event_group_alert, HUMIDITY_ALERT);
		return "Extremely Low";
	}
	else if (h > 25 && h <= 30)
	{
		xEventGroupClearBits(event_group_alert, HUMIDITY_ALERT);
		return "Low";
	}
	else if (h > 30 && h <= 60)
	{
		xEventGroupClearBits(event_group_alert, HUMIDITY_ALERT);
		return "Normal";
	}
	else if (h > 60 && h <= 70)
	{
		xEventGroupClearBits(event_group_alert, HUMIDITY_ALERT);
		return "High";
	}
	if (h  > 70)
	{
		xEventGroupClearBits(event_group_alert, HUMIDITY_ALERT);
		return "Extremely High";
	}

	xEventGroupClearBits(event_group_alert, HUMIDITY_ALERT);
	return " ";
}

static const char* voc_actionable_insight(int32_t voc_index, uint16_t co2)
{
	if (voc_index <= 0)
	{
		xEventGroupClearBits(event_group_alert, VOC_ALERT);
		return "Sensor Warming Up";
	}
	else if (voc_index <= 70 && co2 <=350)
	{
		xEventGroupClearBits(event_group_alert, VOC_ALERT);
		return "Very Clean";
	}
	else if (voc_index <= 90 && co2 <= 550)
	{
		xEventGroupClearBits(event_group_alert, VOC_ALERT);
		return "Clean";
	}
	else if (voc_index <= 130)
	{
		xEventGroupClearBits(event_group_alert, VOC_ALERT);
		return "Normal";
	}
	else if (voc_index <= 170)
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
		   xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		   xEventGroupSetBits(event_group_alert, VOC_ALERT);
		   roomsense_iq_shared.rgb_led_alert = Elevated_AIRBORNE_POLLUTION;
		}
		return "Moderately Polluted";
	}
	else if (voc_index <= 279)
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
		   xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		   xEventGroupSetBits(event_group_alert, VOC_ALERT);
		   roomsense_iq_shared.rgb_led_alert = Elevated_AIRBORNE_POLLUTION;
		   printf("voc_index <= 200 %d in yellow flashing\n", voc_index);
		}
		return "High";
	}
	else if (voc_index >= 280)
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
		   xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		   xEventGroupSetBits(event_group_alert, VOC_ALERT);
		   roomsense_iq_shared.rgb_led_alert = EXTRME_AIRBORNE_POLLUTION;
		}
		return "Extremely High";
	}

	xEventGroupClearBits(event_group_alert, VOC_ALERT);
	return " ";
}

static const char* pm_actionable_insight(uint16_t pm1, uint16_t pm2_5, uint16_t pm10)
{

	if ((pm1 >= 25 || pm2_5 >= 25 || pm10 >= 25) && (pm1 < 75 || pm2_5 < 75 || pm10 < 75))
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
			xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		    xEventGroupSetBits(event_group_alert, PM_ALERT);
		    roomsense_iq_shared.rgb_led_alert = Elevated_AIRBORNE_POLLUTION;
		}
		roomsense_iq_shared.climate_alarm = false;
		return "Elevated Levels";
	}
	else if (pm1 >= 75 || pm2_5 >= 75 || pm10 >= 75)
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
			xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		    xEventGroupSetBits(event_group_alert, PM_ALERT);
		    roomsense_iq_shared.rgb_led_alert = EXTRME_AIRBORNE_POLLUTION;
		    printf("pm1 >= 50 || pm2_5 >= 50 || pm10 >= 50\n");
		}
		roomsense_iq_shared.climate_alarm = true;
		return "Extremely High";
	}

	xEventGroupClearBits(event_group_alert, PM_ALERT);
	roomsense_iq_shared.climate_alarm = false;
	return "No Alarm";
}

static const char* co2_actionable_insight(uint16_t co2)
{
	if (co2 <=350)
	{
		xEventGroupClearBits(event_group_alert, CO2_ALERT);
		return "CO2 Levels Optimal";
	}
	else if (co2 <= 550)
	{
		xEventGroupClearBits(event_group_alert, CO2_ALERT);
		return "Like Fresh Air";
	}
	if (co2 > 550 && co2 <= 800)
	{
		xEventGroupClearBits(event_group_alert, CO2_ALERT);
		return "Elevated";
	}
	else if (co2 > 800 && co2 <= 1000)
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
		   xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		   xEventGroupSetBits(event_group_alert, CO2_ALERT);
		   roomsense_iq_shared.rgb_led_alert = Elevated_AIRBORNE_POLLUTION;
		}
		return "Drowsy Air.Ventilation";
	}
	else if (co2 > 1000)
	{
		if(alert_counter > ALERT_WARM_UP_PERIOD)
		{
	       xEventGroupSetBits(led_event_group, RGB_LED_BIT); // to unblock LED task
		   xEventGroupSetBits(event_group_alert, CO2_ALERT);
		   roomsense_iq_shared.rgb_led_alert = EXTRME_AIRBORNE_POLLUTION;
		}
		return "Extreme High";
	}

	xEventGroupClearBits(event_group_alert, CO2_ALERT);
	return " ";
}

void climatesense_task(void *pvParamters)
{
	sht3x_t sht;
	sgp40_t sgp;
	scd4x_t scd;
	mpm10_t mpm;
	uint16_t serial[3];
	uint16_t *frc_correction;

	float temperature = 0, humidity = 0;
	int32_t voc_index;
	uint16_t data = 0;
	float t_offset;
	bool malfunction = 0;
	int sgp40_retry_count = 0;
	esp_err_t result;
	uint16_t  calibration_counter = 0;
	update_last_tick(&climatesense_pkt_tx_timer);
	init_buffer(&buffer_temperature);
	init_buffer(&buffer_humidity);
	init_buffer(&buffer_co2);
	init_buffer(&buffer_co);
	init_buffer(&buffer_pm10);
	init_buffer(&buffer_pm1);
	init_buffer(&buffer_pm2_5);
	init_buffer(&buffer_voc);

	// initialize values
    strcpy(roomsense_iq_shared.humidity_status, humidity_actionable_insight(roomsense_iq_shared.scd40_shared.humidity));
    strcpy(roomsense_iq_shared.voc_status, voc_actionable_insight(voc_index, roomsense_iq_shared.scd40_shared.co2));
    strcpy(roomsense_iq_shared.co2_status, co2_actionable_insight(roomsense_iq_shared.scd40_shared.co2));
    strcpy(roomsense_iq_shared.pm_status, pm_actionable_insight(roomsense_iq_shared.mpm10_shared.pm1,
    	   roomsense_iq_shared.mpm10_shared.pm25, roomsense_iq_shared.mpm10_shared.pm10));
    strcpy(roomsense_iq_shared.mold_risk_status, dew_point_actionable_insight(roomsense_iq_shared.scd40_shared.temperature, roomsense_iq_shared.scd40_shared.dew_point));



	co_sensor_task_start();

	frc_correction = (uint16_t *)malloc(sizeof(uint16_t));

	// setup MPM10
	memset(&mpm, 0, sizeof(mpm));
	mpm10_init_desc(&mpm, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);

	result = ESP_ERROR_CHECK_WITHOUT_ABORT(measure_pm(&mpm, PM_ONE_REG, &data));
    if (result == ESP_OK) {
    	roomsense_iq_shared.pm_connection = true;
    	ESP_LOGI(TAG, "PM passed");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    else {
    	roomsense_iq_shared.pm_connection = false;
    	ESP_LOGE(TAG, "PM reading error results %d (%s)", result, esp_err_to_name(result));
    }

	// 1- setup SCD4x
	memset(&scd, 0, sizeof(scd));
	ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_init_desc(&scd, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

	//2- Stop measurment
	ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_stop_periodic_measurement(&(scd.i2c_dev)));

	//3- Self test
	result = scd4x_perform_self_test(&(scd.i2c_dev), &malfunction);
    if (result == ESP_OK)
    {
    	roomsense_iq_shared.t_h_co2_connection = true;
    	ESP_LOGI(TAG, "scd4x_perform_self_test passed successfully.\n");
    }
    else
    {
    	roomsense_iq_shared.t_h_co2_connection = false;
    	ESP_LOGI(TAG, "scd4x_perform_self_test failed.\n");
    }

    //4 - re-init
    ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_reinit(&(scd.i2c_dev)));

    //5- read serial number
	ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_get_serial_number(&(scd.i2c_dev), serial, serial + 1, serial + 2));
	ESP_LOGI(TAG, "Sensor serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

	//6- Apply Temperature offset
	ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_stop_periodic_measurement(&(scd.i2c_dev)));
	ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_set_temperature_offset(&(scd.i2c_dev), 12.1)); //offset =7 for large vent enclosure
	ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_get_temperature_offset(&(scd.i2c_dev), &t_offset));


	//7- start periodic measurement
	ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_start_periodic_measurement(&(scd.i2c_dev)));
	vTaskDelay(pdMS_TO_TICKS(1000));

	// setup SGP40
	memset(&sgp, 0, sizeof(sgp));
	ESP_ERROR_CHECK_WITHOUT_ABORT(sgp40_init_desc(&sgp, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
	ESP_ERROR_CHECK_WITHOUT_ABORT(sgp40_init(&sgp));
	ESP_LOGI(TAG, "SGP40 initilalized. Serial: 0x%04x%04x%04x", sgp.serial[0], sgp.serial[1], sgp.serial[2]);

	vTaskDelay(pdMS_TO_TICKS(10000));

	while (sgp40_retry_count < MAX_SGP40_RETRIES) //try up to MAX_SGP40_RETRIES to talk to CO2 sensor on the climatesense board.
	{
		result = sgp40_measure_raw(&sgp, scd.humidity, scd.temperature, &voc_index);

		if (result == ESP_OK)
		{
			roomsense_iq_shared.voc_connection = true;
			ESP_LOGI(TAG, "SGP40 self test passed!");
			break;
		}
		else
		{
			ESP_LOGI(TAG, "SGP40 self test failed! (Error Code: %d). Retrying...\n", result);
			sgp40_retry_count++;
		}

		vTaskDelay(pdMS_TO_TICKS(1000));  // Optional delay between retries
	}

	if (sgp40_retry_count == MAX_SGP40_RETRIES)
	{
		ESP_LOGI(TAG, "Reached maximum SGP40 self test retries.\n");
		roomsense_iq_shared.voc_connection = false;
	}

	if(roomsense_iq_shared.pm_connection  || roomsense_iq_shared.t_h_co2_connection  || roomsense_iq_shared.voc_connection)
	{
		roomsense_iq_shared.climatesense_connection = true;
		ESP_LOGI(TAG, "climatesense_connection = true");
	}
	else
	{
		roomsense_iq_shared.climatesense_connection = false;
		ESP_LOGI(TAG, "climatesense_connection = false");
	}

	// Wait until all set up
	vTaskDelay(pdMS_TO_TICKS(1000));

	TickType_t last_wakeup = xTaskGetTickCount();

	roomsense_iq_shared.forced_calibration = false;

	// Each complete scan of sensors takes around 4.5 seconds
	while (1)
	{
		if(alert_counter++ > ALERT_WARM_UP_PERIOD)
			alert_counter = 125;

		if (roomsense_iq_shared.climatesense_connection) //read from climatesense sensors only if the board is there.
		{
			//mpm10
			result = ESP_ERROR_CHECK_WITHOUT_ABORT(measure_pm(&mpm, PM_ONE_REG, &data));
		    if (result == ESP_OK) {
				roomsense_iq_shared.mpm10_shared.pm1 = data;
				vTaskDelay(1000 / portTICK_PERIOD_MS);
		    } else {
		    	ESP_LOGE(TAG, "PM1 Error reading results %d (%s)", result, esp_err_to_name(result));
		    }


		    result = ESP_ERROR_CHECK_WITHOUT_ABORT(measure_pm(&mpm, PM_TWO_AND_HALF_REG, &data));
		    if (result == ESP_OK) {
				roomsense_iq_shared.mpm10_shared.pm25 = data;
				vTaskDelay(1000 / portTICK_PERIOD_MS);
		    } else {
		    	ESP_LOGE(TAG, "PM2.5 Error reading results %d (%s)", result, esp_err_to_name(result));
		    }


			result = ESP_ERROR_CHECK_WITHOUT_ABORT(measure_pm(&mpm, PM_TEN_REG, &data));
		    if (result == ESP_OK) {
				roomsense_iq_shared.mpm10_shared.pm10 = data;
				vTaskDelay(1000 / portTICK_PERIOD_MS);
		    } else {
		    	ESP_LOGE(TAG, "PM10 Error reading results %d (%s)", result, esp_err_to_name(result));
		    }


			// SDC40
			result = scd4x_read_measurement(&(scd.i2c_dev), &(scd.co2), &(scd.temperature), &(scd.humidity));
			if (result != ESP_OK)
			{
				ESP_LOGE(TAG, "Error reading results %d (%s)", result, esp_err_to_name(result));
				continue;
			}
			if (scd.co2 == 0)
			{
				ESP_LOGW(TAG, "Invalid sample detected, skipping");
				//continue;
			}
			roomsense_iq_shared.scd40_shared.temperature = scd.temperature;
			roomsense_iq_shared.scd40_shared.humidity = scd.humidity;
			roomsense_iq_shared.scd40_shared.co2 = scd.co2;
			vTaskDelay(1000 / portTICK_PERIOD_MS);

			if(roomsense_iq_shared.forced_calibration)
			{
				calibration_counter++;
				if(calibration_counter > 30) //Operate the SCD4x in the periodic measurement for at least 3 minutes
				{
					calibration_counter = 0;
					int16_t correction;
					esp_err_t err;

					err = scd4x_init_desc_native(&(scd.i2c_dev), 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL);
					ESP_ERROR_CHECK_WITHOUT_ABORT(err);
					if (err != ESP_OK)
					{
						ESP_LOGI(TAG, "scd4x_init_desc_native! Error code: 0x%x\n", err);
						// Perform error handling or recovery here
					}
					else
					{
						ESP_LOGI(TAG, "scd4x_init_desc_native No error occurred.\n");
					}
					//////////////////stop/////////////////////
					err = scd4x_stop_periodic_measurement(&(scd.i2c_dev));
					ESP_ERROR_CHECK_WITHOUT_ABORT(err);
					if (err != ESP_OK)
					{
						ESP_LOGI(TAG, "scd4x_stop_periodic_measurement! Error code: 0x%x\n", err);
					}
					else
					{
						ESP_LOGI(TAG, "scd4x_stop_periodic_measurement No error occurred.\n");
					}

					vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait 1000 ms for the command to complete.
					err = scd4x_perform_forced_recalibration(&(scd.i2c_dev), 421, frc_correction); // Assume CO2 in fresh air 421 ppm
					if (err != ESP_OK)
					{
						ESP_LOGE(TAG, "Error perform_forced_recalibration results %d (%s)", err, esp_err_to_name(err));
						continue;
					}

					correction = *frc_correction - 0x8000;
					ESP_LOGI(TAG, "------------------> The FRC correction is %d", correction);
					scd4x_measure_single_shot(&(scd.i2c_dev));
					roomsense_iq_shared.forced_calibration = false;
					ESP_ERROR_CHECK_WITHOUT_ABORT(scd4x_start_periodic_measurement(&(scd.i2c_dev)));
					ESP_LOGI(TAG, "Periodic measurements started");
					vTaskDelay(pdMS_TO_TICKS(1000));
				}

			}


			roomsense_iq_shared.scd40_shared.dew_point = calculate_dew_point(scd.temperature, scd.humidity);

			// Feed T and H to SGP40 for compensation
			ESP_ERROR_CHECK_WITHOUT_ABORT(sgp40_measure_voc(&sgp, scd.humidity, scd.temperature, &voc_index));
			roomsense_iq_shared.sgp40_shared.voc_index = voc_index;

			if(check_time_elapsed(&climatesense_pkt_tx_timer,1000*60*15))
			{
				ESP_LOGI(TAG,"updating climatesense data buffers");
				write_to_buffer(&buffer_temperature,roomsense_iq_shared.scd40_shared.temperature);
				write_to_buffer(&buffer_humidity,roomsense_iq_shared.scd40_shared.humidity);
				write_to_buffer(&buffer_voc,roomsense_iq_shared.sgp40_shared.voc_index);
				write_to_buffer(&buffer_pm1,roomsense_iq_shared.mpm10_shared.pm1);
				write_to_buffer(&buffer_pm2_5,roomsense_iq_shared.mpm10_shared.pm25);
				write_to_buffer(&buffer_pm10,roomsense_iq_shared.mpm10_shared.pm10);
				write_to_buffer(&buffer_co2,roomsense_iq_shared.scd40_shared.co2);
				write_to_buffer(&buffer_co,roomsense_iq_shared.tgs5141_shared.co);
			}


//	        ESP_LOGI(TAG, "Temperature: %.2f °C", roomsense_iq_shared.scd40_shared.temperature);
//	        ESP_LOGI(TAG, "Humidity: %.2f %%", roomsense_iq_shared.scd40_shared.humidity);
//	        ESP_LOGI(TAG, "Due Point: %.2f °C", roomsense_iq_shared.scd40_shared.dew_point);
//	        ESP_LOGI(TAG, "CO2: %u ppm", roomsense_iq_shared.scd40_shared.co2);
//	        ESP_LOGI(TAG, "VOC index: %d", roomsense_iq_shared.sgp40_shared.voc_index);
//	        ESP_LOGI(TAG, "CO %d", roomsense_iq_shared.tgs5141_shared.co);
//	        ESP_LOGI(TAG, "PM1.0 %d", roomsense_iq_shared.mpm10_shared.pm1);
//	        ESP_LOGI(TAG, "PM2.5 %d", roomsense_iq_shared.mpm10_shared.pm25);
//	        ESP_LOGI(TAG, "PM10 %d", roomsense_iq_shared.mpm10_shared.pm10);

            //Copy actionable insights to global variables for task_climate_sense_publish() publishes every CLIMATE_SENSE_REFRESH_RATE_MS
	        strcpy(roomsense_iq_shared.humidity_status, humidity_actionable_insight(roomsense_iq_shared.scd40_shared.humidity));
	        strcpy(roomsense_iq_shared.voc_status, voc_actionable_insight(voc_index, roomsense_iq_shared.scd40_shared.co2));
	        strcpy(roomsense_iq_shared.co2_status, co2_actionable_insight(roomsense_iq_shared.scd40_shared.co2));
	        strcpy(roomsense_iq_shared.pm_status, pm_actionable_insight(roomsense_iq_shared.mpm10_shared.pm1,
	        	   roomsense_iq_shared.mpm10_shared.pm25, roomsense_iq_shared.mpm10_shared.pm10));
	        strcpy(roomsense_iq_shared.mold_risk_status, dew_point_actionable_insight(roomsense_iq_shared.scd40_shared.temperature, roomsense_iq_shared.scd40_shared.dew_point));


//			copy_to_data_buffer(roomsense_iq_shared.co2_status,co2_actionable_insight(roomsense_iq_shared.scd40_shared.co2));
//			copy_to_data_buffer(roomsense_iq_shared.co_status,co_actionable_insight(roomsense_iq_shared.tgs5141_shared.co));
//			copy_to_data_buffer(roomsense_iq_shared.voc_status,voc_actionable_insight(voc_index, roomsense_iq_shared.scd40_shared.co2));
//			copy_to_data_buffer(roomsense_iq_shared.pm_status,pm_actionable_insight(roomsense_iq_shared.mpm10_shared.pm1, roomsense_iq_shared.mpm10_shared.pm25, roomsense_iq_shared.mpm10_shared.pm10));
//			copy_to_data_buffer(roomsense_iq_shared.humidity_status,humidity_actionable_insight(roomsense_iq_shared.scd40_shared.humidity));
//			copy_to_data_buffer(roomsense_iq_shared.mold_risk_status,dew_point_actionable_insight(roomsense_iq_shared.scd40_shared.temperature, roomsense_iq_shared.scd40_shared.dew_point));

			update_action_point(CO2_AP,roomsense_iq_shared.co2_status);
			update_action_point(CO_AP,roomsense_iq_shared.co_status);
			update_action_point(VOC_AP,roomsense_iq_shared.voc_status);
			update_action_point(PM_AP,roomsense_iq_shared.pm_status);
			update_action_point(HUMIDITY_AP,roomsense_iq_shared.humidity_status);
	    	update_action_point(STATUS_AP,roomsense_iq_shared.mold_risk_status);

			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		else
		{
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}
}
	void climatesense_task_start(void)
	{
		ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
		xTaskCreatePinnedToCore(&climatesense_task, "climatesense_task", CLIMATESENSE_TASK_STACK_SIZE, NULL, CLIMATESENSE_TASK_PRIORITY, NULL,
		CLIMATESENSE_TASK_CORE_ID);
	}

