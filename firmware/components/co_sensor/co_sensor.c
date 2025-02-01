#include "driver/adc.h"
#include <co_sensor.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include <esp_log.h>
#include <string.h>
#include "tasks_common.h"
#include "/home/builder/esp/esp-idf/components/esp_adc_cal/include/esp_adc_cal.h"

static const char *TAG = "co-sensor";


#define NO_OF_SAMPLES      5           //Multisampling
#define CO_ENABLE_PIN      18
#define CO_ENABLE_PIN_SEL  (1ULL<<CO_ENABLE_PIN)
#define ADC2_CO_CHANNEL    ADC2_CHANNEL_9

CircularBuffer buffer_co;
extern EventGroupHandle_t event_group_alert;

static const char* co_actionable_insight(uint32_t co)
{
	if (co > 20)
	{
		xEventGroupSetBits(event_group_alert, CO_ALERT);
		roomsense_iq_shared.rgb_led_alert = EXTRME_AIRBORNE_POLLUTION;
		roomsense_iq_shared.climate_alarm = true;
		return "Carbon Monoxide Detected!";
	}

	xEventGroupClearBits(event_group_alert, CO_ALERT);
	roomsense_iq_shared.climate_alarm = false;
	return "No Alarm";
}

void co_sensor_task(void *pvParameter)
{
	int retry_counter = 0;
	int32_t co_value;

	uint32_t voltage;
	uint32_t adc_reading = 0, counter = 0;
	int raw;
	int read_raw;
	esp_err_t r;
	gpio_num_t adc_gpio_num;

//    //zero-initialize the config structure.
//    gpio_config_t io_conf = {};
//    //disable interrupt
//    io_conf.intr_type = GPIO_INTR_DISABLE;
//    //set as output mode
//    io_conf.mode = GPIO_MODE_OUTPUT;
//    //bit mask of the pins that you want to set,e.g.GPIO18/19
//    io_conf.pin_bit_mask = CO_ENABLE_PIN_SEL;
//    //disable pull-down mode
//    io_conf.pull_down_en = 0;
//    //disable pull-up mode
//    io_conf.pull_up_en = 0;
//    //configure GPIO with the given settings
//    gpio_config(&io_conf);
//
//    gpio_set_level(CO_ENABLE_PIN, 1);

	r = adc2_pad_get_io_num( ADC2_CO_CHANNEL, &adc_gpio_num);
	assert(r == ESP_OK);
	adc2_config_channel_atten( ADC2_CO_CHANNEL, ADC_ATTEN_11db);
	vTaskDelay(2 * portTICK_PERIOD_MS);
	adc_power_acquire();

	while (1)
	{
		r = adc2_get_raw( ADC2_CO_CHANNEL, ADC_WIDTH_BIT_12, &read_raw);
		if (r == ESP_OK)
		{

			adc_reading += read_raw;
			counter++;
			if (counter == NO_OF_SAMPLES) //5
			{
				adc_reading /= NO_OF_SAMPLES;

				//convert to calibrated value
				co_value = (int32_t) 1.11 * (adc_reading - 30);
				if (co_value < 0)
				{
					co_value = 0;
				}
				if(roomsense_iq_shared.climatesense_connection == true)
				{
					roomsense_iq_shared.tgs5141_shared.co = co_value;
				}
				else
				{
					roomsense_iq_shared.tgs5141_shared.co = 0;
				}

				strcpy(roomsense_iq_shared.co_status, co_actionable_insight(roomsense_iq_shared.tgs5141_shared.co));
				adc_reading = 0;
				counter = 0;
			}

		}
		else if (r == ESP_ERR_INVALID_STATE)
		{
			printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
		}
		else if (r == ESP_ERR_TIMEOUT)
		{
			//This can not happen in this example. But if WiFi is in use, such error code could be returned.
			printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
		}
		else
		{
			printf("%s\n", esp_err_to_name(r));
		}

		vTaskDelay(20 * portTICK_PERIOD_MS);
	}
}

void co_sensor_task_start()
{
	ESP_LOGI(TAG, "STARTING CO Sensor task...");
	esp_log_level_set("CO Sensor", ESP_LOG_INFO);
	xTaskCreatePinnedToCore(&co_sensor_task, "co_sensor_task", CO_SENSOR_TASK_STACK_SIZE, NULL, CO_SENSOR_TASK_PRIORITY, NULL, CO_SENSOR_TASK_CORE_ID);
}

