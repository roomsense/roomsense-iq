#include "driver/adc.h"
#include <adafruit-161.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "tasks_common.h"
#include "esp_adc_cal.h"

static const char *TAG = "ada161";

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling


#define ONE_MINUTE_MS 60000
#define FIFTEEN_MINUTES_MS 900000

SemaphoreHandle_t xMutex_g_light_density_raw = NULL;

static esp_adc_cal_characteristics_t adc_chars;

CircularBuffer buffer_light;

static const adc_channel_t channel = ADC_CHANNEL_4;
static const adc_atten_t atten     = ADC_ATTEN_DB_11;
static const adc_unit_t unit       = ADC_UNIT_1;

void adc_init()
{
	//Configure ADC
	if (unit == ADC_UNIT_1)
	{
		adc1_config_width(ADC_WIDTH_BIT_12);
		adc1_config_channel_atten(channel, atten);
	}
	else
	{
		adc2_config_channel_atten((adc2_channel_t) channel, atten);
	}

	//Characterize ADC
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
	if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
	{
		printf("ADC characterized using Two Point Value\n");
	}
	else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
	{
		printf("ADC characterized using eFuse Vref\n");
	}
	else
	{
		printf("ADC characterized using Default Vref\n");
	}
}

uint32_t adc_read()
{
	uint32_t voltage;
	uint32_t adc_reading = 0;

	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++)
	{
		if (unit == ADC_UNIT_1)
		{
			adc_reading += adc1_get_raw((adc1_channel_t) channel);
		}
		else
		{
			int raw;
			adc2_get_raw((adc2_channel_t) channel, ADC_WIDTH_BIT_12, &raw);
			adc_reading += raw;
		}
		//vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	adc_reading /= NO_OF_SAMPLES;

	//Convert adc_reading to voltage in mV
	voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
	//printf("raw =%d, vol=%d\n", adc_reading, voltage);

	return adc_reading;
}

void ada161_task(void *pvParameter)
{
	int retry_counter = 0;
	uint32_t pre_value = 20, adc_reading, counter =0;
	int32_t diff;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    ESP_ERROR_CHECK(esp_task_wdt_status(NULL));
     init_buffer(&buffer_light);

	while (true)
	{
		ESP_ERROR_CHECK(esp_task_wdt_reset());

		if (xMutex_g_light_density_raw != NULL)
		{
			if (xSemaphoreTake(xMutex_g_light_density_raw, (TickType_t) 10) == pdTRUE)
			{
				adc_reading = adc_read();
				diff = pre_value - adc_reading;

				//filter out glitches in ADC readings
				if(diff > 40 && counter < 2)
				{
					adc_reading = pre_value;
					counter ++;
				}
				else
				{
					counter = 0;
				}
				roomsense_iq_shared.adafruit_161_shared.light_density_raw = adc_reading;

				write_to_buffer(&buffer_light, (float) roomsense_iq_shared.adafruit_161_shared.light_density_raw);

				pre_value = adc_reading;

				if (xTaskGetTickCount() % (FIFTEEN_MINUTES_MS / portTICK_PERIOD_MS) == 0) {
					//addToBuffer(buffer_light, g_light_density_raw);
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

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void ada161_task_start()
{

	xMutex_g_light_density_raw = xSemaphoreCreateMutex();
	if (xMutex_g_light_density_raw == NULL)
	{
		ESP_LOGI("Main", "Error: unable to create xMutex_g_light_density_raw");
	}
	else
	{
		adc_init();
		ESP_LOGI(TAG, "STARTING ADA161...");
		esp_log_level_set("ada161", ESP_LOG_INFO);

		// Start the ADA161 application task
		xTaskCreatePinnedToCore(&ada161_task, "ada161_task", ADA161_TASK_STACK_SIZE, NULL, ADA161_TASK_PRIORITY, NULL, ADA161_TASK_CORE_ID);
	}
}

