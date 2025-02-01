/*
 * rgb_led.c
 *
 *  Created on: Nov. 30, 2022
 *      Author: builder
 */

#include <stdbool.h>
#include "driver/ledc.h"
#include "rgb_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tasks_common.h"
#include "freertos/event_groups.h"

#define LED_DENSITY 10 // Higher LED_DENSITY means lower density

#define ALL_BITS 0xFFFFFF  // 24 bits set to 1

// handle for rgb_led_pwm_init
bool g_pwm_init_handle = false;

extern EventGroupHandle_t led_event_group;
extern EventGroupHandle_t event_group_alert;
extern const int RGB_LED_BIT;

/**
 * Initializes the RGB LED settings per channel, including
 * the GPIO for each color, mode and timer configuration.
 */
static void rgb_led_pwm_init(void)
{
	int rgb_ch;

	// Red
	ledc_ch[0].channel = LEDC_CHANNEL_0;
	ledc_ch[0].gpio = RGB_LED_RED_GPIO;
	ledc_ch[0].mode = LEDC_LOW_SPEED_MODE;
	ledc_ch[0].timer_index = LEDC_TIMER_0;

	// Green
	ledc_ch[1].channel = LEDC_CHANNEL_1;
	ledc_ch[1].gpio = RGB_LED_GREEN_GPIO;
	ledc_ch[1].mode = LEDC_LOW_SPEED_MODE;
	ledc_ch[1].timer_index = LEDC_TIMER_0;

	// Blue
	ledc_ch[2].channel = LEDC_CHANNEL_2;
	ledc_ch[2].gpio = RGB_LED_BLUE_GPIO;
	ledc_ch[2].mode = LEDC_LOW_SPEED_MODE;
	ledc_ch[2].timer_index = LEDC_TIMER_0;

	// Configure timer zero
	ledc_timer_config_t ledc_timer =
	{ .duty_resolution = LEDC_TIMER_8_BIT, .freq_hz = 100, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0 };
	ledc_timer_config(&ledc_timer);

	// Configure channels
	for (rgb_ch = 0; rgb_ch < RGB_LED_CHANNEL_NUM; rgb_ch++)
	{
		ledc_channel_config_t ledc_channel =
		{ .channel = ledc_ch[rgb_ch].channel, .duty = 0, .hpoint = 0, .gpio_num = ledc_ch[rgb_ch].gpio, .intr_type = LEDC_INTR_DISABLE, .speed_mode =
				ledc_ch[rgb_ch].mode, .timer_sel = ledc_ch[rgb_ch].timer_index, .flags =
		{ .output_invert = 1 } };
		ledc_channel_config(&ledc_channel);
	}

	g_pwm_init_handle = true;
}

/**
 * Sets the RGB color.
 */
static void rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
	// Value should be 0 - 255 for 8 bit number
	ledc_set_duty(ledc_ch[0].mode, ledc_ch[0].channel, red);
	ledc_update_duty(ledc_ch[0].mode, ledc_ch[0].channel);

	ledc_set_duty(ledc_ch[1].mode, ledc_ch[1].channel, green);
	ledc_update_duty(ledc_ch[1].mode, ledc_ch[1].channel);

	ledc_set_duty(ledc_ch[2].mode, ledc_ch[2].channel, blue);
	ledc_update_duty(ledc_ch[2].mode, ledc_ch[2].channel);
}

void rgb_led_wifi_app_started(void)
{
	if (g_pwm_init_handle == false)
	{
		rgb_led_pwm_init();
	}

	rgb_led_set_color(255, 102, 255);
}

void rgb_led_http_server_started(void)
{
	if (g_pwm_init_handle == false)
	{
		rgb_led_pwm_init();
	}

	rgb_led_set_color(204, 255, 51);
}

void rgb_led_wifi_connected(void)
{
	if (g_pwm_init_handle == false)
	{
		rgb_led_pwm_init();
	}

	rgb_led_set_color(0, 255, 153);
}

void rgb_led_task(void *pvParameters)
{
	EventBits_t current_bits;

	xEventGroupClearBits(event_group_alert, 1 << 0);
	xEventGroupClearBits(event_group_alert, 1 << 1);
	xEventGroupClearBits(event_group_alert, 1 << 2);
	xEventGroupClearBits(event_group_alert, 1 << 3);
	xEventGroupClearBits(event_group_alert, 1 << 4);
	xEventGroupClearBits(event_group_alert, 1 << 5);
	xEventGroupClearBits(event_group_alert, 1 << 6);
	xEventGroupClearBits(event_group_alert, 1 << 7);

	while (true)
	{
		if (roomsense_iq_shared.led_enable)
		{
			current_bits = xEventGroupGetBits(event_group_alert);
			if (current_bits == 0)
			{
				switch (roomsense_iq_shared.rgb_led_state)
				{
				case WIFI_DISCONNECTED:
					if (roomsense_iq_shared.rgb_led_shared.reset_sw == false)
					{
						rgb_led_set_color(255 / LED_DENSITY, 120 / LED_DENSITY, 0 / LED_DENSITY);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 255 / 2);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
					}
					else if (roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy)
					{
						roomsense_iq_shared.rgb_led_state = OCCUPIED_LED;
						if (led_event_group != NULL)
						{
							xEventGroupSetBits(led_event_group, RGB_LED_BIT);
						}
					}
					else
					{
						roomsense_iq_shared.rgb_led_state = VACANT_LED;
						if (led_event_group != NULL)
						{
							xEventGroupSetBits(led_event_group, RGB_LED_BIT);
						}
					}
					break;

				case HTPP_DISCONNECTED:

					rgb_led_set_color(0 / LED_DENSITY, 255 / LED_DENSITY, 0 / LED_DENSITY);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 255 / 2);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					break;

				case WIFI_CONNECTED: ///Flashing Blue
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 255 / 2);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 0 / LED_DENSITY);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					break;

				case MQTT_CONNECTED: ///Stay Blue
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 255 / 2);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					break;

				case MQTT_DISCONNECTED: //Flashing Blue
					if (roomsense_iq_shared.rgb_led_shared.reset_sw == false)
					{
						rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 255 / 2);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 0 / LED_DENSITY);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
					}
					else if (roomsense_iq_shared.ha_mqtt_shared.nvs_occupancy)
					{
						roomsense_iq_shared.rgb_led_state = OCCUPIED_LED;
						if (led_event_group != NULL)
						{
							xEventGroupSetBits(led_event_group, RGB_LED_BIT);
						}
					}
					else
					{
						roomsense_iq_shared.rgb_led_state = VACANT_LED;
						if (led_event_group != NULL)
						{
							xEventGroupSetBits(led_event_group, RGB_LED_BIT);
						}
					}
					break;

				case VACANT_LED: ///Stay Blue
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 255 / 2);
					if (led_event_group != NULL)
					{
						xEventGroupWaitBits(led_event_group, RGB_LED_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
					}
					break;

				case OCCUPIED_LED: ///Stay ORAGNE
					rgb_led_set_color(255 / LED_DENSITY, 120 / LED_DENSITY, 0 / LED_DENSITY);
					if (led_event_group != NULL)
					{
						xEventGroupWaitBits(led_event_group, RGB_LED_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
					}
					break;
				}
			}
			else // Alert state
			{
				switch (roomsense_iq_shared.rgb_led_alert)
				{
				case HARDWARE_FAILING_LED:
					rgb_led_set_color(255 / 5, 0 / LED_DENSITY, 0 / LED_DENSITY);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 0 / LED_DENSITY);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					break;
				case Elevated_AIRBORNE_POLLUTION: // Flashing yellow
					rgb_led_set_color(200 / 5, 200 / 5, 0);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 0 / LED_DENSITY);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					break;
				case EXTRME_AIRBORNE_POLLUTION: // Flashing purple
					rgb_led_set_color(160 / 2, 32 / 2, 250 / 2);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					rgb_led_set_color(0 / LED_DENSITY, 0 / LED_DENSITY, 0 / LED_DENSITY);
					vTaskDelay(1000 / portTICK_PERIOD_MS);
					break;
				}
			}
		}
		else
		{
			rgb_led_set_color(0, 0, 0);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}
}
void rgb_led_start()
{

	if (g_pwm_init_handle == false)
	{
		rgb_led_pwm_init();
	}

	xTaskCreatePinnedToCore(&rgb_led_task, "rgb_led_task", RGB_LED_TASK_STACK_SIZE, NULL, RGB_LED_TASK_PRIORITY, NULL, RGB_LED_TASK_CORE_ID);
}

