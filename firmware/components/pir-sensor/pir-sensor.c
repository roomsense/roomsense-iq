#include <stdio.h>
#include "pir-sensor.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <freertos/semphr.h>
#include "tasks_common.h"
#include "/home/builder/esp/esp-idf/components/esp_adc_cal/include/esp_adc_cal.h"
#include "freertos/event_groups.h"

#include <inttypes.h>

static const char *TAG = "PIR";

#define PIR_PIN		48
#define ESP_INTR_FLAG_DEFAULT	0


EventGroupHandle_t pir_group = NULL;
const int PIR_BIT = BIT4;

// Semaphore handle
SemaphoreHandle_t pir_semphore = NULL;


void IRAM_ATTR pir_isr_handler(void *arg)
{
	// Notify pir task by releasing the semaphore
	xSemaphoreGiveFromISR(pir_semphore, NULL);
}

void pir_task(void *arg)
{
	uint32_t io_num;

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = PRESENCE_ENABLE_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    if (roomsense_iq_shared.led_enable && roomsense_iq_shared.climatesense_connection)
    {
        gpio_set_level(PRESENCE_ENABLE_PIN, 1);
    }
    else
    {
    	gpio_set_level(PRESENCE_ENABLE_PIN, 0);
    }

	for (;;)
	{
		if (xSemaphoreTake(pir_semphore, portMAX_DELAY) == pdTRUE)
		{
			roomsense_iq_shared.pir_sensor_shared.g_pir_status = 1;
			if(pir_group != NULL)
			{
			    xEventGroupSetBits(pir_group, PIR_BIT);

			    if (!roomsense_iq_shared.climatesense_connection)
			    {
			        gpio_set_level(PRESENCE_ENABLE_PIN, 1);
			    }
			    else if (roomsense_iq_shared.led_enable)
				{
			    	gpio_set_level(PRESENCE_ENABLE_PIN, 1);
				}
			    else
			    {
			    	gpio_set_level(PRESENCE_ENABLE_PIN, 0);
			    }
			    vTaskDelay(2000 / portTICK_PERIOD_MS);
			    roomsense_iq_shared.pir_sensor_shared.g_pir_status = 0;
			    xEventGroupSetBits(pir_group, PIR_BIT);
			    vTaskDelay(4000 / portTICK_PERIOD_MS);
			}
		}
	}
}
void pir_task_start()
{
	pir_semphore = xSemaphoreCreateBinary();

	// Configure the button and set the direction
	gpio_pad_select_gpio(PIR_PIN);
	gpio_set_direction(PIR_PIN, GPIO_MODE_INPUT);

	// Enable interrupt on the negative edge
	gpio_set_intr_type(PIR_PIN, GPIO_INTR_POSEDGE);

	gpio_set_pull_mode(PIR_PIN, GPIO_PULLDOWN_ONLY);

	// Install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

	// Attach the interrupt service routine
	gpio_isr_handler_add(PIR_PIN, pir_isr_handler, NULL);

	esp_log_level_set("PIR", ESP_LOG_INFO);
	xTaskCreatePinnedToCore(&pir_task, "pir_task", PIR_TASK_STACK_SIZE, NULL, PIR_TASK_PRIORITY, NULL, PIR_TASK_CORE_ID);

}
