#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp_log.h>
#include "tasks_common.h"
#include "direction-detection.h"
#include "../../ld2410/include/ld2410.h"
#include "freertos/event_groups.h"

#define NOMOVEMENT 0
#define AWAY       1
#define TOWARDS    2

EventGroupHandle_t movement_direction_group = NULL;
const int MOVEMENT_DIRECTION_BIT = BIT3;

extern bool macro_movement;

void direction_calculation_task(void *pvParameters)
{
	uint8_t state;
	uint16_t previousDistance = 0;
	uint16_t currentDistance = 0;
	uint16_t filtered_output;
	int distanceChange;
	int i;

	state = NOMOVEMENT;

	while (1)
	{
		vTaskDelay(100 / portTICK_RATE_MS);

		switch (state)
		{
		case NOMOVEMENT:
			previousDistance = roomsense_iq_shared.ld2410_data_shared.detected_distance;
			//only if there is a Macro movement then we go for the direction calculation
			if (roomsense_iq_shared.direction_detection_shared.macro_movement || roomsense_iq_shared.pir_sensor_shared.g_pir_status)
			{
				vTaskDelay(1000 / portTICK_RATE_MS);
				currentDistance = roomsense_iq_shared.ld2410_data_shared.detected_distance;
				distanceChange = abs(currentDistance - previousDistance);
				roomsense_iq_shared.direction_detection_shared.macro_movement = 0;

				if (distanceChange > DISTANCE_RESOLUTION)
				{
					if (currentDistance > previousDistance)
					{
						state = AWAY;
						roomsense_iq_shared.direction_detection_shared.g_movement_direction = 1;
						if(movement_direction_group != NULL)
						{
						    xEventGroupSetBits(movement_direction_group, MOVEMENT_DIRECTION_BIT);
						}
					}
					else if (currentDistance < previousDistance)
					{
						state = TOWARDS;
						roomsense_iq_shared.direction_detection_shared.g_movement_direction = 2;
						if(movement_direction_group != NULL)
						{
						    xEventGroupSetBits(movement_direction_group, MOVEMENT_DIRECTION_BIT);
						}
					}
				}
				else
				{
					roomsense_iq_shared.direction_detection_shared.g_movement_direction = 0;
					if(movement_direction_group != NULL)
					{
					    xEventGroupSetBits(movement_direction_group, MOVEMENT_DIRECTION_BIT);
					}
				}
			}
			break;

		case AWAY:
			vTaskDelay(1000 / portTICK_RATE_MS);
			state = NOMOVEMENT;
			roomsense_iq_shared.direction_detection_shared.g_movement_direction = 0;
			if(movement_direction_group != NULL)
			{
			    xEventGroupSetBits(movement_direction_group, MOVEMENT_DIRECTION_BIT);
			}
			break;

		case TOWARDS:
			vTaskDelay(1000 / portTICK_RATE_MS);
			state = NOMOVEMENT;
			roomsense_iq_shared.direction_detection_shared.g_movement_direction = 0;
			if(movement_direction_group != NULL)
			{
			    xEventGroupSetBits(movement_direction_group, MOVEMENT_DIRECTION_BIT);
			}
			break;
		}
	}
}

void direction_detection_task_start()
{

	esp_log_level_set("DIR_DECTECTION", ESP_LOG_INFO);

	distance_queue_handle = xQueueCreate(3, sizeof(uint16_t));

	xTaskCreatePinnedToCore(&direction_calculation_task, "direction_calculation_task", DIRECTION_DETECTION_TASK_STACK_SIZE, NULL,
	DIRECTION_DETECTION_TASK_PRIORITY, NULL, DIRECTION_DETECTION_TASK_CORE_ID);

}
