#include <stdbool.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>


QueueHandle_t distance_queue_handle;

void direction_calculation_task(void *pvParameters);
void direction_detection_task_start();
