#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "watchdog.h"
#include "tasks_common.h"

void watchdog_start()
{

	//Initialize or reinitialize TWDT
	ESP_ERROR_CHECK(esp_task_wdt_init(WDT_TIMEOUT_SEC, true));

	//Subscribe Idle Tasks to TWDT if they were not subscribed at startup
#ifndef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
	esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 && !CONFIG_FREERTOS_UNICORE
    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));
#endif

}

