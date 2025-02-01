/*
 * adafruit-161.h
 *
 *  Created on: Sep. 17, 2022
 *      Author: sinam
 */

#include <stdbool.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include "../../sht30/include/sht30.h"


#ifdef __cplusplus
extern "C" {
#endif

#define BUFFER_SIZE_LIGHT 96


void adc_init();
uint32_t adc_read();
void ada161_task(void *pvParameter);
void ada161_task_start();
