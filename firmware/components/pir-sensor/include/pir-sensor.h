#include <stdbool.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>



typedef enum pir_state
{
	PIR_START_TIMER = 0,
	PIR_READ_PULSE,
	PULSE_VALIDATION,
    PIR_TASK_DELAY,
} pir_state_e;

void adc_pir_init();
uint32_t adc_pir_read();
void pir_task_start();
void pir_task(void *arg);
