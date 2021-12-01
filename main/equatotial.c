#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include "bsp.h"


static void uart_task(void *arg)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	uint8_t retData[5];

	while (1) 
	{
		int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, 30 / portTICK_RATE_MS);
		vTaskDelay(50/portTICK_RATE_MS);

		if(len != 0)
		{	
			printf("data length = %d\n", len);
		}
	}
}

static void led_task(void *arg)
{
	gpio_reset_pin(LED);
	gpio_set_direction(LED, GPIO_MODE_OUTPUT);

	while(1)
	{
		gpio_set_level(LED, 0);
		vTaskDelay(500/portTICK_RATE_MS);
		gpio_set_level(LED, 1);
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

static void motor_test_task(void *arg)
{
	while(1)
	{
		gpio_set_level(RA_STEP, 1);
		gpio_set_level(DEC_STEP, 1);
		vTaskDelay(1/portTICK_RATE_MS);
		gpio_set_level(RA_STEP, 0);
		gpio_set_level(DEC_STEP, 0);
		vTaskDelay(1/portTICK_RATE_MS);
	}
}

void app_main(void)
{
	gpios_init();				//初始化GPIO
	//uart_init();				//初始化通信串口
	xTaskCreate(led_task, "led_task", 2048, NULL, 20, NULL);
	//xTaskCreate(uart_task, "uart_task", UART_TASK_STACK_SIZE, NULL, 10, NULL);
	xTaskCreate(motor_test_task, "motor_test_task", 2048, NULL, 14, NULL);
}
