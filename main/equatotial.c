#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include "bsp.h"
#include "oled.h"


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

static void tp_example_read_task(void *pvParameter)
{
 

    while (1) 
	{

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

#include "bmp.h"
static void oled_task(void *pvParameter)
{
	vTaskDelay(200/portTICK_RATE_MS);
	OLED_Init();	

	OLED_ShowCHinese(0,0,0);//中
	OLED_ShowCHinese(18,0,1);//景
	OLED_ShowCHinese(36,0,2);//园
	OLED_ShowCHinese(54,0,3);//电
	OLED_ShowCHinese(72,0,4);//子
	OLED_ShowCHinese(90,0,5);//科
	OLED_ShowCHinese(108,0,6);//技

	vTaskDelay(1000/portTICK_RATE_MS);

	while(1)
	{
		OLED_Clear();
		OLED_ShowString(0,0,(unsigned char *)"0.91OLEDTEST",8);
		OLED_ShowString(0,1,(unsigned char *)"0123456789AB",8);
		vTaskDelay(1000/portTICK_RATE_MS);

		OLED_Clear();
		OLED_ShowString(0,0,(unsigned char *)"0.91OLEDTEST",16);
		vTaskDelay(1000/portTICK_RATE_MS);

		OLED_Clear();
		OLED_ShowString(0,0,(unsigned char *)"0123456789AB",16);
		vTaskDelay(1000/portTICK_RATE_MS);

		OLED_Clear();
		for(int i=0; i<10; i++)
		{
			OLED_ShowCHinese(4,i,0);//中
			OLED_ShowCHinese(22,i,1);//景
			OLED_ShowCHinese(40,i,2);//园
			OLED_ShowCHinese(58,i,3);//电
			OLED_ShowCHinese(76,i,4);//子
			OLED_ShowCHinese(94,i,5);//科
			OLED_ShowCHinese(112,i,6);//技
			vTaskDelay(100/portTICK_RATE_MS);
			OLED_Clear();

			if(i==9)
			 	i=0;
		}

		OLED_Clear();
		OLED_DrawBMP(0,0,128,8,BMP1);
		vTaskDelay(5000/portTICK_RATE_MS);
		OLED_Clear();
		vTaskDelay(5000/portTICK_RATE_MS);
	}
}

void app_main(void)
{
	gpios_init();				//初始化GPIO
	//uart_init();				//初始化通信串口
	i2c_master_init();
	xTaskCreate(led_task, "led_task", 2048, NULL, 20, NULL);
	xTaskCreate(motor_test_task, "motor_test_task", 2048, NULL, 14, NULL);
	xTaskCreate(tp_example_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
	xTaskCreate(oled_task, "oled_task", 2048, NULL, 15, NULL);
}
