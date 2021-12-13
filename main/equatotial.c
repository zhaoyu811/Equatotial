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

static void tp_example_read_task(void *pvParameter)
{
 

    while (1) {

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

#define u8 unsigned char 

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  
//开启OLED显示    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(255,OLED_DATA); 
	} //更新显示
}
void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
	} //更新显示
}

static void oled_task(void *pvParameter)
{
	vTaskDelay(200/portTICK_RATE_MS);
	OLED_WR_Byte(0xAE,OLED_CMD);//关闭显示
	
	OLED_WR_Byte(0x40,OLED_CMD);//---set low column address
	OLED_WR_Byte(0xB0,OLED_CMD);//---set high column address

	OLED_WR_Byte(0xC8,OLED_CMD);//-not offset

	OLED_WR_Byte(0x81,OLED_CMD);//设置对比度
	OLED_WR_Byte(0xff,OLED_CMD);

	OLED_WR_Byte(0xa1,OLED_CMD);//段重定向设置

	OLED_WR_Byte(0xa6,OLED_CMD);//
	
	OLED_WR_Byte(0xa8,OLED_CMD);//设置驱动路数
	OLED_WR_Byte(0x1f,OLED_CMD);
	
	OLED_WR_Byte(0xd3,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	
	OLED_WR_Byte(0xd5,OLED_CMD);
	OLED_WR_Byte(0xf0,OLED_CMD);
	
	OLED_WR_Byte(0xd9,OLED_CMD);
	OLED_WR_Byte(0x22,OLED_CMD);
	
	OLED_WR_Byte(0xda,OLED_CMD);
	OLED_WR_Byte(0x02,OLED_CMD);
	
	OLED_WR_Byte(0xdb,OLED_CMD);
	OLED_WR_Byte(0x49,OLED_CMD);
	
	OLED_WR_Byte(0x8d,OLED_CMD);
	OLED_WR_Byte(0x14,OLED_CMD);
	
	OLED_WR_Byte(0xaf,OLED_CMD);

	OLED_Clear();
 	OLED_On();
	 OLED_Clear();
	while(1)
	{
		printf("herer");
		vTaskDelay(1000/portTICK_RATE_MS);
	}
}

void app_main(void)
{
	gpios_init();				//初始化GPIO
	//uart_init();				//初始化通信串口
	i2c_master_init();
	xTaskCreate(led_task, "led_task", 2048, NULL, 20, NULL);
	//xTaskCreate(uart_task, "uart_task", UART_TASK_STACK_SIZE, NULL, 10, NULL);
	xTaskCreate(motor_test_task, "motor_test_task", 2048, NULL, 14, NULL);
	xTaskCreate(tp_example_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
	xTaskCreate(oled_task, "oled_task", 2048, NULL, 15, NULL);
}
