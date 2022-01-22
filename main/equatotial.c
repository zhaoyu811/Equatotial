#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "bsp.h"
#include "oled.h"
#include "csrc/u8g2.h"
#include "motor.h"
#include "coordinate.h"
#include "wifi.h"

u8g2_t u8g2;

static void ledTask(void *arg)
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

static void u8g2_ssd1306_12864_hw_i2c_init(void)
{
    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_esp32_hw_i2c, u8x8_gpio_and_delay_esp32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    /* full buffer example, setup procedure ends in _f */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 1, 8, "U8g2 on RT-Thread");
    u8g2_DrawStr(&u8g2, 1, 20, "hello world你好啊!");
    u8g2_SendBuffer(&u8g2);
}

char atPositionString[30];
char decPositionString[30];
char raPostiontString[30];
char raPulsePosition[30];

static void oledTask(void *pvParameter)
{
	vTaskDelay(200/portTICK_RATE_MS);
    u8g2_ssd1306_12864_hw_i2c_init();
	while(1)
	{
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
		u8g2_DrawStr(&u8g2, 1, 10, atPositionString);
		u8g2_DrawStr(&u8g2, 1, 20, decPositionString);
		u8g2_DrawStr(&u8g2, 1, 30, raPostiontString);
		//u8g2_DrawStr(&u8g2, 1, 40, raPulsePosition);
		u8g2_DrawStr(&u8g2, 1, 40, getCurrentAzStr());
		u8g2_DrawStr(&u8g2, 1, 50, getCurrentAltStr());
		u8g2_DrawStr(&u8g2, 1, 60, ipString);
		u8g2_SendBuffer(&u8g2);
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

static void raDecUpdatePositon(void *arg)
{
	while(1)
	{
		char haString[20], decString[20], raString[20];
		getCurrentHaValueString(haString);
		getCurrentDecValueString(decString);
		getCurrentRaValueString(raString);
		sprintf(atPositionString, "Ha:%s", haString);
		sprintf(decPositionString, "Dec:%s", decString);
		sprintf(raPostiontString, "Ra:%s", raString);
		sprintf(raPulsePosition, "pulse:%7d", getRaMotorPosition());
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

static void motorTestTask(void *arg)
{
	while(1)
	{
		//decMotorTRPMove(15000, 15000, 125.6636, 60);
		//raMotorTRPMove(15000, 15000, 125.6636, 60);
		while(getRaMotorState()!=STOP)
		{
			vTaskDelay(100/portTICK_PERIOD_MS);
		}
		vTaskDelay(5000/portTICK_PERIOD_MS);

		//decMotorTRPMove(15000, 15000, 125.6636, -50);
		//raMotorTRPMove(15000, 15000, 125.6636, -50);
		while(getRaMotorState()!=STOP)
		{
			vTaskDelay(100/portTICK_PERIOD_MS);
		}
		vTaskDelay(5000/portTICK_PERIOD_MS);
	}
}

void app_main(void)
{
	raDecAxisGpiosInit();	//初始化dec ra轴的控制gpio
	raTimerInit();			//初始化r轴的定时器
	decTimerInit();			//初始化dec轴的定时器
	i2cMasterInit();		//初始化oled i2c通信

	xTaskCreate(ledTask, "ledTask", 2048, NULL, 20, NULL);
	xTaskCreate(oledTask, "oledTask", 4096, NULL, 15, NULL);
	xTaskCreate(raDecUpdatePositon, "raDecUpdatePositon", 2048, NULL, 10, NULL);
	xTaskCreate(motorTestTask, "motorTestTask", 20480*2, NULL, 21, NULL);

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	fastScan();
}
