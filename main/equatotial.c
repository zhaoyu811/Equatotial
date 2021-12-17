#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include "bsp.h"
#include "oled.h"
#include "csrc/u8g2.h"

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

uint8_t u8x8_gpio_and_delay_rtthread(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t i;
    switch(msg)
    {
        case U8X8_MSG_DELAY_NANO:               // delay arg_int * 1 nano second  1s = 1000000000ns
            __asm__ volatile("nop");
            break;

        case U8X8_MSG_DELAY_100NANO:            // delay arg_int * 100 nano seconds
            __asm__ volatile("nop");
            break;

        case U8X8_MSG_DELAY_10MICRO:            // delay arg_int * 10 micro seconds 1s = 1000000us
            for (uint16_t n = 0; n < 320; n++)
            {
                __asm__ volatile("nop");
            }
        break;

        case U8X8_MSG_DELAY_MILLI:              // delay arg_int * 1 milli second
            vTaskDelay(arg_int);
            break;

        case U8X8_MSG_GPIO_AND_DELAY_INIT:  
            // Function which implements a delay, arg_int contains the amount of ms  

            // set menu pin mode
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_HOME],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_SELECT],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_PREV],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_NEXT],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_UP],PIN_MODE_INPUT_PULLUP);
            //rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_DOWN],PIN_MODE_INPUT_PULLUP);

            break;

        case U8X8_MSG_DELAY_I2C:
            // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
            // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
            for (uint16_t n = 0; n < (arg_int<=2?160:40); n++)
            {
                __asm__ volatile("nop");
            }
            break;

        //case U8X8_MSG_GPIO_D0:                // D0 or SPI clock pin: Output level in arg_int
        //case U8X8_MSG_GPIO_SPI_CLOCK:

        //case U8X8_MSG_GPIO_D1:                // D1 or SPI data pin: Output level in arg_int
        //case U8X8_MSG_GPIO_SPI_DATA:

        default:
                break;
    }
    return 0;
}
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
uint8_t u8x8_byte_rtthread_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    uint8_t *data;

    uint8_t t = 0;
    i2c_cmd_handle_t cmd = 0;
    int start = 0;
    switch(msg)
    {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            printf("打印数据 %d\n", arg_int);
            for(int i=0; i<arg_int; i++)
            {
                printf("%x ", data[i]);
            }
            printf("\n");
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            for(int i=0; i<arg_int; i++)
            {
                i2c_master_write_byte(cmd, 0x78, ACK_CHECK_EN);
                i2c_master_write_byte(cmd, data[i],ACK_CHECK_EN);
            }
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
            if(ret != ESP_OK)
               printf("write error %d\n", ret);
            i2c_cmd_link_delete(cmd);
            break;

        case U8X8_MSG_BYTE_INIT:
            printf("IIC INIT\n");
            printf("u8x8_GetI2CAddress = %d\n", u8x8_GetI2CAddress(u8x8));
            break;

        case U8X8_MSG_BYTE_SET_DC:
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            printf("U8X8_MSG_BYTE_START_TRANSFER %d \n", arg_int);
            start = 1;
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            printf("U8X8_MSG_BYTE_END_TRANSFER %d\n", arg_int);
            start = 0;
            break;

        default:
            return 0;
    }
    return 1;
}

static void u8g2_ssd1306_12864_hw_i2c_example(void)
{
    u8g2_t u8g2;

    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_rtthread_hw_i2c, u8x8_gpio_and_delay_rtthread);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    /* full buffer example, setup procedure ends in _f */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 1, 18, "U8g2 on RT-Thread");
    u8g2_SendBuffer(&u8g2);

    // Draw Graphics
    //u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
    //u8g2_DrawGlyph(&u8g2, 112, 56, 0x2603 );
    //u8g2_SendBuffer(&u8g2);
}

static void oled_task(void *pvParameter)
{
	vTaskDelay(200/portTICK_RATE_MS);
#if 0
	OLED_Init();	

	OLED_ShowCHinese(0,0,0);//中
	OLED_ShowCHinese(18,0,1);//景
	OLED_ShowCHinese(36,0,2);//园
	OLED_ShowCHinese(54,0,3);//电
	OLED_ShowCHinese(72,0,4);//子
	OLED_ShowCHinese(90,0,5);//科
	OLED_ShowCHinese(108,0,6);//技

	OLED_ShowString(0,0,(unsigned char *)"0.91OLEDTEST",8);
	//OLED_ShowString(0,1,(unsigned char *)"0123456789AB",8);

	//OLED_ShowString(0,0,(unsigned char *)"0.91OLEDTEST",16);

	//OLED_DrawBMP(0,0,128,8,BMP1);
#else
	vTaskDelay(1000/portTICK_RATE_MS);
	//参考AVR
	u8g2_ssd1306_12864_hw_i2c_example();
#endif
	while(1)
	{
		vTaskDelay(5000/portTICK_RATE_MS);
	}
}

void app_main(void)
{
	gpios_init();				//初始化GPIO
	//uart_init();				//初始化通信串口
	i2c_master_init();
	xTaskCreate(led_task, "led_task", 2048, NULL, 20, NULL);
	xTaskCreate(oled_task, "oled_task", 4096, NULL, 15, NULL);
}
