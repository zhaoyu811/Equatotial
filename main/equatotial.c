#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include "bsp.h"
#include "oled.h"
#include "csrc/u8g2.h"


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

uint8_t u8x8_gpio_and_delay_rtthread(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t i;
    switch(msg)
    {
        case U8X8_MSG_DELAY_NANO:               // delay arg_int * 1 nano second
            __asm__ volatile("nop");
            break;

        case U8X8_MSG_DELAY_100NANO:            // delay arg_int * 100 nano seconds
            __asm__ volatile("nop");
            break;

        case U8X8_MSG_DELAY_10MICRO:            // delay arg_int * 10 micro seconds
            for (uint16_t n = 0; n < 320; n++)
            {
                __asm__ volatile("nop");
            }
        break;

        case U8X8_MSG_DELAY_MILLI:              // delay arg_int * 1 milli second
            rt_thread_mdelay(arg_int);
            break;

        case U8X8_MSG_GPIO_AND_DELAY_INIT:  
            // Function which implements a delay, arg_int contains the amount of ms  

            // set spi pin mode 
            rt_pin_mode(u8x8->pins[U8X8_PIN_SPI_CLOCK],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_SPI_DATA],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_RESET],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_DC],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_CS],PIN_MODE_OUTPUT);

            // set i2c pin mode
            rt_pin_mode(u8x8->pins[U8X8_PIN_I2C_DATA],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_I2C_CLOCK],PIN_MODE_OUTPUT);

            // set 8080 pin mode
            rt_pin_mode(u8x8->pins[U8X8_PIN_D0],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_D1],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_D2],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_D3],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_D4],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_D5],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_D6],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_D7],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_E],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_DC],PIN_MODE_OUTPUT);
            rt_pin_mode(u8x8->pins[U8X8_PIN_RESET],PIN_MODE_OUTPUT);

            // set menu pin mode
            rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_HOME],PIN_MODE_INPUT_PULLUP);
            rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_SELECT],PIN_MODE_INPUT_PULLUP);
            rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_PREV],PIN_MODE_INPUT_PULLUP);
            rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_NEXT],PIN_MODE_INPUT_PULLUP);
            rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_UP],PIN_MODE_INPUT_PULLUP);
            rt_pin_mode(u8x8->pins[U8X8_PIN_MENU_DOWN],PIN_MODE_INPUT_PULLUP);

            // set value
            rt_pin_write(u8x8->pins[U8X8_PIN_SPI_CLOCK],1);
            rt_pin_write(u8x8->pins[U8X8_PIN_SPI_DATA],1);
            rt_pin_write(u8x8->pins[U8X8_PIN_RESET],1);
            rt_pin_write(u8x8->pins[U8X8_PIN_DC],1);
            rt_pin_write(u8x8->pins[U8X8_PIN_CS],1);
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

        case U8X8_MSG_GPIO_D2:                  // D2 pin: Output level in arg_int
            rt_pin_write(u8x8->pins[U8X8_PIN_D2],arg_int);
            break;

        case U8X8_MSG_GPIO_D3:                  // D3 pin: Output level in arg_int
            rt_pin_write(u8x8->pins[U8X8_PIN_D3], arg_int);
            break;

        case U8X8_MSG_GPIO_D4:                  // D4 pin: Output level in arg_int
            rt_pin_write(u8x8->pins[U8X8_PIN_D4], arg_int);
            break;

        case U8X8_MSG_GPIO_D5:                  // D5 pin: Output level in arg_int
            rt_pin_write(u8x8->pins[U8X8_PIN_D5], arg_int);
            break;

        case U8X8_MSG_GPIO_D6:                  // D6 pin: Output level in arg_int
            rt_pin_write(u8x8->pins[U8X8_PIN_D6], arg_int);
            break;

        case U8X8_MSG_GPIO_D7:                  // D7 pin: Output level in arg_int
            rt_pin_write(u8x8->pins[U8X8_PIN_D7], arg_int);
            break;

        case U8X8_MSG_GPIO_E:                   // E/WR pin: Output level in arg_int
            rt_pin_write(u8x8->pins[U8X8_PIN_E], arg_int);
            break;

        case U8X8_MSG_GPIO_I2C_CLOCK:
            // arg_int=0: Output low at I2C clock pin
            // arg_int=1: Input dir with pullup high for I2C clock pin
            rt_pin_write(u8x8->pins[U8X8_PIN_I2C_CLOCK], arg_int);
            break;

        case U8X8_MSG_GPIO_I2C_DATA:
            // arg_int=0: Output low at I2C data pin
            // arg_int=1: Input dir with pullup high for I2C data pin
            rt_pin_write(u8x8->pins[U8X8_PIN_I2C_DATA], arg_int);
            break;

        case U8X8_MSG_GPIO_SPI_CLOCK:  
            // Function to define the logic level of the clockline  
            rt_pin_write(u8x8->pins[U8X8_PIN_SPI_CLOCK], arg_int);
            break;

        case U8X8_MSG_GPIO_SPI_DATA:
            // Function to define the logic level of the data line to the display  
            rt_pin_write(u8x8->pins[U8X8_PIN_SPI_DATA], arg_int);
            break;

        case U8X8_MSG_GPIO_CS:
            // Function to define the logic level of the CS line  
            rt_pin_write(u8x8->pins[U8X8_PIN_CS], arg_int);
            break;

        case U8X8_MSG_GPIO_DC:
            // Function to define the logic level of the Data/ Command line  
            rt_pin_write(u8x8->pins[U8X8_PIN_DC], arg_int);
            break;

        case U8X8_MSG_GPIO_RESET:
            // Function to define the logic level of the RESET line
            rt_pin_write(u8x8->pins[U8X8_PIN_RESET], arg_int);
            break;

        default:
            if ( msg >= U8X8_MSG_GPIO(0) )
            {
                i = u8x8_GetPinValue(u8x8, msg);
                if ( i != U8X8_PIN_NONE )
                {
                    if ( u8x8_GetPinIndex(u8x8, msg) < U8X8_PIN_OUTPUT_CNT )
                    {
                        rt_pin_write(i, arg_int);
                    }
                    else
                    {
                        if ( u8x8_GetPinIndex(u8x8, msg) == U8X8_PIN_OUTPUT_CNT )
                        {
                            // call yield() for the first pin only, u8x8 will always request all the pins, so this should be ok
                            // yield();
                        }
                        u8x8_SetGPIOResult(u8x8, rt_pin_read(i) == 0 ? 0 : 1);
                    }
                }
                break;
            }
            return 0;
    }
    return 1;
}

uint8_t u8x8_byte_rtthread_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    struct rt_i2c_msg msgs;
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    uint8_t *data;

    rt_uint8_t t = 0;
    switch(msg)
    {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while( arg_int > 0 )
            {
                buffer[buf_idx++] = *data;
                data++;
                arg_int--;
            }
            break;

        case U8X8_MSG_BYTE_INIT:
            i2c_bus = rt_i2c_bus_device_find(U8G2_I2C_DEVICE_NAME);
            if (i2c_bus == RT_NULL)
            {
                rt_kprintf("[u8g2] Failed to find bus %s\n", U8G2_I2C_DEVICE_NAME);
                return 0;
            }
            break;

        case U8X8_MSG_BYTE_SET_DC:
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            if (i2c_bus == RT_NULL)
            {
                rt_kprintf("[u8g2] Failed to find bus %s\n", U8G2_I2C_DEVICE_NAME);
                return 0;
            }
            // I2C Data Transfer
            msgs.addr  = u8x8_GetI2CAddress(u8x8)>>1;
            msgs.flags = RT_I2C_WR;
            msgs.buf   = buffer;
            msgs.len   = buf_idx;
            while(rt_i2c_transfer(i2c_bus, &msgs, 1) != 1 && t < MAX_RETRY)
            {
                t++;
            };
            if(t >= MAX_RETRY)
            {
                return 0;
            }
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
    u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
    u8g2_DrawGlyph(&u8g2, 112, 56, 0x2603 );
    u8g2_SendBuffer(&u8g2);
}

static void oled_task(void *pvParameter)
{
	vTaskDelay(200/portTICK_RATE_MS);
	//OLED_Init();	

	//OLED_ShowCHinese(0,0,0);//中
	//OLED_ShowCHinese(18,0,1);//景
	//OLED_ShowCHinese(36,0,2);//园
	//OLED_ShowCHinese(54,0,3);//电
	//OLED_ShowCHinese(72,0,4);//子
	//OLED_ShowCHinese(90,0,5);//科
	//OLED_ShowCHinese(108,0,6);//技

	//OLED_ShowString(0,0,(unsigned char *)"0.91OLEDTEST",8);
	//OLED_ShowString(0,1,(unsigned char *)"0123456789AB",8);

	//OLED_ShowString(0,0,(unsigned char *)"0.91OLEDTEST",16);

	//OLED_DrawBMP(0,0,128,8,BMP1);

	vTaskDelay(1000/portTICK_RATE_MS);

	//参考AVR
	u8g2_ssd1306_12864_hw_i2c_example();

	while(1)
	{
		vTaskDelay(5000/portTICK_RATE_MS);
	}
}

void app_main(void)
{
	gpios_init();				//初始化GPIO
	//uart_init();				//初始化通信串口
	//i2c_master_init();
	xTaskCreate(led_task, "led_task", 2048, NULL, 20, NULL);
	xTaskCreate(motor_test_task, "motor_test_task", 2048, NULL, 14, NULL);
	xTaskCreate(tp_example_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
	xTaskCreate(oled_task, "oled_task", 2048, NULL, 15, NULL);
}
