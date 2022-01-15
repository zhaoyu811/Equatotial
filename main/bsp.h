#ifndef BSP_H
#define BSP_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

//led 程序运行指示灯
#define LED     	15

//电源电压采集
#define POWER_ADC_CHANNEL   ADC_CHANNEL_7     //GPIO 35  通道7
extern void print_char_val_type(esp_adc_cal_value_t val_type);
extern void power_input_adc_init(void);

//dec ra 串口配置
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)
#define UART_BAUD_RATE     		(115200)
#define BUF_SIZE                (256)
//RA DEC 串口
#define DEC_UART_PORT_NUM   1
#define DEC_TXD             17
#define DEC_RXD             5
#define RA_UART_PORT_NUM    2
#define RA_TXD              18
#define RA_RXD              19

//初始化函数
extern void dec_tmc2208_uart_init(void);        //初始化dec轴串口
extern void ra_tmc2208_uart_init(void);         //初始化ra轴串口

//触控按键
#define UP_KEY		8	//GPIO26 touchpad8
#define DOWN_KEY	5	//GPIO12 touchpad5
#define LEFT_KEY	9	//GPIO25 touchpad9
#define RIGHT_KEY	7	//GPIO27 touchpad7
#define CONFIRM_KEY	4	//GPIO13 touchpad4
#define CANCEL_KEY	6	//GPIO14 touchpad6
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)
#define TOUCH_THRESH_PERCENT  (80)
extern bool s_pad_activated[9];
extern uint32_t s_pad_init_val[9];
extern void menu_touchpad_init(void);

#endif
