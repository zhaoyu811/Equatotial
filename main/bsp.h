#ifndef BSP_H
#define BSP_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/touch_pad.h"

//串口配置
#define UART_TXD (1)
#define UART_RXD (1)
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM      		(1)
#define UART_BAUD_RATE     		(9600)
#define UART_TASK_STACK_SIZE    (2048)

#define BUF_SIZE (3000)

//引脚配置

#define LED     	15
#define RA_EN		23
#define RA_STEP	    22
#define RA_DIR		21
#define DEC_EN		2
#define DEC_STEP	4
#define DEC_DIR	    16

#define UP_KEY		8	//GPIO26
#define DOWN_KEY	5	//GPIO12 touchpad5
#define LEFT_KEY	9	//GPIO25
#define RIGHT_KEY	7	//GPIO27 touchpad7
#define CONFIRM_KEY	4	//GPIO13 touchpad4
#define CANCEL_KEY	6	//GPIO14 touchpad6

//定时器配置
#define TIMER_DIVIDER         (16)  							// Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds


//初始化函数
extern void gpios_init(void);						//初始化GPIO
extern void uart_init(void);						//初始化串口


#endif
