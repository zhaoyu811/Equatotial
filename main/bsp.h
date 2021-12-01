#ifndef BSP_H
#define BSP_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/timer.h"

//串口配置
#define UART_TXD (CONFIG_UART_TXD)
#define UART_RXD (CONFIG_UART_RXD)
#define UART_RTS (UART_PIN_NO_CHANGE)
#define UART_CTS (UART_PIN_NO_CHANGE)

#define UART_PORT_NUM      		(CONFIG_UART_PORT_NUM)
#define UART_BAUD_RATE     		(CONFIG_UART_BAUD_RATE)
#define UART_TASK_STACK_SIZE    (CONFIG_TASK_STACK_SIZE)

#define BUF_SIZE (3000)

//引脚配置

#define LED     	15
#define RA_EN		23
#define RA_STEP	22
#define RA_DIR		21
#define DEC_EN		2
#define DEC_STEP	4
#define DEC_DIR	16

//定时器配置
#define TIMER_DIVIDER         (16)  							// Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds


//初始化函数
extern void gpios_init(void);						//初始化GPIO
extern void uart_init(void);						//初始化串口


#endif
