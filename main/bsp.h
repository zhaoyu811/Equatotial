#ifndef BSP_H
#define BSP_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/touch_pad.h"
#include "driver/i2c.h"

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

#define UP_KEY			//GPIO26
#define DOWN_KEY	5	//GPIO12 touchpad5
#define LEFT_KEY		//GPIO25
#define RIGHT_KEY	7	//GPIO27 touchpad7
#define CONFIRM_KEY	4	//GPIO13 touchpad4
#define CANCEL_KEY	6	//GPIO14 touchpad6

//定时器配置
#define TIMER_DIVIDER         (16)  							// Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds


//初始化函数
extern void gpios_init(void);						//初始化GPIO
extern void uart_init(void);						//初始化串口
extern esp_err_t i2c_master_init(void);
extern void Write_IIC_Data(unsigned char IIC_Data);
extern void OLED_WR_Byte(unsigned dat,unsigned cmd);

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#endif
