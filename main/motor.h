#ifndef __MOTOR_H
#define __MOTOR_H

#include "driver/timer.h"
#include "driver/gpio.h"

//引脚配置
#define RA_EN		23
#define RA_STEP	    22
#define RA_DIR		21
#define DEC_EN		2
#define DEC_STEP	4
#define DEC_DIR	    16

//电机位置
extern int ra_position;        //ra  赤经轴的位置  0-(320000-1)
extern int dec_position;       //dec 赤纬轴的位置  0~(320000-1)

void ra_dec_axis_gpios_init(void);                  //电机引脚初始化
void ra_timer_init(void);                           //ra 定时器初始化
void ra_timer_set_alarm_value(uint64_t value);      //ra 定时器中断值设置
void dec_timer_init(void);                          //dec 定时器初始化
void dec_timer_set_alarm_value(uint64_t value);     //dec 定时器中断值设置

void getCurrentHtValueString(char *str);         //得到当前的时角字符串
void getCurrentRaValueString(char *str);         //得到当前的赤经字符串
void getCurrentDecValueString(char *str);        //得到当前的赤经字符串


#endif
