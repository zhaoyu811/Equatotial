#ifndef __MOTOR_H
#define __MOTOR_H

#include "driver/timer.h"
#include "driver/gpio.h"
#include <math.h>

#define PI (acos(-1.0))

//引脚配置
#define RA_EN		      23
#define RA_STEP	      22
#define RA_DIR		      21
#define DEC_EN		      2
#define DEC_STEP	      4
#define DEC_DIR	      16

// 定义定时器预分频，定时器实际时钟频率为：MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER               4  // 步进电机驱动器细分设置为：  32  细分
// 定义定时器周期，输出比较模式周期设置为xFFFF
#define CW                                    0 // 顺时针
#define CCW                                   1 // 逆时针
#define STOP                                  0 // 加减速曲线状态：停止
#define ACCEL                                 1 // 加减速曲线状态：加速阶段
#define DECEL                                 2 // 加减速曲线状态：减速阶段
#define RUN                                   3 // 加减速曲线状态：匀速阶段
#define T1_FREQ                               (80000000/(STEPMOTOR_TIM_PRESCALER)) // 频率ft值
#define FSPR                                  200         //步进电机单圈步数
#define MICRO_STEP                            16          // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数
// 数学常数
#define ALPHA                                 ((float)(2*3.14159/SPR))       // α= 2*pi/spr
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10)) // 0.676为误差修正值
#define A_SQ                                  ((float)(2*100000*ALPHA))
#define A_x200                                ((float)(200*ALPHA))

typedef struct {
   volatile unsigned char    run_state;              // 电机旋转状态
   volatile unsigned char    dir;                    // 电机旋转方向
   volatile int              step_delay;             // 下个脉冲周期（时间间隔），启动时为加速度
   volatile unsigned int     decel_start;            // 启动减速位置
   volatile int              decel_val;              // 减速阶段步数
   volatile int              min_delay;              // 最小脉冲周期(最大速度，即匀速段速度)
   volatile int              accel_count;            // 加减速阶段计数值
   volatile unsigned int     step_count;             // 总的运行步数计数
}speedRampData;

void raDecAxisGpiosInit(void);                  //电机引脚初始化
void raTimerInit(void);                         //ra 定时器初始化
void decTimerInit(void);                        //dec 定时器初始化

void raMotorTRPMove(unsigned int accel, unsigned int decel, double speed, int step);
void raMotorTRVMove(unsigned int accel, double speed, unsigned char dir);   //速度模式运动
void raMotorStopMove(unsigned int decel);               //减速停止
int getRaMotorState(void);
int getRaMotorPosition(void);

void decMotorTRPMove(unsigned int accel, unsigned int decel, double speed, int step);
void decMotorTRVMove(unsigned int accel, double speed, unsigned char dir);   //速度模式运动
void decMotorStopMove(unsigned int decel);               //减速停止
int getDecMotorState(void);
int getDecMotorPosition(void);

void startSyncTarget();     //跟踪目标
void stopSyncTarget();      //停止跟踪目标

int getCurrentRaPulseValue(void);
int getCurrentDecPulseValue(void);
void getCurrentHaValueString(char *str);         //得到当前的时角字符串
void getCurrentRaValueString(char *str);         //得到当前的赤经字符串
void getCurrentDecValueString(char *str);        //得到当前的赤经字符串
double getCurrentDecAsecsValue(void);
double getCurrentHaSecsValue(void);
char * getCurrentAzStr(void);
char * getCurrentAltStr(void);

#endif
