#ifndef __MOTOR_H
#define __MOTOR_H

#include "driver/timer.h"
#include "driver/gpio.h"
extern uint64_t counterValue;
//引脚配置
#define RA_EN		23
#define RA_STEP	    22
#define RA_DIR		21
#define DEC_EN		2
#define DEC_STEP	4
#define DEC_DIR	    16

typedef enum {
    IdleState = 0x0,    //停止状态
    AccState  = 0x1,    //加速状态
    KeepState = 0x2,    //匀速状态
    DecState  = 0x3,    //减速状态
} motorState;

typedef enum {
    CCW = 0x0,  //逆时针旋转
    CW = 0x1,   //顺时针旋转
} motorRotateDir;

typedef struct {
    motorState _motorState;         //电机状态
    motorRotateDir _motorRotateDir; //电机方向
    int startSpeedTimerAlarmValue;  //启动速度  定时器中断比较值
    int startDecPulse;              //电机开始减速脉冲
    int decPulseCount;              //减速阶段脉冲数
    int maxSpeedTimerAlarmValue;    //最大速度  定时器中断比较值
    int accPulseCount;              //加速阶段脉冲计数
}motorRunPara;

#define TIMER_FREQ  20000000    //频率ft值
#define FSPR        200         //步进电机单圈步数
#define MICRO_STEP  16          //步进电机驱动器细分数
#define SPR         (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数
// 数学常数
#define ALPHA       ((float)(2*3.14159/SPR))      // α= 2*pi/spr//
#define A_T_x10     ((float)(10*ALPHA*TIMER_FREQ))
#define T_FREQ_148  ((float)((TIMER_FREQ*0.676)/10)) // 0.676为误差修正值
#define A_SQ        ((float)(2*100000*ALPHA))
#define A_x200      ((float)(200*ALPHA))

//电机位置
extern int raPulseCount;        //ra  赤经轴的位置  0-(320000-1)
extern int decPulseCount;       //dec 赤纬轴的位置  0~(320000-1)

void raDecAxisGpiosInit(void);                  //电机引脚初始化
void raTimerInit(void);                           //ra 定时器初始化
void decTimerInit(void);                          //dec 定时器初始化

int raMotorTpMove(double minVel, double maxVel, double tAcc, double tDec, int dist);         //ra轴T型位置移动
int decMotorTpMove(double minVel, double maxVel, double tAcc, double tDec, int dist);        //dec轴T型位置移动
int raMotorTvMove(double minVel, double maxVel, double tAcc, motorRotateDir dir);    //ra轴T型速度运动
int decMotorTvMove(double minVel, double maxVel, double tAcc, motorRotateDir dir);   //dec轴T型速度运动
void raMotorStop();    //ra轴停止运动
void decMotorStop();    //dec轴停止运动
void startSyncTarget();  //跟踪目标
void stopSyncTarget();   //停止跟踪目标

void getCurrentHtValueString(char *str);         //得到当前的时角字符串
void getCurrentRaValueString(char *str);         //得到当前的赤经字符串
void getCurrentDecValueString(char *str);        //得到当前的赤经字符串


#endif
