#include "motor.h"
#include <freertos/portmacro.h>
#include <freertos/projdefs.h>
#include <driver/timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "coordinate.h"
#include <math.h>

//GPIO初始化
void raDecAxisGpiosInit(void)
{
	gpio_reset_pin(RA_EN);
	gpio_reset_pin(RA_STEP);
	gpio_reset_pin(RA_DIR);

	gpio_set_direction(RA_EN, GPIO_MODE_OUTPUT);
	gpio_set_direction(RA_STEP, GPIO_MODE_OUTPUT);
	gpio_set_direction(RA_DIR, GPIO_MODE_OUTPUT);

	gpio_set_level(RA_EN, 0);
	gpio_set_level(RA_STEP, 0);
	gpio_set_level(RA_DIR, 0);
	
	gpio_reset_pin(DEC_EN);
	gpio_reset_pin(DEC_STEP);
	gpio_reset_pin(DEC_DIR);
	
	gpio_set_direction(DEC_EN, GPIO_MODE_OUTPUT);
	gpio_set_direction(DEC_STEP, GPIO_MODE_OUTPUT);
	gpio_set_direction(DEC_DIR, GPIO_MODE_OUTPUT);

	gpio_set_level(DEC_EN, 0);
	gpio_set_level(DEC_STEP, 0);
	gpio_set_level(DEC_DIR, 0);
}

motorRunPara raMotorRunPara, decMotorRunPara;

int raTargetPulse = 0;              //ra  赤经轴目标位置    0~(320000-1)
int decTargetPulse = 0;             //dec 赤纬轴目标位置    0~(320000-1)

int raPulseCount = 0;               //ra  赤经轴的位置  0-(320000-1)
int decPulseCount = 0;              //dec 赤纬轴的位置  0~(320000-1)
static int raInterruptCount = 0;    //ra  赤经轴的定时器中断计数，用于 STEP 引脚翻转电平  0~(640000-1)
static int decInterruptCount = 0;   //dec 赤纬轴的定时器中断计数，用于 STEP 引脚翻转电平  0~(640000-1)
uint64_t counterValue;
//定时器组 0  定时器 0 中断服务函数    定时翻转赤经轴电机STEP引脚电平
static bool IRAM_ATTR timer0_group0_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    //printf("pulse count: %ld\n", timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &counterValue));
    if(raMotorRunPara._motorRotateDir == CW)  //顺时针旋转
        raInterruptCount++;
    else                        //逆时针
        raInterruptCount--;

    if(raInterruptCount<0)      //将区间 限制在 0~(640000-1)
        raInterruptCount += 640000;
    else if(raInterruptCount>=640000)
        raInterruptCount -= 640000;

    if(raInterruptCount % 2 == 0)
        gpio_set_level(RA_STEP, 1);
    else
        gpio_set_level(RA_STEP, 0);

    raPulseCount = (raInterruptCount/2)%320000;      //电机位置 一圈  320000pulse
    
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &counterValue);
    return high_task_awoken == pdTRUE; 
}

//定时器组 0  定时器 1 中断服务函数    定时翻转赤维轴电机STEP引脚电平
static bool IRAM_ATTR timer1_group0_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    if(decMotorRunPara._motorRotateDir == CW)
        decInterruptCount++;
    else
        decInterruptCount--;

    if(decInterruptCount<0)      //将区间 限制在 0~(640000-1) 是否等同于 decInterruptCount/640000 负数取余？？
        decInterruptCount += 640000;
    else if(decInterruptCount>=640000)
        decInterruptCount -= 640000;

    if(decInterruptCount % 2 == 0)
        gpio_set_level(DEC_STEP, 1);
    else
        gpio_set_level(DEC_STEP, 0);

    decPulseCount = (decInterruptCount/2)%320000;    //电机位置 一圈  320000pulse

    //设置下一个脉冲的计数值
    return high_task_awoken == pdTRUE; 
}

//ra 定时器初始化
void raTimerInit(void)
{
    timer_config_t config = 
	{
        .divider = 4,                           //分频系数  4分频   20M   1ms 20000个计数   TIMER_BASE_CLK 定时器基频 80M
        .counter_dir = TIMER_COUNT_UP,          //向上计数
        .counter_en = TIMER_PAUSE,              //停止定时器
        .alarm_en = TIMER_ALARM_EN,             //
        .auto_reload = true,                    //自动重装载
        .intr_type = TIMER_INTR_LEVEL,          //中断模式
    }; 
    timer_init(TIMER_GROUP_0, TIMER_0, &config);          //定时器组0 定时器0
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);   //从 0 开始计数
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);            //使能中断
	
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer0_group0_isr_callback, NULL, 0);
}

//dec 定时器初始化
void decTimerInit(void)
{
    timer_config_t config = 
	{
        .divider = 4,                           //分频系数  4分频   20M   1ms 20000个计数
        .counter_dir = TIMER_COUNT_UP,          //向上计数
        .counter_en = TIMER_PAUSE,              //停止定时器
        .alarm_en = TIMER_ALARM_EN,             //
        .auto_reload = true,                    //自动重装载
        .intr_type = TIMER_INTR_LEVEL,          //中断模式
    }; 
    timer_init(TIMER_GROUP_0, TIMER_1, &config);          //定时器组0 定时器0
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);   //从 0 开始计数
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);            //使能中断
	
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, timer1_group0_isr_callback, NULL, 0);
}

void STEPMOTOR_AxisMoveRel(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{ 
    __IO uint16_t tim_count;
    // 达到最大速度时的步数
    __IO uint32_t max_s_lim;
    // 必须要开始减速的步数（如果加速没有达到最大速度）
    __IO uint32_t accel_lim;

    if(step < 0) // 步数为负数
    {
        srd.dir = CCW; // 逆时针方向旋转
        STEPMOTOR_DIR_REVERSAL();
        step =-step;   // 获取步数绝对值
    }

    else
    {
        srd.dir = CW; // 顺时针方向旋转
        STEPMOTOR_DIR_FORWARD();
    }

    if(step == 1)    // 步数为1
    {
        srd.accel_count = -1;   // 只移动一步
        srd.run_state = DECEL;  // 减速状态.
        srd.step_delay = 1000;   // 短延时      
    }
    else if(step != 0)  // 如果目标运动步数不为0
    {
        // 我们的驱动器用户手册有详细的计算及推导过程
        // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
        // min_delay = (alpha / tt)/ w
        srd.min_delay = (int32_t)(A_T_x10/speed);

        // 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
        // step_delay = 1/tt * sqrt(2*alpha/accel)
        // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100

        srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
        // 计算多少步之后达到最大速度的限制
        // max_s_lim = speed^2 / (2*alpha*accel)
        max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));

        // 如果达到最大速度小于0.5步，我们将四舍五入为0
        // 但实际我们必须移动至少一步才能达到想要的速度
        if(max_s_lim == 0)
        {
            max_s_lim = 1;
        }

        // 计算多少步之后我们必须开始减速
        // n1 = (n1+n2)decel / (accel + decel)
        accel_lim = (uint32_t)(step*decel/(accel+decel));

        // 我们必须加速至少1步才能才能开始减速.
        if(accel_lim == 0)
        {
            accel_lim = 1;
        }

        // 使用限制条件我们可以计算出减速阶段步数
        if(accel_lim <= max_s_lim)
        {
            srd.decel_val = accel_lim - step;
        }
        else
        {
            srd.decel_val = -(max_s_lim*accel/decel);
        }

        // 当只剩下一步我们必须减速
        if(srd.decel_val == 0)
        {
            srd.decel_val = -1;
        }
        // 计算开始减速时的步数
        srd.decel_start = step + srd.decel_val;

        // 如果最大速度很慢，我们就不需要进行加速运动
        if(srd.step_delay <= srd.min_delay)
        {
            srd.step_delay = srd.min_delay;
            srd.run_state = RUN;
        }
        else
        {
            srd.run_state = ACCEL;
        }   
        // 复位加速度计数值
        srd.accel_count = 0;
    }
    MotionStatus = 1; // 电机为运动状态
    tim_count=__HAL_TIM_GET_COUNTER(&htimx_STEPMOTOR);  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,STEPMOTOR_TIM_CHANNEL_x,tim_count+srd.step_delay); // 设置定时器比较值
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, STEPMOTOR_TIM_CHANNEL_x, TIM_CCx_ENABLE);// 使能定时器通道
    STEPMOTOR_OUTPUT_ENABLE();
}

int raMotorTpMove(double minSpeed, double maxSpeed, double acc, double dec, int step)         //ra轴T型位置移动
{
    unsigned short timeCount;
    unsigned int maxSpeedPulse; //达到最大速度时的步速
    unsigned int startDecPulse; //开始减速的步速

    if(step < 0)    //步数为负数
    {
        raMotorRunPara._motorRotateDir = CCW;
        gpio_set_level(RA_DIR, 0);
        step = -step;
    }
    else
    {
        raMotorRunPara._motorRotateDir = CW;
        gpio_set_level(RA_DIR, 1);
    }

    if(step == 1)   //如果只有一步，直接减速
    {
        raMotorRunPara._motorState = DecState;
    }
    else if(step != 0)
    {
        //设置最大速度极限，计算的到定时器的计数器的值  4分频 频率为  20M 20000000
        raMotorRunPara.maxSpeedTimerAlarmValue = (int)(A_T_x10/maxSpeed);
        raMotorRunPara.startSpeedTimerAlarmValue = (int)((T_FREQ_148*sqrt(A_SQ/acc))/10);
        //计算多少步之后达到最大速度的限制
        maxSpeedPulse = (unsigned int)(maxSpeed*maxSpeed/(A_x200*acc/10));
        if(maxSpeed==0)
            maxSpeed = 1;
        //计算多少步之后必须开始减速
        startDecPulse = (unsigned int)(step*dec/(acc+dec));
        if(startDecPulse==0)
            startDecPulse = 1;

        if(startDecPulse <= maxSpeedPulse)
            raMotorRunPara.startDecPulse = maxSpeedPulse - step;
        else
            raMotorRunPara.startDecPulse = -(maxSpeedPulse*acc/dec);

        //当只剩下一步必须件数
        if(raMotorRunPara.startDecPulse == 0)
        {
            raMotorRunPara.startDecPulse = -1;
        }
        //计算开始件数时的不熟
        raMotorRunPara.startDecPulse = step + raMotorRunPara.startDecPulse;

        //如果最大速度很慢，我们就不需要进行加速运动
        if(raMotorRunPara.startDecPulse <= raMotorRunPara.maxSpeedTimerAlarmValue)
        {
            raMotorRunPara.startDecPulse = raMotorRunPara.maxSpeedTimerAlarmValue;
            raMotorRunPara._motorState = KeepState;
        }
        else 
        {
            raMotorRunPara._motorState = AccState;
        }
    }
    uint64_t counterValue;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &counterValue);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, counterValue+raMotorRunPara.startSpeedTimerAlarmValue);
    timer_start(TIMER_GROUP_0, TIMER_GROUP_1);
    return 0;
}

int decMotorTpMove(double minVel, double maxVel, double tAcc, double tDec, int dist)        //dec轴T型位置移动
{
    return 0;
}

int raMotorTvMove(double minVel, double maxVel, double tAcc, motorRotateDir dir)    //ra轴T型速度运动
{
    return 0;
}

int decMotorTvMove(double minVel, double maxVel, double tAcc, motorRotateDir dir)   //dec轴T型速度运动
{
    return 0;
}

void raMotorStop()     //ra轴停止运动
{

}

void decMotorStop()    //dec轴停止运动
{

}

void startSyncTarget(void)
{
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 2692625);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void stopSyncTarget(void)
{
    timer_pause(TIMER_GROUP_0, TIMER_0);
}

//  RA/Dec:hh:mm:ss.s SDD*MM'SS.S  RA/Dec坐标系
//  Az/Alt:DDD*MM'SS.S SDD*MM'SS.S  Az/Alt坐标系
//  AT/Dec:hh:mm:ss.s SDD*MM'SS.S   时角坐标系

double getCurrentHtSecsValue(void)
{
    //指向北天极后
    //raPulseCount=0 对应 0h  raPulseCount = 80000 对应 6h
    //raPulseCount=160000 对应 12h raPulseCount = 240000对应 18h
    double htSecs = raPulseCount * (24.0*3600.0/320000.0);   //得出时角的秒数

    return htSecs;
}

void getCurrentHtValueString(char *str)
{
    int hour, min; 
    float sec;

    double htSecs = getCurrentHtSecsValue();

    hour = (int)htSecs/3600;       //得出小时
    min = (int)htSecs%3600/60;     //得出分钟
    sec = htSecs-(hour*3600)-(min*60);     //得出秒

    sprintf(str, "%02d:%02d:%02.0f", hour, min, sec);
}

double getCurrentDecAsecsValue(void)
{
    double decAsecs = decPulseCount * (360.0*3600.0/320000); //转换成角度秒
    return decAsecs;
}

void getCurrentDecValueString(char *str)
{
    //指向北天极后
    //decPulseCount=0        对应 90°        decPulseCount=80000 对应 0度
    //decPulseCount=160000   对应 -90度      decPulseCount=24000 对应 0度
    
    int degree;
    int amin;
    float asec;
    
    double decAsecs = getCurrentDecAsecsValue();

    if(decAsecs<=(180*3600.0))
    {
        //建立坐标关系  
        // y = ax + b;   90° = 0°a + b; -90° = 180°a + b;  a = -1, b = 90°
        // y = -x + 90°；
        decAsecs = -decAsecs + 90.0*3600.0;
        if(decAsecs>0)
        {
            degree = (int)decAsecs/3600;
            amin = (int)decAsecs%3600/60;
            asec = decAsecs-(degree*3600)-(amin*60);
            sprintf(str, "%02d*%02d'%02.0f", degree, amin, asec);
        }
        else
        {
            decAsecs = -decAsecs;
            degree = (int)decAsecs/3600;
            amin = (int)decAsecs%3600/60;
            asec = decAsecs-(degree*3600)-(amin*60);
            sprintf(str, "-%02d*%02d'%02.0f", degree, amin, asec);
        }
    }
    else
    {
        //建立坐标关系  
        // y = ax + b;   -90° = 180°a + b; 90° = 360°a + b;  a = 1, b = -270°
        // y = x + -270°；
        decAsecs = decAsecs - 270.0*3600.0;
        if(decAsecs>0)
        {
            degree = (int)decAsecs/3600;
            amin = (int)decAsecs%3600/60;
            asec = decAsecs-(degree*3600)-(amin*60);
            sprintf(str, "%02d*%02d'%02.0f", degree, amin, asec);
        }
        else
        {
            decAsecs = -decAsecs;
            degree = (int)decAsecs/3600;
            amin = (int)decAsecs%3600/60;
            asec = decAsecs-(degree*3600)-(amin*60);
            sprintf(str, "-%02d*%02d'%02.0f", degree, amin, asec);
        }
    }
}

//当前的时间 位置  时间角度 换算出当前的赤经值
void getCurrentRaValueString(char *str)         //得到当前的赤经字符串
{
    double raSecs = htSec2raSec(getCurrentHtSecsValue());
    raSec2RaStr(raSecs, str);
}
