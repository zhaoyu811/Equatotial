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

volatile speedRampData raSRD, decSRD;

static volatile int raPulseCount = 0;        //ra  赤经轴的位置  0-(320000-1)
static volatile int decPulseCount = 0;       //dec 赤纬轴的位置  0~(320000-1)
static volatile int raInterruptCount = 0;    //ra  赤经轴的定时器中断计数，用于 STEP 引脚翻转电平  0~(640000-1)
static volatile int decInterruptCount = 0;   //dec 赤纬轴的定时器中断计数，用于 STEP 引脚翻转电平  0~(640000-1)
//定时器组 0  定时器 0 中断服务函数    定时翻转赤经轴电机STEP引脚电平

static void raTimerRun(void)
{
    //保存新(下)一个延时周期
    volatile int new_step_delay = 0;
    static int rest = 0;

    raSRD.step_count++;
    //判断电机现在所处状态
    if(raSRD.run_state == ACCEL)
    {
        raSRD.accel_count++;// 加速计数值加1
        ////计算新(下)一步脉冲周期(时间间隔)
        new_step_delay = raSRD.step_delay - (((2 *raSRD.step_delay) + rest)/(4 * raSRD.accel_count + 1));
        // 计算余数，下次计算补上余数，减少误差
        rest = ((2 * raSRD.step_delay)+rest)%(4 * raSRD.accel_count + 1);
        
        if(raSRD.step_count >= raSRD.decel_start)  //大于必须减速的步数
        {
            raSRD.step_delay = new_step_delay;
            rest = 0;
            raSRD.run_state = DECEL;
        }
        else
        {
            if(new_step_delay < raSRD.min_delay)   //判断是否
            {
                raSRD.step_delay = raSRD.min_delay;
                raSRD.run_state = RUN;
            }
            else
            {
                raSRD.step_delay = new_step_delay;
                raSRD.run_state = ACCEL;
            }
        }
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, raSRD.step_delay);
    }
    else if(raSRD.run_state == RUN)
    {
        //判断是否到了该减速的位置
        if(raSRD.step_count>=raSRD.decel_start) 
            raSRD.run_state = DECEL;
    }
    else if(raSRD.run_state == DECEL)
    {
        raSRD.decel_val++;
        //判断是否到了该停止的时候
        if(raSRD.decel_val==0)
        {
            timer_pause(TIMER_GROUP_0, TIMER_0);
            raSRD.run_state = STOP;
            rest = 0;
        }
        else
        {
            new_step_delay = raSRD.step_delay - (((2 *raSRD.step_delay) + rest)/(4 * raSRD.decel_val + 1));
            rest = ((2 * raSRD.step_delay)+rest)%(4 * raSRD.decel_val + 1);
            raSRD.step_delay = new_step_delay;
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, raSRD.step_delay);
        }
    }
}

static bool IRAM_ATTR timer0_group0_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    if(raSRD.dir == CW)  //顺时针旋转
        raInterruptCount++;
    else                        //逆时针
        raInterruptCount--;

    if(raInterruptCount<0)      //将区间 限制在 0~(640000-1)
        raInterruptCount += 640000;
    else if(raInterruptCount>=640000)
        raInterruptCount -= 640000;

    if(raInterruptCount % 2 == 1)
    {
        gpio_set_level(RA_STEP, 1);
    }
    else
    {
        gpio_set_level(RA_STEP, 0);
        raTimerRun();
    }
    raPulseCount = (raInterruptCount/2);      //电机位置 一圈  320000pulse 
    return high_task_awoken == pdTRUE; 
}

static void decTimerRun(void)
{
    //保存新(下)一个延时周期
    volatile int new_step_delay = 0;
    static volatile int rest = 0;

    decSRD.step_count++;
    //判断电机现在所处状态
    if(decSRD.run_state == ACCEL)
    {
        decSRD.accel_count++;// 加速计数值加1
        ////计算新(下)一步脉冲周期(时间间隔)
        new_step_delay = decSRD.step_delay - (((2 *decSRD.step_delay) + rest)/(4 * decSRD.accel_count + 1));
        // 计算余数，下次计算补上余数，减少误差
        rest = ((2 * decSRD.step_delay)+rest)%(4 * decSRD.accel_count + 1);
        
        if(decSRD.step_count >= decSRD.decel_start)  //大于必须减速的步数
        {
            decSRD.step_delay = new_step_delay;
            rest = 0;
            decSRD.run_state = DECEL;
        }
        else
        {
            if(new_step_delay < decSRD.min_delay)   //判断是否
            {
                decSRD.step_delay = decSRD.min_delay;
                decSRD.run_state = RUN;
            }
            else
            {
                decSRD.step_delay = new_step_delay;
                decSRD.run_state = ACCEL;
            }
        }
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, decSRD.step_delay);
    }
    else if(decSRD.run_state == RUN)
    {
        //判断是否到了该减速的位置
        if(decSRD.step_count>=decSRD.decel_start) 
            decSRD.run_state = DECEL;         
    }
    else if(decSRD.run_state == DECEL)
    {
        decSRD.decel_val++;
        //判断是否到了该停止的时候
        if(decSRD.decel_val==0)
        {
            timer_pause(TIMER_GROUP_0, TIMER_1);
            decSRD.run_state = STOP;
            rest = 0;
            return;
        }
        else
        {
            new_step_delay = decSRD.step_delay - (((2 *decSRD.step_delay) + rest)/(4 * decSRD.decel_val + 1));
            rest = ((2 * decSRD.step_delay)+rest)%(4 * decSRD.decel_val + 1);
            decSRD.step_delay = new_step_delay;
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, decSRD.step_delay);
        }
    }
}

//定时器组 0  定时器 1 中断服务函数    定时翻转赤维轴电机STEP引脚电平
static bool IRAM_ATTR timer1_group0_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    if(decSRD.dir == CW)  //顺时针旋转
        decInterruptCount++;
    else                        //逆时针
        decInterruptCount--;

    if(decInterruptCount<0)      //将区间 限制在 0~(640000-1)
        decInterruptCount += 640000;
    else if(decInterruptCount>=640000)
        decInterruptCount -= 640000;

    if(decInterruptCount % 2 == 1)
    {
        gpio_set_level(DEC_STEP, 1);
    }
    else
    {
        gpio_set_level(DEC_STEP, 0);
        decTimerRun();
    }
    decPulseCount = (decInterruptCount/2);      //电机位置 一圈  320000pulse 
    return high_task_awoken == pdTRUE; 
}

//ra 定时器初始化
void raTimerInit(void)
{
    timer_config_t config = 
	{
        .divider = STEPMOTOR_TIM_PRESCALER,                           //分频系数  4分频   20M   1ms 20000个计数   TIMER_BASE_CLK 定时器基频 80M
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
        .divider = STEPMOTOR_TIM_PRESCALER,                           //分频系数  4分频   20M   1ms 20000个计数
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

//int  step= -200;
//unsigned int accel=15000;//rad/s
//unsigned int decel=15000;//rad/s
//double speed = 125.6636; //0.1rad/s
//ra轴 梯形加减速相对位置运动
void raMotorTRPMove(unsigned int accel, unsigned int decel, double speed, int step)  //相对运动
{
    //达到最大速度时的步数
    unsigned int max_s_lim;
    //必须要开始减速的步数 如果加速没有达到最大速度
    unsigned int accel_lim;
    if(step<0)
    {
        raSRD.dir = CCW;
        gpio_set_level(RA_DIR, CCW);  //设置方向为逆时针
        step = -step;
    }
    else if(step>0)
    {
        raSRD.dir = CW;
        gpio_set_level(RA_DIR, CW);   //设置方向为顺时针
    }
    else
    {
        return;     
    }
    raSRD.step_count = 0;
    if(step == 1)
    {
        raSRD.accel_count = -1;   //只移动一步
        raSRD.run_state = DECEL;  //减速状态
        raSRD.step_delay = 10000;  //短延时
    }
    else    //如果目标运动步数大于1
    {
        //最小延时，最大速度的延时，x10
        raSRD.min_delay = (unsigned int)(A_T_x10/speed);
        //通过计算第一个(c0)的步进延时来设定家属都，其中accel的单位为0.1rad/sec^2
        raSRD.step_delay = (unsigned int)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
        //计算多少步子厚达到最大速度的限制
        max_s_lim = (unsigned int)(speed*speed/(A_x200*accel/10));
        //如果达到最大速度小于0.5步，我们将四舍五入为0
        //但实际我们必须移动至少一步才能达到想要的速度
        if(max_s_lim == 0)
        {
            max_s_lim = 1;
        }
        //使用限制条件我们可以计算出减速阶段步数
        accel_lim = (unsigned int)(step*decel/(accel+decel));
        //我们必须加速自少一步才能开始减速
        if(accel_lim == 0)
        {
            accel_lim = 1;
        }
        //使用限制条件我们可以计算出减速阶段步数
        if(accel_lim <= max_s_lim)
        {
            raSRD.decel_val = accel_lim - step;
        }
        else
        {
            raSRD.decel_val = -(max_s_lim*accel/decel);
        }
        //当只剩下一步我们必须减速
        if(raSRD.decel_val == 0)
        {
            raSRD.decel_val = -1;
        }
        //计算开始减速是的步数
        raSRD.decel_start = step + raSRD.decel_val;
        //如果最大速度很慢，我们就不需要进行加速运动
        if(raSRD.step_delay <= raSRD.min_delay)
        {
            raSRD.step_delay = raSRD.min_delay;
            raSRD.run_state = RUN;
        }
        else
        {
            raSRD.run_state = ACCEL;
        }
        //复位加速度技术值
        raSRD.accel_count = 0;
    }
    //设置起始计数值
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);     //先清空计数值
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, raSRD.step_delay);
    //开始运动
    timer_start(TIMER_GROUP_0, TIMER_0);
    //输出信息
    printf("电机旋转状态%d\n",raSRD.run_state);
    printf("电机旋转方向%d\n", raSRD.dir);
    printf("下一个脉冲时间间隔%d\n", raSRD.step_delay);
    printf("启动减速位置%d\n", raSRD.decel_start);
    printf("减速阶段步数%d\n", raSRD.decel_val);
    printf("最小脉冲周期%d\n", raSRD.min_delay);
    printf("加速阶段计数值%d\n", raSRD.accel_count);
}

void raMotorTRVMove(unsigned int accel, double speed, unsigned char dir)   //速度模式运动
{
    //达到最大速度时的步数
    unsigned int max_s_lim;

    raSRD.dir = dir;
    gpio_set_level(RA_DIR, dir);   //设置方向为顺时针
    //最小延时，最大速度的延时，x10
    raSRD.min_delay = (unsigned int)(A_T_x10/speed);
    //通过计算第一个(c0)的步进延时来设定家属都，其中accel的单位为0.1rad/sec^2
    raSRD.step_delay = (unsigned int)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
    //计算多少步子厚达到最大速度的限制
    max_s_lim = (unsigned int)(speed*speed/(A_x200*accel/10));
    //如果达到最大速度小于0.5步，我们将四舍五入为0
    //但实际我们必须移动至少一步才能达到想要的速度
    if(max_s_lim == 0)
    {
        max_s_lim = 1;
    }
    //如果最大速度很慢，我们就不需要进行加速运动
    if(raSRD.step_delay <= raSRD.min_delay)
    {
        raSRD.step_delay = raSRD.min_delay;
        raSRD.run_state = RUN;
    }
    else
    {
        raSRD.run_state = ACCEL;
    }
    //复位加速度技术值
    raSRD.accel_count = 0;
    raSRD.decel_start = 0xFFFFFFFF;
    //设置起始计数值
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);     //先清空计数值
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, raSRD.step_delay);
    //开始运动
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void raMotorStopMove(unsigned int decel)                //减速停止
{
    if(raSRD.run_state == DECEL)
        return;

    //必须要开始减速的步数 如果加速没有达到最大速度
    unsigned int accel_lim;
    //使用限制条件我们可以计算出减速阶段步数
    int step = raSRD.accel_count;
    accel_lim = (unsigned int)(step*decel/(decel+decel));
    //我们必须加速自少一步才能开始减速
    if(accel_lim == 0)
    {
        accel_lim = 1;
    }
    //使用限制条件我们可以计算出减速阶段步数
    if(accel_lim <= step)
    {
        raSRD.decel_val = accel_lim - step;
    }
    else
    {
        raSRD.decel_val = -(step*decel/decel);
    }
    //当只剩下一步我们必须减速
    if(raSRD.decel_val == 0)
    {
        raSRD.decel_val = -1;
    }
    //计算开始减速是的步数
    raSRD.decel_start = step + raSRD.decel_val;
    raSRD.decel_start = 0x0;
}

int getRaMotorState(void)
{
    return raSRD.run_state;
}

int getRaMotorPosition(void)
{
    return raPulseCount;
}

//dec轴 梯形加减速相对位置运动
void decMotorTRPMove(unsigned int accel, unsigned int decel, double speed, int step)  //相对运动
{
    //达到最大速度时的步数
    unsigned int max_s_lim;
    //必须要开始减速的步数 如果加速没有达到最大速度
    unsigned int accel_lim;
    if(step<0)
    {
        decSRD.dir = CCW;
        gpio_set_level(DEC_DIR, CCW);  //设置方向为逆时针
        step = -step;
    }
    else if(step>0)
    {
        decSRD.dir = CW;
        gpio_set_level(DEC_DIR, CW);   //设置方向为顺时针
    }
    else
    {
        return;     
    }
    decSRD.step_count = 0;
    if(step == 1)
    {
        decSRD.accel_count = -1;   //只移动一步
        decSRD.run_state = DECEL;  //减速状态
        decSRD.step_delay = 10000;  //短延时
    }
    else    //如果目标运动步数大于1
    {
        //最小延时，最大速度的延时，x10
        decSRD.min_delay = (unsigned int)(A_T_x10/speed);
        //通过计算第一个(c0)的步进延时来设定家属都，其中accel的单位为0.1rad/sec^2
        decSRD.step_delay = (unsigned int)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
        //计算多少步子厚达到最大速度的限制
        max_s_lim = (unsigned int)(speed*speed/(A_x200*accel/10));
        //如果达到最大速度小于0.5步，我们将四舍五入为0
        //但实际我们必须移动至少一步才能达到想要的速度
        if(max_s_lim == 0)
        {
            max_s_lim = 1;
        }
        //使用限制条件我们可以计算出减速阶段步数
        accel_lim = (unsigned int)(step*decel/(accel+decel));
        //我们必须加速自少一步才能开始减速
        if(accel_lim == 0)
        {
            accel_lim = 1;
        }
        //使用限制条件我们可以计算出减速阶段步数
        if(accel_lim <= max_s_lim)
        {
            decSRD.decel_val = accel_lim - step;
        }
        else
        {
            decSRD.decel_val = -(max_s_lim*accel/decel);
        }
        //当只剩下一步我们必须减速
        if(decSRD.decel_val == 0)
        {
            decSRD.decel_val = -1;
        }
        //计算开始减速是的步数
        decSRD.decel_start = step + decSRD.decel_val;
        //如果最大速度很慢，我们就不需要进行加速运动
        if(decSRD.step_delay <= decSRD.min_delay)
        {
            decSRD.step_delay = decSRD.min_delay;
            decSRD.run_state = RUN;
        }
        else
        {
            decSRD.run_state = ACCEL;
        }
        //复位加速度技术值
        decSRD.accel_count = 0;
    }
    //设置起始计数值
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);     //先清空计数值
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, decSRD.step_delay);
    //开始运动
    timer_start(TIMER_GROUP_0, TIMER_1);
    //输出信息
    printf("电机旋转状态%d\n",decSRD.run_state);
    printf("电机旋转方向%d\n", decSRD.dir);
    printf("下一个脉冲时间间隔%d\n", decSRD.step_delay);
    printf("启动减速位置%d\n", decSRD.decel_start);
    printf("减速阶段步数%d\n", decSRD.decel_val);
    printf("最小脉冲周期%d\n", decSRD.min_delay);
    printf("加速阶段计数值%d\n", decSRD.accel_count);
}

void decMotorTRVMove(unsigned int accel, double speed, unsigned char dir)   //速度模式运动
{
    //达到最大速度时的步数
    unsigned int max_s_lim;

    decSRD.dir = dir;
    gpio_set_level(DEC_DIR, dir);   //设置方向为顺时针
    //最小延时，最大速度的延时，x10
    decSRD.min_delay = (unsigned int)(A_T_x10/speed);
    //通过计算第一个(c0)的步进延时来设定家属都，其中accel的单位为0.1rad/sec^2
    decSRD.step_delay = (unsigned int)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
    //计算多少步子厚达到最大速度的限制
    max_s_lim = (unsigned int)(speed*speed/(A_x200*accel/10));
    //如果达到最大速度小于0.5步，我们将四舍五入为0
    //但实际我们必须移动至少一步才能达到想要的速度
    if(max_s_lim == 0)
    {
        max_s_lim = 1;
    }
    //如果最大速度很慢，我们就不需要进行加速运动
    if(decSRD.step_delay <= decSRD.min_delay)
    {
        decSRD.step_delay = decSRD.min_delay;
        decSRD.run_state = RUN;
    }
    else
    {
        decSRD.run_state = ACCEL;
    }
    //复位加速度技术值
    decSRD.accel_count = 0;
    decSRD.decel_start = 0xFFFFFFFF;
    //设置起始计数值
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);     //先清空计数值
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, decSRD.step_delay);
    //开始运动
    timer_start(TIMER_GROUP_0, TIMER_1);
}

void decMotorStopMove(unsigned int decel)                //减速停止
{
    if(decSRD.run_state == DECEL)
        return;

    //必须要开始减速的步数 如果加速没有达到最大速度
    unsigned int accel_lim;
    //使用限制条件我们可以计算出减速阶段步数
    int step = decSRD.accel_count;
    accel_lim = (unsigned int)(step*decel/(decel+decel));
    //我们必须加速自少一步才能开始减速
    if(accel_lim == 0)
    {
        accel_lim = 1;
    }
    //使用限制条件我们可以计算出减速阶段步数
    if(accel_lim <= step)
    {
        decSRD.decel_val = accel_lim - step;
    }
    else
    {
        decSRD.decel_val = -(step*decel/decel);
    }
    //当只剩下一步我们必须减速
    if(decSRD.decel_val == 0)
    {
        decSRD.decel_val = -1;
    }
    //计算开始减速是的步数
    decSRD.decel_start = step + decSRD.decel_val;
    decSRD.decel_start = 0x0;
}

int getDecMotorState(void)
{
    return decSRD.run_state;
}

int getDecMotorPosition(void)
{
    return decPulseCount;
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

double getCurrentHaSecsValue(void)
{
    //指向北天极后
    //raPulseCount=0 对应 0h  raPulseCount = 80000 对应 6h
    //raPulseCount=160000 对应 12h raPulseCount = 240000对应 18h
    double haSecs = raPulseCount * (24.0*3600.0/320000.0);   //得出时角的秒数
    if(decPulseCount >= 160000)
        haSecs += 12*3600;
    return haSecs;
}

double getCurrentHaRadValue(void)   //得到当前时角 弧度值
{
    double haRad = (raPulseCount/320000.0)*2*PI;
    if(decPulseCount >= 160000)
        haRad += PI;
    return haRad;
}

void getCurrentHaValueString(char *str)
{
    int hour, min; 
    float sec;

    double haSecs = getCurrentHaSecsValue();

    hour = (int)haSecs/3600%24;       //得出小时
    min = (int)haSecs%3600/60;     //得出分钟
    sec = haSecs-(hour*3600)-(min*60);     //得出秒

    sprintf(str, "%02d:%02d:%02.0f", hour, min, sec);
}

double getCurrentDecAsecsValue(void)
{
    double decAsecs = decPulseCount * (360.0*3600.0/320000.0); //转换成角度秒
    return decAsecs;
}

double getCurrentDecRadValue(void)
{
    double decAsecs = getCurrentDecAsecsValue();
    double decRad;
    if(decAsecs<=(180*3600.0))
    {
        decRad =(-(decAsecs/3600.0) + 90)/180*PI;
    }
    else
    {
        decRad =((decAsecs/3600.0) - 270)/180*PI;
    }
    return decRad;
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
            sprintf(str, "%02d*%02d'%02.0f", degree, amin, fabs(asec));
        }
        else
        {
            decAsecs = -decAsecs;
            degree = (int)decAsecs/3600;
            amin = (int)decAsecs%3600/60;
            asec = decAsecs-(degree*3600)-(amin*60);
            sprintf(str, "-%02d*%02d'%02.0f", degree, amin, fabs(asec));
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
            sprintf(str, "%02d*%02d'%02.0f", degree, amin, fabs(asec));
        }
        else
        {
            decAsecs = -decAsecs;
            degree = (int)decAsecs/3600;
            amin = (int)decAsecs%3600/60;
            asec = decAsecs-(degree*3600)-(amin*60);
            sprintf(str, "-%02d*%02d'%02.0f", degree, amin, fabs(asec));
        }
    }
}

//当前点机位置 换算出当前的赤经值
void getCurrentRaValueString(char *str)         //得到当前的赤经字符串
{
    double raSecs = haSec2raSec(getCurrentHaSecsValue());
    raSec2RaStr(raSecs, str);
}

//得到当前的方位角   azimuth angle
double getCurrentAzRadValue(void)
{
    double sina, cosa;
    double decRad = getCurrentDecRadValue();
    double haRad = getCurrentHaRadValue();
    double latiRad = getCurrentSiteLatitudeRadValue();

    #if 1
    sina = -cos(decRad) * sin(haRad);
    cosa = cos(latiRad) * sin(decRad) - cos(decRad) * sin(latiRad) * cos(haRad);

    double azRad;
    azRad = atan2(sina, cosa);
    if(azRad < 0)
        azRad += 2*PI;
    #elif
    double x, y, z;
    x = cos(haRad) * cos(decRad);
    y = sin(haRad) * cos(decRad);
    z = sin(decRad);
    double xhor, yhor, zhor;
    xhor = x*sin(latiRad) - z*cos(latiRad);
    yhor = y;
    zhor = x*cos(latiRad) + z*sin(latiRad);

    double azRad = atan2(yhor, xhor) + PI;
    #endif
    return azRad;
}

//得到当前的高度角  Altitude angle
double getCurrentAltRadValue(void)
{
    double latiRad = getCurrentSiteLatitudeRadValue();
    double decRad = getCurrentDecRadValue();
    double haRad = getCurrentHaRadValue();

    #if 1
    double altRad = asin(sin(latiRad)*sin(decRad)+cos(latiRad)*cos(decRad)*cos(haRad));
    #elif
    double x, y, z;
    x = cos(haRad) * cos(decRad);
    y = sin(haRad) * cos(decRad);
    z = sin(decRad);
    double xhor, yhor, zhor;
    xhor = x*sin(latiRad) - z*cos(latiRad);
    yhor = y;
    zhor = x*cos(latiRad) + z*sin(latiRad);
    double altRad = asin(zhor);
    #endif

    return altRad;
}

char currentAzString[32];
char currentAltString[32];
char * getCurrentAzStr(void)
{
    double azRad = getCurrentAzRadValue();
    int degree, amin;
    double asec;
    double azAsec = azRad/(2*PI)*360.0*3600.0;  //转换为角秒

    degree = (int)azAsec/3600;
    amin = (int)azAsec%3600/60;
    asec = azAsec-(degree*3600)-(amin*60);
    sprintf(currentAzString, "%03d*%02d'%02.0f", degree, amin, fabs(asec));
    
    return currentAzString;
}

char * getCurrentAltStr(void)
{
    double altRad = getCurrentAltRadValue();
    int degree, amin;
    double asec;
    double altAsec = altRad/(2*PI)*360.0*3600.0;  //转换为角秒

    if(altAsec < 0)
    {
        altAsec = -altAsec;
        degree = (int)altAsec/3600;
        amin = (int)altAsec%3600/60;
        asec = altAsec-(degree*3600)-(amin*60);
        sprintf(currentAltString, "-%02d*%02d'%02.0f", degree, amin, fabs(asec));
    }
    else
    {
        degree = (int)altAsec/3600;
        amin = (int)altAsec%3600/60;
        asec = altAsec-(degree*3600)-(amin*60);
        sprintf(currentAltString, "%02d*%02d'%02.0f", degree, amin, fabs(asec));
    }
    return currentAltString;
}

