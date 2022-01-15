#include "motor.h"
#include <freertos/portmacro.h>
#include <freertos/projdefs.h>
#include "coordinate.h"

int ra_position = 0;        //ra  赤经轴的位置  0-(320000-1)
int dec_position = 0;       //dec 赤纬轴的位置  0~(320000-1)

static int ra_count = 0;    //ra  赤经轴的定时器中断计数，用于 STEP 引脚翻转电平
static int dec_count = 0;   //dec 赤纬轴的定时器中断计数，用于 STEP 引脚翻转电平

//GPIO初始化
void ra_dec_axis_gpios_init(void)
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

//定时器组 0  定时器 0 中断服务函数    定时翻转赤经轴电机STEP引脚电平
static bool IRAM_ATTR timer0_group0_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    ra_count ++;
    if(ra_count % 2 == 0)
        gpio_set_level(RA_STEP, 1);
    else
        gpio_set_level(RA_STEP, 0);

    ra_position = (ra_count/2)%320000;      //电机位置 一圈  320000pulse

    return high_task_awoken == pdTRUE; 
}

//定时器组 0  定时器 1 中断服务函数    定时翻转赤维轴电机STEP引脚电平
static bool IRAM_ATTR timer1_group0_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    dec_count++;
    if(dec_count % 2 == 0)
        gpio_set_level(DEC_STEP, 1);
    else
        gpio_set_level(DEC_STEP, 0);

    dec_position = (dec_count/2)%320000;    //电机位置 一圈  320000pulse

    return high_task_awoken == pdTRUE; 
}

//ra 定时器初始化
void ra_timer_init(void)
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
//ra 中断计数值
void ra_timer_set_alarm_value(uint64_t value)
{
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, value);
}

//dec 定时器初始化
void dec_timer_init(void)
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
//dec 中断计数值
void dec_timer_set_alarm_value(uint64_t value)
{
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, value);
}

//  RA/Dec:hh:mm:ss.s SDD*MM'SS.S  RA/Dec坐标系
//  Az/Alt:DDD*MM'SS.S SDD*MM'SS.S  Az/Alt坐标系
//  AT/Dec:hh:mm:ss.s SDD*MM'SS.S   时角坐标系

double getCurrentHtSecsValue(void)
{
    //指向北天极后
    //ra_position=0 对应 0h  ra_position = 80000 对应 6h
    //ra_position=160000 对应 12h ra_position = 240000对应 18h
    double htSecs = ra_position * (24.0*3600.0/320000.0);   //得出时角的秒数

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
    double decAsecs = dec_position * (360.0*3600.0/320000); //转换成角度秒
    return decAsecs;
}

void getCurrentDecValueString(char *str)
{
    //指向北天极后
    //dec_position=0        对应 90°        dec_position=80000 对应 0度
    //dec_position=160000   对应 -90度      dec_position=24000 对应 0度
    
    int degree;
    int amin;
    float asec;

    double decAsecs = getCurrentDecAsecsValue();

    if(ra_position<=80000)      
    {
        //建立坐标关系  
        // y = ax + b;   90° = 0°a + b; 0° = 90°a + b;  a = -1, b = 90°
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
    else if(ra_position<=160000)
    {
        //建立坐标关系  
        // y = ax + b;   0° = 90°a + b; -90° = 180°a + b;  a = -1, b = 90°
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
    else if(ra_position<=240000)
    {
        //建立坐标关系  
        // y = ax + b;   -90° = 180°a + b; 0° = 270°a + b;  a = 1, b = -270°
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
    else
    {
        //建立坐标关系  
        // y = ax + b;   0° = 270°a + b; 90° = 360°a + b;  a = 1, b = -270°
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
