#include "bsp.h"
#include <string.h>

//定时器相关
typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} timer_info_t;

typedef struct {
    timer_info_t info;
    uint64_t timer_counter_value;
} timer_event_t;

static timer_info_t timer_info_[4];


static bool IRAM_ATTR timer00_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    timer_info_t *info = (timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    if (!info->auto_reload) 
	{
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }



    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static bool IRAM_ATTR timer01_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    timer_info_t *info = (timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    if (!info->auto_reload) 
	{
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }
    /* Now just send the event data back to the main program task */
    
    
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static bool IRAM_ATTR timer10_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    timer_info_t *info = (timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    if (!info->auto_reload) 
	{
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */
    

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static bool IRAM_ATTR timer11_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    timer_info_t *info = (timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    if (!info->auto_reload) 
	{
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */


    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static void tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_msec, timer_isr_t isr_handler)
{
    /* Select and initialize basic parameters of the timer */
	timer_deinit(group, timer);
    timer_config_t config = 
	{
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_msec * TIMER_SCALE/1000);
    timer_enable_intr(group, timer);
	
	timer_info_t *timer_info;
	if(group==0&&timer==0)
	{
		timer_info = &timer_info_[0];
	}
	else if(group==0&&timer==1)
	{
		timer_info = &timer_info_[1];
	}
	else if(group==1&&timer==0)
	{
		timer_info = &timer_info_[2];
	}
	else
	{
		timer_info = &timer_info_[3];
	}
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_msec;
    timer_isr_callback_add(group, timer, isr_handler, timer_info, 0);
}


//tg_timer_init(TIMER_GROUP_1, TIMER_0, true, 1, timer10_group_isr_callback);// 1 1 1


void gpios_init(void)
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
	
/*
	ESP_ERROR_CHECK(touch_pad_init());
	touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
	touch_pad_config(RIGHT_KEY, 0);
	touch_pad_config(CANCEL_KEY, 0);
	touch_pad_config(DOWN_KEY, 0);
	touch_pad_config(CONFIRM_KEY, 0);
	touch_pad_filter_start(10);	
    */
}


void uart_init(void)
{
	/* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = 
	{
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD, UART_RXD, UART_RTS, UART_CTS));
}



