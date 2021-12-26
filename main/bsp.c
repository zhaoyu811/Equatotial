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

int dec_position = 0;
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

    dec_position ++;
    if(dec_position % 2 == 0)
        gpio_set_level(DEC_STEP, 1);
    else
        gpio_set_level(DEC_STEP, 0);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}
int ra_position = 0;
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
    
    ra_position++;
    if(ra_position % 2 == 0)
        gpio_set_level(RA_STEP, 1);
    else
        gpio_set_level(RA_STEP, 0);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

static void tg_timer_init(int group, int timer, bool auto_reload, double timer_interval_msec, timer_isr_t isr_handler)
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
    timer_set_alarm_value(group, timer, timer_interval_msec * TIMER_SCALE/1000);    //5000000M/1000 = 5000HZ
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

void ra_dec_timer_init()
{
    tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 269.2625, timer00_group_isr_callback);   //269.2625毫秒
    timer_start(0, 0);
    tg_timer_init(TIMER_GROUP_0, TIMER_1, true, 269.2625, timer01_group_isr_callback);
    timer_start(0, 1);
}

void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void power_input_adc_init(void)
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

void dec_ra_axis_gpios_init(void)
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

void dec_tmc2208_uart_init(void)
{
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
    ESP_ERROR_CHECK(uart_driver_install(DEC_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(DEC_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DEC_UART_PORT_NUM, DEC_TXD, DEC_RXD, UART_RTS, UART_CTS));
}

void ra_tmc2208_uart_init(void)
{
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
    ESP_ERROR_CHECK(uart_driver_install(RA_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(RA_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RA_UART_PORT_NUM, RA_TXD, RA_RXD, UART_RTS, UART_CTS));
}

bool s_pad_activated[9];
uint32_t s_pad_init_val[9];

static void tp_menu_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 4; i <= 9; i++) {
        if ((pad_intr >> i) & 0x01) {
            s_pad_activated[i] = true;
        }
    }
}

void menu_touchpad_init(void)
{
    ESP_ERROR_CHECK(touch_pad_init());
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(UP_KEY, 0);
    touch_pad_config(DOWN_KEY, 0);
    touch_pad_config(LEFT_KEY, 0);
    touch_pad_config(RIGHT_KEY, 0);
    touch_pad_config(CONFIRM_KEY, 0);
    touch_pad_config(CANCEL_KEY, 0);
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);

    uint16_t touch_value;
    for (int i = 4; i <= 9; i++) 
    {
        //read filtered value
        touch_pad_read_filtered(i, &touch_value);
        s_pad_init_val[i] = touch_value;
        printf("test init: touch pad [%d] val is %d\n", i, touch_value);
        //set interrupt threshold.
        ESP_ERROR_CHECK(touch_pad_set_thresh(i, touch_value * 2 / 3));
    }
    touch_pad_isr_register(tp_menu_rtc_intr, NULL);
}


