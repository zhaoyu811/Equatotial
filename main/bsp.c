#include "bsp.h"
#include <string.h>

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


