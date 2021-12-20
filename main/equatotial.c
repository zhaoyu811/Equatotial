#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "bsp.h"
#include "oled.h"
#include "csrc/u8g2.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <sys/socket.h>


u8g2_t u8g2;

static void led_task(void *arg)
{
	gpio_reset_pin(LED);
	gpio_set_direction(LED, GPIO_MODE_OUTPUT);

	while(1)
	{
		gpio_set_level(LED, 0);
		vTaskDelay(500/portTICK_RATE_MS);
		gpio_set_level(LED, 1);
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

static void u8g2_ssd1306_12864_hw_i2c_init(void)
{
    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_esp32_hw_i2c, u8x8_gpio_and_delay_esp32);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    /* full buffer example, setup procedure ends in _f */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    u8g2_DrawStr(&u8g2, 1, 8, "U8g2 on RT-Thread");
    u8g2_DrawStr(&u8g2, 1, 20, "hello world你好啊!");
    u8g2_SendBuffer(&u8g2);

    // Draw Graphics
    //u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
    //u8g2_DrawGlyph(&u8g2, 112, 56, 0x2603 );
    //u8g2_SendBuffer(&u8g2);
}

static void oled_task(void *pvParameter)
{
	vTaskDelay(200/portTICK_RATE_MS);
    u8g2_ssd1306_12864_hw_i2c_init();
	while(1)
	{
		vTaskDelay(5000/portTICK_RATE_MS);
	}
}

static void dec_uart_task(void *arg)
{
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	uint8_t dat[][4] = {
		{0x05, 0x00, 0x00, 0x48}, 
		{0x05, 0x00, 0x01, 0xC1}, 
		{0x05, 0x00, 0x02, 0x8F}, 
		{0x05, 0x00, 0x05, 0x21}, 
		{0x05, 0x00, 0x06, 0x6F}, 
		{0x05, 0x00, 0x07, 0xE6}, 
		{0x05, 0x00, 0x12, 0xB7}, 
		{0x05, 0x00, 0x6A, 0xED}, 
		{0x05, 0x00, 0x6B, 0x64}, 
		{0x05, 0x00, 0x6C, 0xCA}, 
		{0x05, 0x00, 0x6F, 0x84}, 
		{0x05, 0x00, 0x70, 0x62}, 
		{0x05, 0x00, 0x71, 0xEB}, 
		{0x05, 0x00, 0x72, 0xA5}
	};

	vTaskDelay(10000/portTICK_RATE_MS);

	for(int i=0; i<14; i++)
	{
		uart_write_bytes(DEC_UART_PORT_NUM, (const char *)dat[i], 4);
		int len = uart_read_bytes(DEC_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
		for(int i=0; i<len; i++)
		{
			printf("0x%2x ", data[i]);
		}
		printf("\n");
	}

	while(1)
	{
		//int len = uart_read_bytes(DEC_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        //uart_write_bytes(DEC_UART_PORT_NUM, (const char *) data, len);
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

static void ra_uart_task(void *arg)
{
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	uint8_t dat[][4] = {
		{0x05, 0x00, 0x00, 0x48}, 
		{0x05, 0x00, 0x01, 0xC1}, 
		{0x05, 0x00, 0x02, 0x8F}, 
		{0x05, 0x00, 0x05, 0x21}, 
		{0x05, 0x00, 0x06, 0x6F}, 
		{0x05, 0x00, 0x07, 0xE6}, 
		{0x05, 0x00, 0x12, 0xB7}, 
		{0x05, 0x00, 0x6A, 0xED}, 
		{0x05, 0x00, 0x6B, 0x64}, 
		{0x05, 0x00, 0x6C, 0xCA}, 
		{0x05, 0x00, 0x6F, 0x84}, 
		{0x05, 0x00, 0x70, 0x62}, 
		{0x05, 0x00, 0x71, 0xEB}, 
		{0x05, 0x00, 0x72, 0xA5}
	};

	for(int i=0; i<14; i++)
	{
		uart_write_bytes(RA_UART_PORT_NUM, (const char *)dat[i], 4);
		int len = uart_read_bytes(RA_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
		for(int i=0; i<len; i++)
		{
			printf("0x%2x ", data[i]);
		}
		printf("\n");
	}

	while(1)
	{
		//int len = uart_read_bytes(RA_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        //uart_write_bytes(RA_UART_PORT_NUM, (const char *) data, len);

		vTaskDelay(100/portTICK_RATE_MS);
	}
}

#define NO_OF_SAMPLES   64       //Multisampling
#define DEFAULT_VREF    0        //Use adc2_vref_to_gpio() to obtain a better estimate  
static void power_input_adc_task(void *arg)
{
	static esp_adc_cal_characteristics_t *adc_chars;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(POWER_ADC_CHANNEL, ADC_ATTEN_DB_0);
	//Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

	vTaskDelay(2000/portTICK_RATE_MS);
	while (1) 
	{	
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) 
		{
            adc_reading += adc1_get_raw(POWER_ADC_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
		char raw_string[20];
		char vol_string[20];
        sprintf(raw_string, "Raw: %d", adc_reading);
		sprintf(vol_string, "Vol: %dmV", voltage);

		//显示
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
		u8g2_DrawStr(&u8g2, 1, 10, raw_string);
		u8g2_DrawStr(&u8g2, 1, 20, vol_string);
		u8g2_SendBuffer(&u8g2);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void menu_touchpad_task(void *arg)
{
	touch_pad_intr_enable();
	char touchpad_string[24];
    while (1) 
	{
		for (int i = 4; i <= 9; i++) 
		{
			if (s_pad_activated[i] == true) 
			{
				sprintf(touchpad_string, "T%d activated!", i);
				// Wait a while for the pad being released
				u8g2_DrawStr(&u8g2, 3, 30, touchpad_string);
				vTaskDelay(200 / portTICK_PERIOD_MS);
				// Clear information on pad activation
				s_pad_activated[i] = false;
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static const char *TAG = "scan";
esp_err_t create_tcp_server()
{
	ESP_LOGI(TAG, "server socket....,port=%d\n", 6000);
	int server_socket = socket(AF_INET, SOCK_STREAM, 0);

	if (server_socket < 0)
	{
		printf("create_server failed:%d\n", server_socket);
		return ESP_FAIL;
	}
	struct sockaddr_in server_addr;
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(6000);//指定的端口号
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);


	//开始创建
	if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
	{
		printf("bind_server failed：%d\n", server_socket);
		close(server_socket);
		return ESP_FAIL;
	}

    //监听指定的端口
    if (listen(server_socket, 5) < 0)
    {
        printf("listen_server:%d\n", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }

	struct sockaddr_in client_addr;
	socklen_t socklen;
    int connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
    //判断是否连接成功
    if (connect_socket < 0)
    {
        printf("accept_server:%d\n", connect_socket);
        close(server_socket);
        return ESP_FAIL;
    }

    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
    return ESP_OK;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		create_tcp_server();
    }
}

/* Initialize Wi-Fi as sta and set scan method */
static void fast_scan(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Initialize and start WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "mi10",
            .password = "1234567890",
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold.rssi = -127,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
	dec_ra_axis_gpios_init();	//初始化dec ra轴的控制gpio
	dec_tmc2208_uart_init();	//初始化dec uart通信
	ra_tmc2208_uart_init();		//初始化ra	uart通信
	i2c_master_init();			//初始化oled i2c通信
	power_input_adc_init();		//初始化电源采集adc
	menu_touchpad_init();		//初始化按键
	xTaskCreate(led_task, "led_task", 2048, NULL, 20, NULL);
	xTaskCreate(oled_task, "oled_task", 4096, NULL, 15, NULL);
	xTaskCreate(dec_uart_task, "dec_uart_task", 2048, NULL, 1, NULL);
	xTaskCreate(ra_uart_task, "ra_uart_task", 2048, NULL, 2, NULL);
	xTaskCreate(power_input_adc_task, "power_input_adc_task", 2048, NULL, 25, NULL);
	xTaskCreate(menu_touchpad_task, "menu_touchpad_task", 2048, NULL, 3, NULL);


	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	fast_scan();
}
