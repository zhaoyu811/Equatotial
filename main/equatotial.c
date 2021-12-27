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
char vol_string[40];
char touchpad_string[24];
char databuff[200];
char ra_position_string[30];
char dec_position_string[30];
char ip_string[30];
static void oled_task(void *pvParameter)
{
	vTaskDelay(200/portTICK_RATE_MS);
    u8g2_ssd1306_12864_hw_i2c_init();
	while(1)
	{
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
		u8g2_DrawStr(&u8g2, 1, 10, vol_string);
		u8g2_DrawStr(&u8g2, 1, 20, touchpad_string);
		u8g2_DrawStr(&u8g2, 1, 30, databuff);
		u8g2_DrawStr(&u8g2, 1, 40, ra_position_string);		//22:04:45#
		u8g2_DrawStr(&u8g2, 1, 50, dec_position_string);	//09*11'55#	
		u8g2_DrawStr(&u8g2, 1, 60, ip_string);
		u8g2_SendBuffer(&u8g2);
		//vTaskDelay(50/portTICK_RATE_MS);
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
#if 0
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
#endif
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
#if 0
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
#endif
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
		sprintf(vol_string, "Vol:%4dmV,Raw:%4d", voltage, adc_reading);

		//显示
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static void menu_touchpad_task(void *arg)
{
	touch_pad_intr_enable();
    while (1) 
	{
		for (int i = 4; i <= 9; i++) 
		{
			if (s_pad_activated[i] == true) 
			{
				sprintf(touchpad_string, "T%d activated!", i);
				// Wait a while for the pad being released
				vTaskDelay(100 / portTICK_PERIOD_MS);
				// Clear information on pad activation
				s_pad_activated[i] = false;
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
static const char *TAG = "scan";
int connect_socket = 0;
static void server_recv_data(void *pvParameters)
{
	int len = 0;	//长度 char databuff[1024]; //缓存 while (1)
	char timestr[20];
	strcpy(timestr, "22:16:20#");
	while(1)
	{
		//清空缓存
		//读取接收数据
		len = recv(connect_socket, databuff, sizeof(databuff), 0); 
		if (len > 0)
		{
			databuff[len] = '\0';
			//ESP_LOGI(TAG, "recvData: %s", databuff);
			
			if(databuff[0]==0x06)
			{
				send(connect_socket, "P", 1, 0);
			}
			else if(strcmp(databuff, ":GVD#")==0)
			{
				send(connect_socket, "dec 26 2021#", strlen("dec 26 2021#"), 0);
			}
			else if(strcmp(databuff, ":GVP#")==0)
			{
				send(connect_socket, "my telescope#", strlen("my telescope#"), 0);
			}
			else if(strcmp(databuff, ":GVN#")==0)
			{
				send(connect_socket, "00.0#", strlen("00.0#"), 0);
			}
			else if(strcmp(databuff, ":GVT#")==0)
			{
				send(connect_socket, "12:12:12#", strlen("12:12:12#"), 0);
			}
			else if(strcmp(databuff, ":Gg#")==0)
			{
				send(connect_socket, "+247*28#", strlen("+247*28#"), 0);
			}
			else if(strcmp(databuff,":D#")==0)
			{
				send(connect_socket, "i dont't known#", strlen("i dont't known#"), 0);
			}
			else if(strcmp(databuff, ":GR#")==0)
			{
				send(connect_socket, ra_position_string, strlen(ra_position_string), 0);
			}
			else if(strcmp(databuff, ":GD#")==0)
			{
				send(connect_socket, dec_position_string, strlen(dec_position_string), 0);
			}
			else if(strcmp(databuff, ":GW#")==0)
			{
				send(connect_socket, "GT1#", strlen("GT1#"), 0);
			}
			else if(strncmp(databuff, ":SG", 3)==0)
			{
				send(connect_socket, "1", strlen("1"), 0);
			}
			else if(strncmp(databuff, ":St", 3)==0)
			{
				send(connect_socket, "1", strlen("1"), 0);
			}
			else if(strncmp(databuff, ":Sg", 3)==0)
			{
				send(connect_socket, "1", strlen("1"), 0);
			}
			else if(strncmp(databuff, ":SL", 3)==0)
			{
				strcpy(timestr, databuff+3);
				send(connect_socket, "1", strlen("1"), 0);
			}
			else if(strcmp(databuff, ":GC#")==0)
			{
				send(connect_socket, "12/17/21#", strlen("12/17/21#"), 0);
			}
			else if(strncmp(databuff, ":SC", 3)==0)
			{
				send(connect_socket, "1Updating Planetary Data#", strlen("1Updating Planetary Data#"), 0);
			}
			else if(strcmp(databuff, ":Gt#")==0)
			{
				send(connect_socket, "+37*50#", strlen("+37*50#"), 0);
			}
			else if(strcmp(databuff, ":GL#")==0)
			{
				printf("%s\n", timestr);
				timestr[7]=timestr[7]+2;
				timestr[0] = '1';
				timestr[1] = '4';
				send(connect_socket, timestr, strlen(timestr), 0);
			}
			else if(strcmp(databuff, ":GG#")==0)
			{
				send(connect_socket, "-08.0#", strlen("-08.0#"), 0);
			}
			else if(strncmp(databuff, ":Sr", 3)==0)
			{
				send(connect_socket, "1", strlen("1"), 0);
			}
			else if(strncmp(databuff, ":Sd", 3)==0)
			{
				send(connect_socket, "1", strlen("1"), 0);
			}
			else if(strcmp(databuff, ":MS#")==0)
			{
				send(connect_socket, "0", strlen("0"), 0);
			}
			else if(strcmp(databuff, ":CM#")==0)
			{
				send(connect_socket, "sync world!#", strlen("sync world!#"), 0);
			}
			else
			{
				ESP_LOGI(TAG, "recvData: %s", databuff);
			}
		}
		vTaskDelay(50/portTICK_RATE_MS);
	}
}

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
    connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
    //判断是否连接成功
    if (connect_socket < 0)
    {
        printf("accept_server:%d\n", connect_socket);
        close(server_socket);
        return ESP_FAIL;
    }

    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
	xTaskCreate(server_recv_data, "server_recv_data", 2048, NULL, 8, NULL);
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
		sprintf(ip_string, IPSTR, IP2STR(&event->ip_info.ip));
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

static void ra_dec_update_position(void *arg)
{
	double ra_arcsec = 0;
	double dec_arcsec = 0;
	while(1)
	{
		//ra格式  22:04:45# 赤精时间
		//dec格式 09*11'55#

		//1个脉冲对应 4.05角秒 解析出字符串
		ra_arcsec = ra_position * 4.05;		//角度
		dec_arcsec = dec_position * 4.05;		//角度
		dec_arcsec = 45*3600;
		ra_arcsec = 10000 * 4.05;		//角度

		sprintf(ra_position_string, "%02d:%02d:%02d#", (int)(ra_arcsec/15/3600)%24, (int)(ra_arcsec/15/60)%60, (int)(ra_arcsec/15)%60);
		sprintf(dec_position_string, "%02d*%02d'%02d#", (int)(dec_arcsec/3600)%90, (int)(dec_arcsec/60)%60, (int)(dec_arcsec)%60);
		vTaskDelay(100/portTICK_RATE_MS);

		//printf("ra_position = %d, ra_arcsec = %f, %02d:%02d:%02d#\n", ra_position, ra_arcsec, (int)(ra_arcsec/15/3600)%24, (int)(ra_arcsec/60)%60, (int)(ra_arcsec)%60);
	}
}

void app_main(void)
{
	dec_ra_axis_gpios_init();	//初始化dec ra轴的控制gpio
	dec_tmc2208_uart_init();	//初始化dec uart通信
	ra_tmc2208_uart_init();		//初始化ra	uart通信
	i2c_master_init();			//初始化oled i2c通信
	power_input_adc_init();		//初始化电源采集adc
	menu_touchpad_init();		//初始化按键
	ra_dec_timer_init();	//初始化ra dec的定时器
	xTaskCreate(led_task, "led_task", 2048, NULL, 20, NULL);
	xTaskCreate(oled_task, "oled_task", 4096, NULL, 15, NULL);
	xTaskCreate(dec_uart_task, "dec_uart_task", 2048, NULL, 1, NULL);
	xTaskCreate(ra_uart_task, "ra_uart_task", 2048, NULL, 2, NULL);
	xTaskCreate(power_input_adc_task, "power_input_adc_task", 2048, NULL, 25, NULL);
	xTaskCreate(menu_touchpad_task, "menu_touchpad_task", 2048, NULL, 3, NULL);
	xTaskCreate(ra_dec_update_position, "ra_dec_update_position", 2048, NULL, 10, NULL);


	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	fast_scan();
}
