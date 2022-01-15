#include "wifi.h"
#include <string.h>
#include <esp_ota_ops.h>
#include "coordinate.h"

char ipString[30];
static char dataBuff[256];
static const char *TAG = "scan";
static int connectSocket = 0;
static int serverSocket;

static TaskHandle_t tcpRecvTaskHandle;

//指令环形缓冲区  最多缓存8条指令  来不及处理会被覆盖
static char cmdRingBuff[8][32];             //缓冲区
static char *startCmdPtr = cmdRingBuff[0];  //缓冲区头部
static char *endCmdPtr = cmdRingBuff[7];    //缓冲区尾部
static char *writeCmdPtr = cmdRingBuff[0];   //写入指针
static char *readCmdPtr = cmdRingBuff[0];    //读取指针
static int cmdCnt = 0;
//得到下一个写地址
static char * nextWriteCmdPtr(void)
{
    if(writeCmdPtr+sizeof(cmdRingBuff[0]) > endCmdPtr)
        return startCmdPtr;
    else
        return writeCmdPtr+sizeof(cmdRingBuff[0]);
}
//得到下一个读取地址
static char * nextReadCmdPtr(void)
{
    if(readCmdPtr+sizeof(cmdRingBuff[0]) > endCmdPtr)
        return startCmdPtr;
    else
        return readCmdPtr+sizeof(cmdRingBuff[0]);
}
//判断环形缓冲区是否为空
static int isEmpty(void)   //空  返回 1  非空返回 0
{
    return cmdCnt==0 ? 1:0;
}
//判断环形缓冲区是否满
static int isFull()
{
    return cmdCnt==8 ? 1:0;
}
//读出一条指令
static void popCmd(char *str)
{
    if(isEmpty() == 0)  //非空
    {
        if(str!=NULL)
            strcpy(str, readCmdPtr);

        //指向下一个读取地址
        readCmdPtr = nextReadCmdPtr();
        cmdCnt++;
    }
}
//写入一条指令存放到 str 中
static void pushCmd(char *str)
{
    if(isFull() == 0)    //未满
    {
        strcpy(writeCmdPtr, str);

        //指向下一个写入地址
        writeCmdPtr = nextWriteCmdPtr(); 
        cmdCnt--;  
    }
    else
    {
        popCmd(NULL);    //满了弹出一条指令丢弃
        return pushCmd(str);  //插入新的指令
    }
}

static void execCmd(void)
{
    char cmdTmpBuff[32];     //存储取出的指令
    char sendBuff[64];
    while(!isEmpty())
    {
        popCmd(cmdTmpBuff);
        if(strncmp(cmdTmpBuff, "GVD", 3)==0)        //Get Telescope Firmware Date
        {
            const esp_app_desc_t *app_desc = esp_ota_get_app_description();
            sprintf(sendBuff, "%s#", app_desc->date);
            send(connectSocket, sendBuff, strlen(sendBuff), 0);
        }
        else if(strncmp(cmdTmpBuff, "GVP", 3)==0)   //Get Telescope Product Name
        {
            send(connectSocket, "my telescope#", strlen("my telescope#"), 0);
        }
        else if(strncmp(cmdTmpBuff, "GVN", 3)==0)   //Get Telescope Firmware Number
        {
            send(connectSocket, "01.0#", strlen("01.0#"), 0);
        }
        else if(strncmp(cmdTmpBuff, "GVT", 3)==0)   //Get Telescope Firmware Time
        {
            const esp_app_desc_t *app_desc = esp_ota_get_app_description();
            sprintf(sendBuff, "%s#", app_desc->time);
            send(connectSocket, sendBuff, strlen(sendBuff), 0);
        }
        else if(strncmp(cmdTmpBuff, "GR", 2)==0)    //得到当前赤经值
        {
            char aTString[32];
            getCurrentRaValueString(aTString);
            sprintf(sendBuff, "%s#", aTString);
            send(connectSocket, sendBuff, strlen(sendBuff), 0);
        }
        else if(strncmp(cmdTmpBuff, "GD", 2)==0)    //得到当前赤纬值
        {
            char decString[32];
            getCurrentDecValueString(decString);
            sprintf(sendBuff, "%s#", decString);
            send(connectSocket, sendBuff, strlen(sendBuff), 0);
        }
        else if(strncmp(cmdTmpBuff, "D", 1)==0)
        {
            send(connectSocket, "i don't know!#", strlen("i don't know!#"), 0);
        }
        else if(strncmp(cmdTmpBuff, "Gg", 2)==0)    //得到当前位置经度
        {

        }
        else if(strncmp(cmdTmpBuff, "Sg", 2)==0)    //设置当前位置经度
        {

        }
        else if(strncmp(cmdTmpBuff, "GC", 2)==0)    //得到当前日期
        {
            sprintf(sendBuff, "%s#", getCurrentDateString());
            send(connectSocket, sendBuff, strlen(sendBuff), 0);
        }
        else if(strncmp(cmdTmpBuff, "SC", 2) ==0)   //设置当前日期
        {
            setCurrentDateByString(cmdTmpBuff+2);
            send(connectSocket, "1Updating Planetary Data#", strlen("1Updating Planetary Data#"), 0);
        }
        else if(strncmp(cmdTmpBuff, "GL", 2) == 0)  //得到当前时间
        {
            sprintf(sendBuff, "%s#", getCurrentTimeString());
            send(connectSocket, sendBuff, strlen(sendBuff), 0);
        }
        else if(strncmp(cmdTmpBuff, "SL", 2) == 0)  //设置当前时间
        {
            setCurrentTimeByString(cmdTmpBuff+2);
            send(connectSocket, "1", strlen("1"), 0);
        }
        else if(strncmp(cmdTmpBuff, "GG", 2) == 0)  //得到当前时区
        {
            send(connectSocket, "-8.0#", strlen("-8.0#"), 0);
        }
        else if(strncmp(cmdTmpBuff, "SG", 2) == 0)  //设置当前时区
        {
            setCurrentSiteTimeZone(cmdTmpBuff+2);
            send(connectSocket, "1", strlen("1"), 0);
        }

    }
}

static void tcpServerRecvDataTask(void *pvParameters)
{
	int len = 0;	//长度 char dataBuff[1024]; //缓存 while (1)
	while(1)
	{
		//读取接收数据， 将数据放到环形缓冲区中 
		len = recv(connectSocket, dataBuff, sizeof(dataBuff), 0);   //阻塞直到有数据，或则断开了连接
		if (len > 0)
		{
            dataBuff[len] = '\0';
			if(dataBuff[0]==0x06)   //Query of alignment mounting mode.
			{
				send(connectSocket, "A", strlen("A"), 0);
			}
            else
            {
                char cmdTmpBuff[32];
                for(int i = 0; i<len; i++)  //遍历数据，找出命令
                {
                    //定位包头 
                    if(dataBuff[i] == ':')
                    {
                        for(int j=0; j<len; j++)
                        {
                            if(dataBuff[i+j+1] == '#')  //定位包尾
                            {
                                cmdTmpBuff[j] = '\0';
                                pushCmd(cmdTmpBuff);
                                i = i+j;
                                break;
                            }
                            else
                            {
                                cmdTmpBuff[j] = dataBuff[i+j+1];
                            }
                        }
                    }
                }
            }

            execCmd();          //逐个执行命令
		}
        else
        {
            close(connectSocket);                           //异常，关闭客户端连接

            struct sockaddr_in clientAddr;
	        socklen_t sockLen;
            connectSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &sockLen);     //等待下一个客户端连接
            //判断是否连接成功
            if (connectSocket < 0)
            {
                close(serverSocket);                        //异常，关闭tcp server
                vTaskDelete(tcpRecvTaskHandle);             //删除任务，等待重建
            }
        }
	}
}

static esp_err_t createTcpServer()
{
    close(serverSocket);   //关闭上次建立tcp_server;
    vTaskDelay(100);        //等待 server_recv_data 任务里的

	serverSocket = socket(AF_INET, SOCK_STREAM, 0);

	if (serverSocket < 0)
	{
		return ESP_FAIL;
	}
	struct sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(6000);          //指定的端口号
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	//开始创建
	if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
	{
		close(serverSocket);
		return ESP_FAIL;
	}

    //监听指定的端口
    if (listen(serverSocket, 5) < 0)
    {
        close(serverSocket);
        return ESP_FAIL;
    }

	struct sockaddr_in clientAddr;
	socklen_t sockLen;
    connectSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &sockLen);     //阻塞直到有客户端连接
    //判断是否连接成功
    if (connectSocket < 0)
    {
        close(serverSocket);
        return ESP_FAIL;
    }

    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
	xTaskCreate(tcpServerRecvDataTask, "tcpServerRecvDataTask", 2048, NULL, 8, &tcpRecvTaskHandle);
    return ESP_OK;
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		sprintf(ipString, IPSTR, IP2STR(&event->ip_info.ip));           //记录IP
		createTcpServer();
    }
}

/* Initialize Wi-Fi as sta and set scan method */
void fastScan(void)
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