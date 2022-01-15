#ifndef __WIFI_H
#define __WIFI_H

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <sys/socket.h>
#include "motor.h"

extern char ipString[30];      //ip 地址
extern void fastScan(void);    //扫描Wifi 并建立连接

#endif