#include "coordinate.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

char currentSiteLongitudeString[32] = {0};    //当前位置经度
char currentSiteLatitudeString[32] = {0};     //当前位置纬度
double currentSiteLongitudeValue = 0;
double currentSiteLatitudeValue = 0;
//经度转换为角秒 sDDD*MM
//The current site Longitude. East Longitudes are expressed as negative
//str 实际东经 为 360度 的补角
static double longitude2asec(char *str)
{
    int longitudeDegree, longitudeAmin;
    sscanf(str, "%d*%d", &longitudeDegree, &longitudeAmin);
    return (longitudeDegree * 3600.0) + (longitudeAmin*60.0);
}
//设置经度  Set current site’s longitude to DDD*MM an ASCII position string
void setCurrentSiteLongitudeByString(char * longitudeString)
{
    strcpy(currentSiteLatitudeString, longitudeString);
    currentSiteLongitudeValue = longitude2asec(currentSiteLongitudeString);
}
//设置纬度
void setCurrentSiteLatitudeByString(char * latitudeString)
{
    strcpy(currentSiteLatitudeString, latitudeString);
}
//设置经度
void setCurrentSiteLongitudeByValue(double longitudeValue)
{

}
//设置纬度
void setCurrentSiteLatitudeByValue(double latitudeValue)
{

}
//获取经度
char * getCurrentSiteLongitudeString()
{
    return currentSiteLongitudeString;
}
//获取纬度
char * getCurrentSiteLatitudeString()
{
    return currentSiteLatitudeString;
}
//获取经度
double getCurrentSiteLongitudeValue()
{
    return currentSiteLongitudeValue;
}
//获取纬度
double getCurrentSiteLatitudeValue()
{
    return currentSiteLatitudeValue;
}

//char targetCelestialBodyRaString[32] = {0};       //目标天体赤经
//char targetCelestialBodyDecString[32] = {0};      //目标天体赤纬

char currentDateString[32];
char currentTimeString[32];
//设置当前日期  MM/DD/YY  月/日/年
void setCurrentDateByString(char *str)
{
    int MM, DD, YY;
    struct tm *timeInfo;
    time_t now;

    sscanf(str, "%d/%d/%d", &MM, &DD, &YY);
    time(&now);                     //获得当前时间  时间戳
    timeInfo = localtime(&now);     //转换为当前时区的时间

    YY += 2000;                     //更改 年月 日
    timeInfo->tm_year = YY-1900;
    timeInfo->tm_mon = MM-1;
    timeInfo->tm_mday = DD;

    now = mktime(&timeInfo);        //得到时间戳

    struct timeval tvDelta = {.tv_sec = now, .tv_usec = now % 1000000L};
    settimeofday(&tvDelta, NULL);   //设置时间
}
//获得当前日期 MM/DD/YY
char * getCurrentDateString()
{
    struct tm *timeInfo;
    time_t now;

    time(&now);
    timeInfo = localtime(&now);

    sprintf(currentDateString, "%02d/%02d/%02d", timeInfo->tm_year+1900, timeInfo->tm_mon+1, timeInfo->tm_mday);
    return currentDateString;
}
//设置当前时间 HH:MM:SS
void setCurrentTimeByString(char *str)
{
    int HH, MM, SS;
    struct tm *timeInfo;
    time_t now;

    sscanf(str, "%d:%d:%d", &HH, &MM, &SS);
    time(&now);                     //获得当前时间  时间戳
    timeInfo = localtime(&now);     //转换为当前时区的时间

    timeInfo->tm_hour = HH;
    timeInfo->tm_min = MM;
    timeInfo->tm_sec = SS;

    now = mktime(&timeInfo);        //得到时间戳

    struct timeval tvDelta = {.tv_sec = now, .tv_usec = now % 1000000L};
    settimeofday(&tvDelta, NULL);   //设置时间
}
//获得当前时间
char * getCurrentTimeString(char *str)
{
    struct tm *timeInfo;
    time_t now;

    time(&now);
    timeInfo = localtime(&now);

    sprintf(currentTimeString, "%02d:%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
    return currentTimeString;
}

void setCurrentSiteTimeZone(char *str)
{
    printf("setCurrentSiteTimeZone = %s\n", str);
    setenv("TZ", "CST-8", 1);
    tzset();
}

//将 RA HH:MM:SS.S 格式转换为 秒
double raStr2raSec(char * raStr) 
{
    int hour, min;
    double sec;
    sscanf(raStr, "%d:%d:%lf", &hour, &min, &sec);

    return hour*3600+min*60+sec;
}

//目标赤经转换为时间角度 返回角秒
double raSec2HtSec(double raSec) 
{
    //当前日期距离 1月1日 天数
    time_t now;
    struct tm *timeinfo;
    time(&now);                     //获得当前时间  时间戳
    timeinfo = localtime(&now);     //转换为当前时区的时间
    //printf("local time now: %04d/%02d/%02d %02d:%02d:%02d\n", timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    double seconds = timeinfo->tm_hour*3600+timeinfo->tm_min*60+timeinfo->tm_sec;    //得出当天秒数
    double longitudeAsec = (360.0*3600.0) - longitude2asec(currentSiteLongitudeString);

    double htSec = ((6*3600)+(40*60))+(timeinfo->tm_yday*(3*60+56))+seconds - (((120*3600)-longitudeAsec)/15)-raSec;

    return htSec;
}

double htSec2raSec(double htSec)
{
    time_t now;
    struct tm *timeinfo;
    time(&now);
    timeinfo = localtime(&now);
    double seconds = timeinfo->tm_hour*3600+timeinfo->tm_min*60+timeinfo->tm_sec;    //得出当天秒数
    double longitudeAsec = (360.0*3600.0) - longitude2asec(currentSiteLongitudeString);

    double raSec = ((6*3600)+(40*60))+(timeinfo->tm_yday*(3*60+56))+seconds - (((120*3600)-longitudeAsec)/15) - htSec;

    return raSec;
}

//将时角 秒数表示 转换为字符串 HH:MM:SS
void htSec2HtStr(double htSecs, char *hourTimeStr)
{
    int htHour, htMin, htSec;

    htHour = (int)(htSecs)/3600%24;
    htMin = (int)(htSecs)%3600/60;
    htSec = (int)(htSecs)%3600%60;

    sprintf(hourTimeStr, "%02d:%02d:%02d", htHour, htMin, htSec);
}

//将赤经 秒数表示 转换为字符串 HH:MM:SS
void raSec2RaStr(double raSecs, char *raStr)
{
    int raHour, raMin, raSec;

    raHour = (int)(raSecs)/3600%24;
    raMin = (int)(raSecs)%3600/60;
    raSec = (int)(raSecs)%3600%60;

    sprintf(raStr, "%02d:%02d:%02d", raHour, raMin, raSec);
}