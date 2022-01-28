#include "coordinate.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

static char currentSiteLongitudeString[32] = {0};    //当前位置经度 经度以0°自西向东递增到360°
static char currentSiteLatitudeString[32] = {0};     //当前位置纬度 纬度从北极到南极 +90°到-90°    sDD*MM
static double currentSiteLongitudeValue = 0;         // 0'' ~ 360*3600'' 角秒
static double currentSiteLatitudeValue = 0;          // -90*3600'' ~ + 90*3600'' 角秒
static double currentSiteLongitudeRadValue = 0;      //弧度值
static double currentSiteLatitudeRadValue = 0;       //弧度值
//经度转换为角秒 sDDD*MM
//The current site Longitude. East Longitudes are expressed as negative
//str 实际东经 为 360度 的补角
static double longitude2asec(char *str) //sDDD*MM 
{
    int longitudeDegree, longitudeAmin;
    sscanf(str, "%d*%d", &longitudeDegree, &longitudeAmin);
    return (longitudeDegree * 3600.0) + (longitudeAmin*60.0);
}
static double latitude2asec(char *str)  //sDD*MM
{
    int latitudeDegree, latitudeAmin;
    sscanf(str, "%d*%d", &latitudeDegree, &latitudeAmin);
    if(latitudeDegree<0)  //负值
        return -((abs(latitudeDegree) * 3600.0) + (latitudeAmin * 60.0));
    else
        return (latitudeDegree * 3600.0) + (latitudeAmin * 60.0);
}
static double longitudeAsec2Rad(double Asec)
{
    return Asec/(360.0*3600.0)*2*PI;
}

static double latitudeAsec2Rad(double Asec)
{
    return Asec/(360.0*3600.0)*2*PI;
}
//设置经度  Set current site’s longitude to DDD*MM an ASCII position string
void setCurrentSiteLongitudeByString(char * longitudeString)
{
    strcpy(currentSiteLongitudeString, longitudeString);
    currentSiteLongitudeValue = longitude2asec(currentSiteLongitudeString);
    currentSiteLongitudeRadValue = longitudeAsec2Rad(currentSiteLongitudeValue);
}
//设置纬度
void setCurrentSiteLatitudeByString(char * latitudeString)
{
    strcpy(currentSiteLatitudeString, latitudeString);
    currentSiteLatitudeValue = latitude2asec(currentSiteLatitudeString);
    currentSiteLatitudeRadValue = latitudeAsec2Rad(currentSiteLatitudeValue);
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
double getCurrentSiteLongitudeRadValue()   //获得弧度值
{
    return currentSiteLongitudeRadValue;
}

double getCurrentSiteLatitudeRadValue()   //获得弧度值
{
    return currentSiteLatitudeRadValue;
}


static char currentDateString[32];
static char currentTimeString[32];
static char currentTimeZoneString[32];
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
    timeInfo->tm_wday = 0;
    timeInfo->tm_yday = 0;

    now = mktime(timeInfo);        //得到时间戳

    struct timeval tvDelta = {.tv_sec = now, .tv_usec = now % 1000000L};
    settimeofday(&tvDelta, NULL);   //设置时间
}
//获得当前日期 MM/DD/YY
char * getCurrentDateString(void)
{
    struct tm *timeInfo;
    time_t now;

    time(&now);
    timeInfo = localtime(&now);

    sprintf(currentDateString, "%02d/%02d/%02d", timeInfo->tm_mon+1, timeInfo->tm_mday, timeInfo->tm_year+1900-2000);
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

    now = mktime(timeInfo);        //得到时间戳

    struct timeval tvDelta = {.tv_sec = now, .tv_usec = now % 1000000L};
    settimeofday(&tvDelta, NULL);   //设置时间
}
//获得当前时间
char * getCurrentTimeString(void)
{
    struct tm *timeInfo;
    time_t now;

    time(&now);
    timeInfo = localtime(&now);

    sprintf(currentTimeString, "%02d:%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
    return currentTimeString;
}
//设置当前时区
void setCurrentSiteTimeZone(char *str)
{
    //东区 西区 暂时默认为东八区北京时间
    printf("setCurrentSiteTimeZone = %s\n", str);
    setenv("TZ", "CST-8", 1);
    tzset();
}
//得到当前时区
char * getCurrentSiteTimeZone(void)
{
    strcpy(currentTimeZoneString, "-8.0");
    return currentTimeZoneString;
}

static char targetCelestialBodyRaString[32] = {0};       //目标天体赤经
static char targetCelestialBodyDecString[32] = {0};      //目标天体赤纬
static double targetCelestialBodyRaValue = 0;
static double targetCelestialBodyDecValue = 0;
static double targetCelestialBodyRaRadValue = 0;
static double targetCelestialBodyDecRadValue = 0;
static int targetCelestialBodyRaPulseValue1 = 0;        //对应到电机上有两个位置
static int targetCelestialBodyRaPulseValue2 = 0;        //得到当前目标的 Ha/Dec 坐标系 映射到电机坐标系中， 求出电机坐标系 pulseValue
static int targetCelestialBodyDecPulseValue1 = 0;
static int targetCelestialBodyDecPulseValue2 = 0;       //dec [0-160000)对应一个  raPulseValue   [160000-320000)对应一个 raPulseValue

//将赤经 秒数表示 转换为字符串 HH:MM:SS
void raSec2RaStr(double raSecs, char *raStr)
{
    int raHour, raMin, raSec;

    raHour = (int)(raSecs)/3600%24;
    raMin = (int)(raSecs)%3600/60;
    raSec = (int)(raSecs)%3600%60;

    sprintf(raStr, "%02d:%02d:%02d", raHour, raMin, raSec);
}

//将 RA HH:MM:SS.S 格式转换为 秒
double raStr2raSec(char * raStr) 
{
    int hour, min;
    double sec;
    sscanf(raStr, "%d:%d:%lf", &hour, &min, &sec);

    return hour*3600+min*60+sec;
}

void decAsec2decStr(double decAsecs, char *decStr)
{
    int DD, MM, SS;
    if(decAsecs<0)
    {
        decAsecs = -(decAsecs);
        DD = (int)(decAsecs)/3600%24;
        MM = (int)(decAsecs)%3600/60;
        SS = (int)(decAsecs)%3600%60;

        sprintf(decStr, "-%02d*%02d:%02d", DD, MM, SS);
    }
    else
    {
        DD = (int)(decAsecs)/3600%24;
        MM = (int)(decAsecs)%3600/60;
        SS = (int)(decAsecs)%3600%60;

        sprintf(decStr, "%02d*%02d:%02d", DD, MM, SS);
    }
}

double decStr2decAsec(char * decStr) //sDD*MM:SS
{
    int DD, MM, SS;
    sscanf(decStr, "%d:%d:%d", &DD, &MM, &SS);
    if(DD<0)    //如果小于零
        return -(((abs(DD)*3600.0))+(MM*60.0)+SS);
    else
        return (DD*3600.0)+(MM*60.0)+SS;
}

//设置目标赤经值
void setTargetCelestialBodyRaByString(char *str)
{
    strcpy(targetCelestialBodyRaString, str);
    targetCelestialBodyRaValue = raStr2raSec(targetCelestialBodyRaString);
    targetCelestialBodyRaRadValue = targetCelestialBodyRaValue / 3600.0 * 15 /180.0 * PI;
}
//设置目标赤纬值
void setTargetCelestialBodyDecByString(char *str)
{
    strcpy(targetCelestialBodyDecString, str);
    targetCelestialBodyDecValue = decStr2decAsec(targetCelestialBodyDecString);
    targetCelestialBodyDecRadValue = targetCelestialBodyDecValue / 3600.0 / 180.0 * PI;
}
//设置目标赤经值
void setTargetCelestialBodyRaByValue(double sec)
{
    targetCelestialBodyRaValue = sec;
    raSec2RaStr(sec, targetCelestialBodyRaString);
    targetCelestialBodyRaRadValue = targetCelestialBodyRaValue / 3600.0 * 15 /180.0 * PI;
}
//设置目标赤纬值
void setTargetCelestialBodyDecByValue(double asec)
{
    targetCelestialBodyDecValue = asec;
    decAsec2decStr(asec, targetCelestialBodyDecString);
    targetCelestialBodyDecRadValue = targetCelestialBodyDecValue / 3600.0 / 180.0 * PI;
}
//得到目标赤经字符串
char * getTargetCelestialBodyRaString(void)
{
    return targetCelestialBodyRaString;
}
//得到目标赤纬字符串
char * getTargetCelestialBodyDecString(void)
{
    return targetCelestialBodyDecString;
}
//得到目标赤经 秒值
double getTargetCelestialBodyRaValue(void)
{
    return targetCelestialBodyRaValue;
}
//得到目标赤纬 角秒值
double getTargetCelestialBodyDecValue(void)
{
    return targetCelestialBodyDecValue;
}

double getTargetCelestialBodyRaRadValue(void)
{
    return targetCelestialBodyRaRadValue;
}

double getTargetCelestialBodyDecRadValue(void)
{
    return targetCelestialBodyDecRadValue;
}

//目标赤经转换为时间角度 返回角秒
double raSec2HaSec(double raSec) 
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

int isPossibleToSlowToTarget(void)
{
    //计算天体的目标位置，看是否低于地平线
    double latiRad = getCurrentSiteLatitudeRadValue();
    double decRad = getTargetCelestialBodyDecRadValue();
    double haRad = raSec2HaSec(getTargetCelestialBodyRaValue())*15.0/3600.0/180.0*PI;

    printf("%lf  %lf  %lf\n", latiRad, decRad, haRad);

    double altRad = asin(sin(latiRad)*sin(decRad)+cos(latiRad)*cos(decRad)*cos(haRad));

    int degree, amin;
    double asec;
    double altAsec = altRad/(2*PI)*360.0*3600.0;  //转换为角秒

    if(altAsec < 0)
    {
        altAsec = -altAsec;
        degree = (int)altAsec/3600;
        amin = (int)altAsec%3600/60;
        asec = altAsec-(degree*3600)-(amin*60);
        printf("target alt:-%02d*%02d'%02.0f\n", degree, amin, fabs(asec));
        return 0;
    }
    else
    {
        degree = (int)altAsec/3600;
        amin = (int)altAsec%3600/60;
        asec = altAsec-(degree*3600)-(amin*60);
        printf("target alt:%02d*%02d'%02.0f\n", degree, amin, fabs(asec));
        return 1;
    }
    
    if(altRad < 0)  //低于地平线
        return 0;
    else
        return 1;
}

double haSec2raSec(double haSec)
{
    time_t now;
    struct tm *timeinfo;
    time(&now);
    timeinfo = localtime(&now);
    double seconds = timeinfo->tm_hour*3600+timeinfo->tm_min*60+timeinfo->tm_sec;    //得出当天秒数
    double longitudeAsec = (360.0*3600.0) - longitude2asec(currentSiteLongitudeString);

    double raSec = ((6*3600)+(40*60))+(timeinfo->tm_yday*(3*60+56))+seconds - (((120*3600)-longitudeAsec)/15) - haSec;
    if(raSec<0)
        raSec = 24*3600.0 + raSec;

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

