#ifndef __COORDINATE_H
#define __COORDINATE_H

#include <math.h>

#define PI (acos(-1.0))

//时间相关函数
void setCurrentDateByString(char *str);//设置当前日期
void setCurrentTimeByString(char *str); //设置当前时间
void setCurrentSiteTimeZone(char *str);          //设置当前时区
char * getCurrentDateString();
char * getCurrentTimeString();
char * getCurrentSiteTimeZone();

//地理位置相关函数
void setCurrentSiteLongitudeByString(char * longitudeString);
void setCurrentSiteLatitudeByString(char * latitudeString);
char * getCurrentSiteLongitudeString();     //获得字符串
char * getCurrentSiteLatitudeString();      //获得字符串
double getCurrentSiteLongitudeValue();      //获得角秒值
double getCurrentSiteLatitudeValue();       //获得角秒值
double getCurrentSiteLongitudeRadValue();   //获得弧度值
double getCurrentSiteLatitudeRadValue();    //获得弧度值

//目标天体位置相关函数
void setTargetCelestialBodyRaByString(char *str);
void setTargetCelestialBodyDecByString(char *str);
void setTargetCelestialBodyRaByValue(double sec);
void setTargetCelestialBodyDecByValue(double asec);
char * getTargetCelestialBodyRaString(void);
char * getTargetCelestialBodyDecString(void);
double getTargetCelestialBodyRaValue(void);
double getTargetCelestialBodyDecValue(void);
int isPossibleToSlowToTarget(void);

//  实现坐标系间的转换
//  Ra/Dec:HH:MM:SS.S sDD*MM'SS.S  RA/Dec坐标系
//  Az/Alt:DDD*MM'SS.S sDD*MM'SS.S  Az/Alt坐标系
//  Ht/Dec:HH:MM:SS.S sDD*MM'SS.S   时角坐标系
//  电机坐标系： 0-320000pulse 0-320000pulse
//赤经转为时角
double raStr2raSec(char * raStr);                //将 RA HH:MM:SS.S 格式转换为 秒
double raSec2HaSec(double raSec);                //目标赤经转换为时间角度 返回角秒
//时角转为赤经
double haSec2raSec(double haSec);                //将时角转换为赤经
void raSec2RaStr(double raSecs, char *raStr);    //将赤经秒数转换为字符串

#endif //Coordinate 