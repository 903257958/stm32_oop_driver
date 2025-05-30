#ifndef WIFI_H
#define WIFI_H

/* 标准库头文件 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

/* 驱动头文件 */
#include "delay.h"
#include "uart.h"
#include "esp8266.h"

/* cJSON库 */
#include "cJSON.h"

/* 时间信息结构体 */
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} wifi_time_info_t;

/* 天气预报天数 */
#define WEATHER_FORECAST_DAY_NUM    3

/* Sunny Cloudy Overcast Heavy rain Thundershower */

/* 每日天气信息结构体 */
typedef struct {
    char date[16];      // 日期
    char weather[16];   // 天气
    int temp_high;      // 最高气温
    int temp_low;       // 最低气温
} wifi_weather_day_info_t;

/* 天气信息结构体 */
typedef struct {
    char city[32];          // 城市
    int temp_now;           // 当前气温
    char weather_now[16];   // 当前天气
    char last_update[32];   // 最近更新时间
    wifi_weather_day_info_t daily[WEATHER_FORECAST_DAY_NUM];  // 每日天气信息
    int daily_cnt;          //天数
} wifi_weather_info_t;

/* 函数声明 */
int8_t esp8266_get_time(esp8266_dev_t *dev, wifi_time_info_t *time);
int8_t esp8266_get_weather(esp8266_dev_t *dev, const char *city, wifi_weather_info_t *weather);

#endif
