#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} time_info_t;

/* 天气预报天数 */
#define WEATHER_FORECAST_DAY_NUM    3

/* 每日天气信息结构体 */
typedef struct {
    char date[16];      // 日期
    char weather[16];   // 天气
    int temp_high;      // 最高气温
    int temp_low;       // 最低气温
} weather_day_info_t;

/* 天气信息结构体 */
typedef struct {
    char city[32];          // 城市
    int temp_now;           // 当前气温
    char weather_now[16];   // 当前天气
    char last_update[32];   // 最近更新时间
    weather_day_info_t daily[WEATHER_FORECAST_DAY_NUM];  // 每日天气信息
    int daily_cnt;          //天数
} weather_info_t;

/**
 * @brief	解析时间
 * @param[in]  buf  原始数据
 * @param[out] time time_info_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int parse_time(uint8_t *buf, time_info_t *time);

/**
 * @brief	解析当前天气
 * @param[in]  json_str 接收到的 JSON 数据
 * @param[out] weather  weather_info_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int parse_cur_weather(const char *json_str, weather_info_t *weather);

/**
 * @brief	解析天气预报
 * @param[in]  json_str 接收到的 JSON 数据
 * @param[out] weather  weather_info_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int parse_weather_forecast_data(const char *json_str, weather_info_t *weather);

#endif
