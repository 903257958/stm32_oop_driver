#ifndef RTC_H
#define RTC_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	#include "stm32f10x.h"
    #include <time.h>

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	#include "stm32f4xx.h"

#else
    #error rtc.h: No processor defined!
#endif

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_config_t;

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t week;
} rtc_time_t;

typedef struct rtc_dev {
    rtc_config_t config;
    rtc_time_t time;
	bool init_flag;							// 初始化标志
    int8_t (*set_time)(struct rtc_dev *dev, rtc_time_t *time);
    int8_t (*get_time)(struct rtc_dev *dev);
	int8_t (*deinit)(struct rtc_dev *dev);	// 去初始化
} rtc_dev_t;

int8_t rtc_init(rtc_dev_t *dev);

#endif
