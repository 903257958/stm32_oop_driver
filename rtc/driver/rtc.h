#ifndef __RTC_H
#define __RTC_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
	
#if defined(STM32F40_41xxx) || defined(STM32F411xE)
	#include "stm32f4xx.h"
#else
    #error rtc.h: No processor defined!
#endif

#ifndef rtc_log
    #define rtc_log(x) 
#endif

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
}RTCTime_t;

typedef struct {
	uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t week;
}RTCDate_t;

typedef struct RTCDev {
    RTCTime_t time;
    RTCDate_t date;
	bool init_flag;							// 初始化标志
    int (*set_time)(struct RTCDev *dev, RTCTime_t *time);
    int (*set_date)(struct RTCDev *dev, RTCDate_t *date);
    void (*get_time)(struct RTCDev *dev, RTCTime_t *time);
    void (*get_date)(struct RTCDev *dev, RTCDate_t *date);
	int (*deinit)(struct RTCDev *dev);		// 去初始化
}RTCDev_t;

int rtc_init(RTCDev_t *dev);

#endif
