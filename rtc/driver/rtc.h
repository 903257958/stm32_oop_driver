#ifndef __RTC_H
#define __RTC_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
	
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
	uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t week;
}RTCDate_t;

typedef struct RTCDev {
	bool initFlag;							// 初始化标志
    int (*set_time)(struct RTCDev *pDev, RTCTime_t *pTime);
    int (*set_date)(struct RTCDev *pDev, RTCDate_t *pDate);
    void (*get_time)(struct RTCDev *pDev, RTCTime_t *pTime);
    void (*get_date)(struct RTCDev *pDev, RTCDate_t *pDate);
	int (*deinit)(struct RTCDev *pDev);		// 去初始化
}RTCDev_t;

int rtc_init(RTCDev_t *pDev);

#endif
