#include "drv_rtc.h"
#include <stddef.h>
#include <errno.h>

/**
 * @brief   RTC 根据日期计算星期
 * @param[in] time rtc_time_t 结构体指针
 * @return	星期
 */
static uint8_t rtc_calc_week(const rtc_time_t *time)
{
    uint16_t year = time->year;
    uint8_t month = time->month;
    uint8_t day = time->day;
    uint8_t week;

    if (month < 3) {
        month += 12;
        year--;
    }

    /* 使用Zeller公式计算星期 */
    week = (day + 13 * (month + 1) / 5 + year + year / 4 - year / 100 + year / 400) % 7;

    /* 调整结果为1-7对应周一到周日 */
    week = (week + 5) % 7 + 1;
    return week;
}

/* --------------------------------- 硬件抽象层 --------------------------------- */

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
#define DRV_RTC_PLATFORM_STM32F1 1
#include "stm32f10x.h"
#include <time.h>

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
#define DRV_RTC_PLATFORM_STM32F4 1
#include "stm32f4xx.h"

#else
#error drv_rtc.c: No processor defined!
#endif

/**
 * @brief	使能 RTC 时钟
 */
static void rtc_hw_rtc_clock_enable(void)
{
#if DRV_RTC_PLATFORM_STM32F1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
#elif DRV_RTC_PLATFORM_STM32F4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
#endif
}

/**
 * @brief   RTC 硬件设置时间
 * @param[in] time rtc_time_t 结构体指针
 */
static void rtc_hw_set_time(const rtc_time_t *time)
{
#if DRV_RTC_PLATFORM_STM32F1
	time_t time_cnt;		// 定义秒计数器数据类型
	struct tm time_date;	// 定义日期时间数据类型

	/* 将时间赋值给日期时间结构体 */
	time_date.tm_year = time->year - 1900;
	time_date.tm_mon  = time->month - 1;
	time_date.tm_mday = time->day;
	time_date.tm_hour = time->hour;
	time_date.tm_min  = time->minute;
	time_date.tm_sec  = time->second;

	/* 将日期时间转换为秒计数器格式 */
	time_cnt = mktime(&time_date) - 8 * 60 * 60;	// - 8 * 60 * 60为东八区的时区调整
													
	RTC_SetCounter(time_cnt);						// 将秒计数器写入到RTC的CNT中
	RTC_WaitForLastTask();							// 等待上一次操作完成
#elif DRV_RTC_PLATFORM_STM32F4
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	RTC_TimeTypeInitStructure.RTC_Hours   = time->hour;
	RTC_TimeTypeInitStructure.RTC_Minutes = time->minute;
	RTC_TimeTypeInitStructure.RTC_Seconds = time->second;
	RTC_TimeTypeInitStructure.RTC_H12     = RTC_H12_AM;	// 24小时制时此配置无意义
	
	if (RTC_SetTime(RTC_Format_BIN, &RTC_TimeTypeInitStructure) == 0)
		return;

	RTC_DateTypeDef RTC_DateTypeInitStructure;
	RTC_DateTypeInitStructure.RTC_Year    = time->year - 2000;
	RTC_DateTypeInitStructure.RTC_Month   = time->month;
	RTC_DateTypeInitStructure.RTC_Date    = time->day;
	RTC_DateTypeInitStructure.RTC_WeekDay = rtc_calc_week(time);

	if (RTC_SetDate(RTC_Format_BIN, &RTC_DateTypeInitStructure) == 0)
		return;
#endif
}

/**
 * @brief   RTC 硬件获取时间
 * @param[out] time rtc_time_t 结构体指针
 */
static void rtc_hw_get_time(rtc_time_t *time)
{
#if DRV_RTC_PLATFORM_STM32F1
	time_t time_cnt;
	struct tm time_date;
	
	/* 
	 * 读取RTC的CNT，获取当前的秒计数器
	 * + 8 * 60 * 60为东八区的时区调整
	 */
	time_cnt = RTC_GetCounter() + 8 * 60 * 60; 
	
	time_date = *localtime(&time_cnt);	// 使用localtime函数，将秒计数器转换为日期时间格式
	
	time->year   = time_date.tm_year + 1900;
	time->month  = time_date.tm_mon + 1;
	time->day    = time_date.tm_mday;
	time->hour   = time_date.tm_hour;
	time->minute = time_date.tm_min;
	time->second = time_date.tm_sec;
	time->week   = rtc_calc_week(time);

#elif DRV_RTC_PLATFORM_STM32F4
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	time->hour   = RTC_TimeStruct.RTC_Hours;
	time->minute = RTC_TimeStruct.RTC_Minutes;
	time->second = RTC_TimeStruct.RTC_Seconds;

	RTC_DateTypeDef RTC_DateStruct;
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
	time->year  = RTC_DateStruct.RTC_Year + 2000;
	time->month = RTC_DateStruct.RTC_Month;
	time->day   = RTC_DateStruct.RTC_Date;
	time->week  = RTC_DateStruct.RTC_WeekDay;
#endif
}

/**
 * @brief   初始化 RTC
 */
static void rtc_hw_rtc_init(void)
{
	const rtc_time_t init_time = {
		.year   = 2024,
		.month  = 12,
		.day    = 31,
		.hour   = 23,
		.minute = 59,
		.second = 56
	};

#if DRV_RTC_PLATFORM_STM32F1
	/* 备份寄存器访问使能 */
	PWR_BackupAccessCmd(ENABLE);

	/* 通过写入备份寄存器的标志位，判断RTC是否是第一次配置 */
	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5) {
		RCC_LSEConfig(RCC_LSE_ON);							// 开启LSE时钟
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET);	// 等待LSE准备就绪
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);				// 选择RTCCLK来源为LSE
		RCC_RTCCLKCmd(ENABLE);								// RTCCLK使能
		
		RTC_WaitForSynchro();								// 等待同步
		RTC_WaitForLastTask();								// 等待上一次操作完成
		
		RTC_SetPrescaler(32768 - 1);						// 设置RTC预分频器，预分频后的计数频率为1Hz
		RTC_WaitForLastTask();								// 等待上一次操作完成
		
		rtc_hw_set_time(&init_time);						// 设置初始时间
		
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);			// 在备份寄存器写入自己规定的标志位，用于判断RTC是不是第一次执行配置
	} else {
		/* RTC 不是第一次配置 */
		RTC_WaitForSynchro();								// 等待同步
		RTC_WaitForLastTask();								// 等待上一次操作完成
	}
#elif DRV_RTC_PLATFORM_STM32F4
	/* 备份寄存器访问使能 */
    PWR_BackupAccessCmd(ENABLE);								// 使用PWR开启对备份寄存器的访问
	
	/* 通过写入备份寄存器的标志位，判断RTC是否是第一次配置 */
	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0xA5A5) {						
		RCC_LSEConfig(RCC_LSE_ON);								// 开启LSE时钟
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET);		// 等待LSE准备就绪
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);					// 选择RTCCLK来源为LSE
		RCC_RTCCLKCmd(ENABLE);									// RTCCLK使能

		RTC_InitTypeDef RTC_InitStructure;
		RTC_InitStructure.RTC_AsynchPrediv = 0x7F;				// RTC异步分频系数(1~0X7F)
		RTC_InitStructure.RTC_SynchPrediv  = 0xFF;				// RTC同步分频系数(0~7FFF)
		RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;	// 24小时格式
		RTC_Init(&RTC_InitStructure);
 
		rtc_hw_set_time(&init_time);							// 设置初始时间
		
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);	// 在备份寄存器写入自己规定的标志位，用于判断RTC是不是第一次执行配置
	}	
#endif
}

/**
 * @brief   初始化 RTC 硬件
 */
static void rtc_hw_init(void)
{	
	rtc_hw_rtc_clock_enable();
	rtc_hw_rtc_init();
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int rtc_set_time_impl(rtc_dev_t *dev, const rtc_time_t *time);
static int rtc_get_time_impl(rtc_dev_t *dev, rtc_time_t *time);
static int rtc_deinit_impl(rtc_dev_t *dev);

/* 操作接口表 */
static const rtc_ops_t rtc_ops = {
	.set_time = rtc_set_time_impl,
	.get_time = rtc_get_time_impl,
	.deinit   = rtc_deinit_impl
};

/**
 * @brief   初始化 RTC 驱动
 * @param[out] dev rtc_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_rtc_init(rtc_dev_t *dev)
{
	if (!dev)
        return -EINVAL;
	
	dev->ops = &rtc_ops;
	rtc_hw_init();
	return 0;
}

/**
 * @brief   RTC 设置时间
 * @param[in] dev  rtc_dev_t 结构体指针
 * @param[in] time rtc_time_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int rtc_set_time_impl(rtc_dev_t *dev, const rtc_time_t *time)
{
	if (!dev || !time)
		return -EINVAL;

	rtc_hw_set_time(time);
	return 0;
}

/**
 * @brief   RTC 获取时间
 * @param[in]  dev  rtc_dev_t 结构体指针
 * @param[out] time rtc_time_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int rtc_get_time_impl(rtc_dev_t *dev, rtc_time_t *time)
{
	if (!dev)
		return -EINVAL;

	rtc_hw_get_time(time);
	return 0;
}

/**
 * @brief   去初始化 RTC
 * @param[in] dev rtc_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int rtc_deinit_impl(rtc_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}
