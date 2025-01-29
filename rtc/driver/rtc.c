#include "rtc.h"

/* 函数声明 */
static int __rtc_set_time(RTCDev_t *dev, RTCTime_t *time);
static int __rtc_set_date(RTCDev_t *dev, RTCDate_t *date);
static void __rtc_get_time(RTCDev_t *dev, RTCTime_t *time);
static void __rtc_get_date(RTCDev_t *dev, RTCDate_t *date);
static int __rtc_deinit(RTCDev_t *dev);

/******************************************************************************
 * @brief	初始化RTC
 * @param	dev	:  RTCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int rtc_init(RTCDev_t *dev)
{
	if (!dev)
		return -1;

	#if defined(STM32F40_41xxx) || defined(STM32F411xE)
	/* 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);

	/* 备份寄存器访问使能 */
    PWR_BackupAccessCmd(ENABLE);									// 使用PWR开启对备份寄存器的访问
	
	/* 通过写入备份寄存器的标志位，判断RTC是否是第一次配置 */
	if (RTC_ReadBackupRegister(RTC_BKP_DR0) != 0xA5A5)
	{						
		RCC_LSEConfig(RCC_LSE_ON);									// 开启LSE时钟
		
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET);			// 等待LSE准备就绪
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);						// 选择RTCCLK来源为LSE
		RCC_RTCCLKCmd(ENABLE);										// RTCCLK使能

		RTC_InitTypeDef RTC_InitStructure;
		RTC_InitStructure.RTC_AsynchPrediv = 0x7F;					// RTC异步分频系数(1~0X7F)
		RTC_InitStructure.RTC_SynchPrediv  = 0xFF;					// RTC同步分频系数(0~7FFF)
		RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;		// 24小时格式
		RTC_Init(&RTC_InitStructure);
 
		RTCTime_t time = {23, 59, 56};
		RTCDate_t date = {25, 1, 1};
		__rtc_set_time(dev, &time);	// 设置时间
		__rtc_set_date(dev, &date);	// 设置日期
		
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);	// 在备份寄存器写入自己规定的标志位，用于判断RTC是不是第一次执行配置
	}

	#endif
	
	/* 函数指针赋值 */
	dev->set_time = __rtc_set_time;
	dev->set_date = __rtc_set_date;
	dev->get_time = __rtc_get_time;
	dev->get_date = __rtc_get_date;
	dev->deinit = __rtc_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	RTC根据日期计算星期
 * @param	time	:  RTCTime_t 结构体指针
 * @return	当前星期
 ******************************************************************************/
static uint8_t __rtc_calc_week(RTCDate_t *date)
{
	uint16_t year = date->year + 2000;
	uint8_t week;
	
    if (date->month < 3)
	{
        date->month += 12;
        year--;
    }
    
    /* Zeller 公式 */
    week = (date->date + 13 * (date->month + 1) / 5 + (year % 100) + (year % 100) / 4 + (year / 100) / 4 + 5 * (year / 100)) % 7;

    /* 映射到 1-7 为星期一~星期日 */
    week = (week + 5) % 7 + 1;
	
    return week;
}

/******************************************************************************
 * @brief	RTC设置时间
 * @param	dev	:  RTCDev_t 结构体指针
 * @param	time	:  RTCTime_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __rtc_set_time(RTCDev_t *dev, RTCTime_t *time)
{
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;

	RTC_TimeTypeInitStructure.RTC_Hours = time->hour;
	RTC_TimeTypeInitStructure.RTC_Minutes = time->minute;
	RTC_TimeTypeInitStructure.RTC_Seconds = time->second;
	RTC_TimeTypeInitStructure.RTC_H12 = RTC_H12_AM;	// 24小时制时此配置无意义
	
	return !RTC_SetTime(RTC_Format_BIN, &RTC_TimeTypeInitStructure);
}

/******************************************************************************
 * @brief	RTC设置日期
 * @param	dev	:  RTCDev_t 结构体指针
 * @param	date	:  RTCDate_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __rtc_set_date(RTCDev_t *dev, RTCDate_t *date)
{
	RTC_DateTypeDef RTC_DateTypeInitStructure;
	
	RTC_DateTypeInitStructure.RTC_Year = date->year;
	RTC_DateTypeInitStructure.RTC_Month = date->month;
	RTC_DateTypeInitStructure.RTC_Date = date->date;
	RTC_DateTypeInitStructure.RTC_WeekDay = __rtc_calc_week(date);

	return !RTC_SetDate(RTC_Format_BIN, &RTC_DateTypeInitStructure);
}

/******************************************************************************
 * @brief	RTC读取时间
 * @param	dev	:  RTCDev_t 结构体指针
 * @param	time	:  RTCTime_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __rtc_get_time(RTCDev_t *dev, RTCTime_t *time)
{
	RTC_TimeTypeDef RTC_TimeStruct;

	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	time->hour = RTC_TimeStruct.RTC_Hours;
	time->minute = RTC_TimeStruct.RTC_Minutes;
	time->second = RTC_TimeStruct.RTC_Seconds;
}

/******************************************************************************
 * @brief	RTC读取日期
 * @param	dev	:  RTCDev_t 结构体指针
 * @param	date	:  RTCDate_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __rtc_get_date(RTCDev_t *dev, RTCDate_t *date)
{
	RTC_DateTypeDef RTC_DateStruct;

	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
	date->year = RTC_DateStruct.RTC_Year;
	date->month = RTC_DateStruct.RTC_Month;
	date->date = RTC_DateStruct.RTC_Date;
	date->week = RTC_DateStruct.RTC_WeekDay;
}

/******************************************************************************
 * @brief	去初始化RTC
 * @param	dev	:  RTCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __rtc_deinit(RTCDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
