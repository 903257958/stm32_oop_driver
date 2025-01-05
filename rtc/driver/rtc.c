#include "rtc.h"

/* 函数声明 */
static int __rtc_set_time(RTCDev_t *pDev, RTCTime_t *pTime);
static int __rtc_set_date(RTCDev_t *pDev, RTCDate_t *pDate);
static void __rtc_get_time(RTCDev_t *pDev, RTCTime_t *pTime);
static void __rtc_get_date(RTCDev_t *pDev, RTCDate_t *pDate);
static int __rtc_deinit(RTCDev_t *pDev);

/******************************************************************************
 * @brief	初始化RTC
 * @param	pDev	:  RTCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int rtc_init(RTCDev_t *pDev)
{
	if (!pDev)
		return -1;

	#if defined(STM32F40_41xxx)

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
		RTCDate_t date = {25, 1, 1, 3};
		__rtc_set_time(pDev, &time);	// 设置时间
		__rtc_set_date(pDev, &date);	// 设置日期
		
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);	// 在备份寄存器写入自己规定的标志位，用于判断RTC是不是第一次执行配置
	}

	#endif
	
	/* 函数指针赋值 */
	pDev->set_time = __rtc_set_time;
	pDev->set_date = __rtc_set_date;
	pDev->get_time = __rtc_get_time;
	pDev->get_date = __rtc_get_date;
	pDev->deinit = __rtc_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	RTC设置时间
 * @param	pDev	:  RTCDev_t 结构体指针
 * @param	pTime	:  RTCTime_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __rtc_set_time(RTCDev_t *pDev, RTCTime_t *pTime)
{
	RTC_TimeTypeDef RTC_TimeTypeInitStructure;

	RTC_TimeTypeInitStructure.RTC_Hours = pTime->hour;
	RTC_TimeTypeInitStructure.RTC_Minutes = pTime->minute;
	RTC_TimeTypeInitStructure.RTC_Seconds = pTime->second;
	RTC_TimeTypeInitStructure.RTC_H12 = RTC_H12_AM;	// 24小时制时此配置无意义
	
	return !RTC_SetTime(RTC_Format_BIN, &RTC_TimeTypeInitStructure);
}

/******************************************************************************
 * @brief	RTC设置日期
 * @param	pDev	:  RTCDev_t 结构体指针
 * @param	pDate	:  RTCDate_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __rtc_set_date(RTCDev_t *pDev, RTCDate_t *pDate)
{
	RTC_DateTypeDef RTC_DateTypeInitStructure;
	
	RTC_DateTypeInitStructure.RTC_Year = pDate->year;
	RTC_DateTypeInitStructure.RTC_Month = pDate->month;
	RTC_DateTypeInitStructure.RTC_Date = pDate->date;
	RTC_DateTypeInitStructure.RTC_WeekDay = pDate->week;

	return !RTC_SetDate(RTC_Format_BIN, &RTC_DateTypeInitStructure);
}

/******************************************************************************
 * @brief	RTC读取时间
 * @param	pDev	:  RTCDev_t 结构体指针
 * @param	pTime	:  RTCTime_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __rtc_get_time(RTCDev_t *pDev, RTCTime_t *pTime)
{
	RTC_TimeTypeDef RTC_TimeStruct;

	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	pTime->hour = RTC_TimeStruct.RTC_Hours;
	pTime->minute = RTC_TimeStruct.RTC_Minutes;
	pTime->second = RTC_TimeStruct.RTC_Seconds;
}

/******************************************************************************
 * @brief	RTC读取日期
 * @param	pDev	:  RTCDev_t 结构体指针
 * @param	pDate	:  RTCDate_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __rtc_get_date(RTCDev_t *pDev, RTCDate_t *pDate)
{
	RTC_DateTypeDef RTC_DateStruct;

	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
	pDate->year = RTC_DateStruct.RTC_Year;
	pDate->month = RTC_DateStruct.RTC_Month;
	pDate->date = RTC_DateStruct.RTC_Date;
	pDate->week = RTC_DateStruct.RTC_WeekDay;
}

/******************************************************************************
 * @brief	去初始化RTC
 * @param	pDev	:  RTCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __rtc_deinit(RTCDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pDev->initFlag = false;	// 修改初始化标志
	
	return 0;
}
