#include "rtc.h"

/* 函数声明 */
static uint8_t __rtc_calc_week(RTCTime_t *time);
static int8_t __rtc_set_time(RTCDev_t *dev, RTCTime_t *time);
static int8_t __rtc_get_time(RTCDev_t *dev);
static int8_t __rtc_deinit(RTCDev_t *dev);

/******************************************************************************
 * @brief	初始化RTC
 * @param	dev	:  RTCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t rtc_init(RTCDev_t *dev)
{
	if (!dev)
		return -1;

	dev->init_flag = true;

	RTCTime_t rtc_time = {
		.year = dev->config.year,
		.month = dev->config.month,
		.day = dev->config.day,
		.hour = dev->config.hour,
		.minute = dev->config.minute,
		.second = dev->config.second,
	};

	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	/* 开启时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);

	/*备份寄存器访问使能*/
	PWR_BackupAccessCmd(ENABLE);

	/* 通过写入备份寄存器的标志位，判断RTC是否是第一次配置 */
	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	{
		RCC_LSEConfig(RCC_LSE_ON);							// 开启LSE时钟
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET);	// 等待LSE准备就绪
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);				// 选择RTCCLK来源为LSE
		RCC_RTCCLKCmd(ENABLE);								// RTCCLK使能
		
		RTC_WaitForSynchro();								// 等待同步
		RTC_WaitForLastTask();								// 等待上一次操作完成
		
		RTC_SetPrescaler(32768 - 1);						// 设置RTC预分频器，预分频后的计数频率为1Hz
		RTC_WaitForLastTask();								// 等待上一次操作完成
		
		__rtc_set_time(dev, &rtc_time);						// 设置初始时间
		
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);			// 在备份寄存器写入自己规定的标志位，用于判断RTC是不是第一次执行配置
	}
	else													// RTC不是第一次配置
	{
		RTC_WaitForSynchro();								// 等待同步
		RTC_WaitForLastTask();								// 等待上一次操作完成
	}

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

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
 
		__rtc_set_time(dev, &rtc_time);								// 设置初始时间
		
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5);	// 在备份寄存器写入自己规定的标志位，用于判断RTC是不是第一次执行配置
	}

	#endif
	
	/* 函数指针赋值 */
	dev->set_time = __rtc_set_time;
	dev->get_time = __rtc_get_time;
	dev->deinit = __rtc_deinit;
	
	return 0;
}

/******************************************************************************
 * @brief	RTC设置时间
 * @param	dev	:  RTCDev_t 结构体指针
 * @param	time	:  RTCTime_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rtc_set_time(RTCDev_t *dev, RTCTime_t *time)
{
	if (!dev || !dev->init_flag)
		return -1;

	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	time_t time_cnt;		// 定义秒计数器数据类型
	struct tm time_date;	// 定义日期时间数据类型
	
	/* 将时间赋值给日期时间结构体 */
	time_date.tm_year = time->year - 1900;
	time_date.tm_mon = time->month - 1;
	time_date.tm_mday = time->day;
	time_date.tm_hour = time->hour;
	time_date.tm_min = time->minute;
	time_date.tm_sec = time->second;

	/* 将日期时间转换为秒计数器格式 */
	time_cnt = mktime(&time_date) - 8 * 60 * 60;	// - 8 * 60 * 60为东八区的时区调整
													
	RTC_SetCounter(time_cnt);						// 将秒计数器写入到RTC的CNT中
	RTC_WaitForLastTask();							// 等待上一次操作完成

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

	RTC_TimeTypeDef RTC_TimeTypeInitStructure;
	RTC_TimeTypeInitStructure.RTC_Hours = time->hour;
	RTC_TimeTypeInitStructure.RTC_Minutes = time->minute;
	RTC_TimeTypeInitStructure.RTC_Seconds = time->second;
	RTC_TimeTypeInitStructure.RTC_H12 = RTC_H12_AM;	// 24小时制时此配置无意义
	
	if (RTC_SetTime(RTC_Format_BIN, &RTC_TimeTypeInitStructure) == 0)
	{
		return -2;
	}

	RTC_DateTypeDef RTC_DateTypeInitStructure;
	RTC_DateTypeInitStructure.RTC_Year = time->year - 2000;
	RTC_DateTypeInitStructure.RTC_Month = time->month;
	RTC_DateTypeInitStructure.RTC_Date = time->day;
	RTC_DateTypeInitStructure.RTC_WeekDay = __rtc_calc_week(time);

	if (RTC_SetDate(RTC_Format_BIN, &RTC_DateTypeInitStructure) == 0)
	{
		return -3;
	}

	#endif

	return 0;
}

/******************************************************************************
 * @brief	RTC读取时间
 * @param	dev	:  RTCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rtc_get_time(RTCDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	time_t time_cnt;		// 定义秒计数器数据类型
	struct tm time_date;	// 定义日期时间数据类型
	
	time_cnt = RTC_GetCounter() + 8 * 60 * 60;		// 读取RTC的CNT，获取当前的秒计数器
													// + 8 * 60 * 60为东八区的时区调整
	
	time_date = *localtime(&time_cnt);				// 使用localtime函数，将秒计数器转换为日期时间格式
	
	dev->time.year = time_date.tm_year + 1900;
	dev->time.month = time_date.tm_mon + 1;
	dev->time.day = time_date.tm_mday;
	dev->time.hour = time_date.tm_hour;
	dev->time.minute = time_date.tm_min;
	dev->time.second = time_date.tm_sec;
	dev->time.week = __rtc_calc_week(&dev->time);

	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	dev->time.hour = RTC_TimeStruct.RTC_Hours;
	dev->time.minute = RTC_TimeStruct.RTC_Minutes;
	dev->time.second = RTC_TimeStruct.RTC_Seconds;

	RTC_DateTypeDef RTC_DateStruct;
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
	dev->time.year = RTC_DateStruct.RTC_Year + 2000;
	dev->time.month = RTC_DateStruct.RTC_Month;
	dev->time.day = RTC_DateStruct.RTC_Date;
	dev->time.week = RTC_DateStruct.RTC_WeekDay;

	#endif

	return 0;
}

/******************************************************************************
 * @brief	去初始化RTC
 * @param	dev	:  RTCDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rtc_deinit(RTCDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}

/******************************************************************************
 * @brief	RTC根据日期计算星期
 * @param	time	:  RTCTime_t 结构体指针
 * @return	当前星期
 ******************************************************************************/
static uint8_t __rtc_calc_week(RTCTime_t *time)
{
    uint16_t year = time->year;
    uint8_t month = time->month;
    uint8_t day = time->day;
    uint8_t week;

    if (month < 3)
    {
        month += 12;
        year--;
    }

    /* 使用Zeller公式计算星期 */
    week = (day + 13 * (month + 1) / 5 + year + year / 4 - year / 100 + year / 400) % 7;

    /* 调整结果为1-7对应周一到周日 */
    week = (week + 5) % 7 + 1;

    return week;
}
