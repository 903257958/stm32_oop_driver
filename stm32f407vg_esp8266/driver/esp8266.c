#include "delay.h"
#include "esp8266.h"

#define REV_OK		0	// 接收完成标志
#define REV_WAIT	1	// 接收未完成标志

/* ESP8266接收缓冲区 */
uint8_t g_rx_buf[512];
uint16_t g_cnt = 0;

/* 日期时间结构体 */
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
}Time_t;

/* 函数声明 */
static int8_t __esp8266_clear(ESP8266Dev_t *dev);
static int8_t __esp8266_send_cmd(ESP8266Dev_t *dev, char *cmd, char *res);
static int8_t __esp8266_send_data(ESP8266Dev_t *dev, unsigned char *data, unsigned short len);
static uint8_t *__esp8266_get_ipd(ESP8266Dev_t *dev, unsigned short timeout);
static int8_t __esp8266_get_beijing_time(ESP8266Dev_t *dev, uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second);
static int8_t __esp8266_get_weather(ESP8266Dev_t *dev);
static int8_t __esp8266_deinit(ESP8266Dev_t *dev);
static void __esp8266_uart_init(unsigned int baud);
static void __esp8266_uart_send_string(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);
static int8_t __esp8266_wait_recv(void);
static int8_t __extract_time_from_buf(uint8_t *buf, Time_t *time);

/******************************************************************************
 * @brief	ESP8266初始化
 * @param	dev	:	ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t esp8266_init(ESP8266Dev_t *dev)
{
	__esp8266_uart_init(115200);
	
	dev->init_flag = true;

	/* 函数指针赋值 */
	dev->clear = __esp8266_clear;
	dev->send_cmd = __esp8266_send_cmd;
	dev->send_data = __esp8266_send_data;
	dev->get_ipd = __esp8266_get_ipd;
	dev->get_beijing_time = __esp8266_get_beijing_time;
	dev->get_weather = __esp8266_get_weather;
	dev->deinit = __esp8266_deinit;
    
    ESP8266_DEBUG(" \r\nESP8266 Init...\r\n");
	
    /* 恢复出厂设置 */
	while(__esp8266_send_cmd(dev, "AT+RESTORE\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("RESTORE OK\r\n");

	/* 关闭回显 */
	while(__esp8266_send_cmd(dev, "ATE0\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("ATE0 OK\r\n");
    
	/* AT测试 */
	while(__esp8266_send_cmd(dev, "AT\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("AT OK\r\n");
	
	/* 设置客户端模式 */
	while(__esp8266_send_cmd(dev, "AT+CWMODE=1\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("CWMODE OK\r\n");
	
    /* 连接WiFi */
	while(__esp8266_send_cmd(dev, ESP8266_WIFI_INFO, "GOT IP"))
		delay_ms(500);
    ESP8266_DEBUG("WiFi OK\r\n");
	
	ESP8266_DEBUG("ESP8266 Init OK\r\n");

	return 0;
}

/******************************************************************************
 * @brief	ESP8266清空缓存
 * @param	dev	:	ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_clear(ESP8266Dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	memset(g_rx_buf, 0, sizeof(g_rx_buf));
	g_cnt = 0;

	return 0;
}

/******************************************************************************
 * @brief	ESP8266发送命令
 * @param	cmd	:	命令
 * @param	res	:	需要检查的返回指令
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_send_cmd(ESP8266Dev_t *dev, char *cmd, char *res)
{
	if (!dev || !dev->init_flag)
		return -1;
		
	uint16_t timeout = 500;

	__esp8266_clear(dev);
	#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
    __esp8266_uart_send_string(USART2, (unsigned char *)cmd, strlen((const char *)cmd));
    #elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
    __esp8266_uart_send_string(USART3, (unsigned char *)cmd, strlen((const char *)cmd));
    #endif
	
	if (res == NULL)
	{
		return 0;
	}
	
	while(timeout--)
	{
		if(__esp8266_wait_recv() == REV_OK)					// 如果收到数据
		{
            if(strstr((const char *)g_rx_buf, res) != NULL)	// 如果检索到关键词
			{
				return 0;
			}
		}
		
		delay_ms(10);
	}
	
	return 1;
}

/******************************************************************************
 * @brief	ESP8266发送数据
 * @param	data	:	数据
 * @param	len		:	长度
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_send_data(ESP8266Dev_t *dev, unsigned char *data, unsigned short len)
{
	if (!dev || !dev->init_flag)
		return -1;

	char cmd_buf[32];
	
	__esp8266_clear(dev);							// 清空接收缓存
	sprintf(cmd_buf, "AT+CIPSEND=%d\r\n", len);		// 发送命令
	if(!__esp8266_send_cmd(dev, cmd_buf, ">"))		// 收到‘>’时可以发送数据
	{
		#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
        __esp8266_uart_send_string(USART2, data, len);		// 发送设备连接请求数据
        #elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
        __esp8266_uart_send_string(USART3, data, len);		// 发送设备连接请求数据
        #endif
	}

	return 0;
}

/******************************************************************************
 * @brief	ESP8266获取平台返回的数据，返回格式为 "+IPD,x:y" x是数据长度，y是数据内容
 * @param	timeout	:	等待的时间(乘以10ms)
 * @return	平台返回的原始数据
 ******************************************************************************/
static uint8_t *__esp8266_get_ipd(ESP8266Dev_t *dev, unsigned short timeout)
{
	char *ipd = NULL;
	
	do
	{
		if(__esp8266_wait_recv() == REV_OK)			// 如果接收完成
		{
			ipd = strstr((char *)g_rx_buf, "IPD,");	// 搜索“IPD”头
			if(ipd == NULL)							// 如果没找到，可能是IPD头的延迟，还是需要等待一会，但不会超过设定的时间
			{
				// ESP8266_DEBUG("\"IPD\" not found\r\n");
			}
			else
			{
				ipd = strchr(ipd, ':');				// 找到':'
				if(ipd != NULL)
				{
					ipd++;
					return (unsigned char *)(ipd);
				}
				else
					return NULL;
				
			}
		}
		delay_ms(5);										// 延时等待
	} while(timeout--);
	
	return NULL;	// 超时还未找到，返回空指针
}

/******************************************************************************
 * @brief	ESP8266获取北京时间
 * @param	dev		:  ESP8266Dev_t 结构体指针
 * @param	year	:  年
 * @param	month	:  月
 * @param	day		:  日
 * @param	hour	:  时
 * @param	minute	:  分
 * @param	second	:  秒
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_get_beijing_time(	ESP8266Dev_t *dev, 
                                            uint16_t *year, 
                                            uint8_t *month, 
                                            uint8_t *day, 
                                            uint8_t *hour, 
                                            uint8_t *minute, 
                                            uint8_t *second	)
{
	if (!dev || !dev->init_flag)
		return -1;

	Time_t time;
	char time_str[50];

	/* 恢复出厂设置 */
	while(__esp8266_send_cmd(dev, "AT+RESTORE\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("RESTORE OK\r\n");

	/* 关闭回显 */
	while(__esp8266_send_cmd(dev, "ATE0\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("ATE0 OK\r\n");
    
	/* AT测试 */
	while(__esp8266_send_cmd(dev, "AT\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("AT OK\r\n");
	
	/* 设置客户端模式 */
	while(__esp8266_send_cmd(dev, "AT+CWMODE=1\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("CWMODE OK\r\n");
	
    /* 连接WiFi */
	while(__esp8266_send_cmd(dev, ESP8266_WIFI_INFO, "GOT IP"))
		delay_ms(500);
    ESP8266_DEBUG("WiFi OK\r\n");

	/* 设置透传模式 */
    while(__esp8266_send_cmd(dev, "AT+CIPMODE=1\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("CIPMODE OK\r\n");
    
    /* 连接TCP服务器 */
    while(__esp8266_send_cmd(dev, "AT+CIPSTART=\"TCP\",\"www.beijing-time.org\",80\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("CIPSTART OK\r\n");
	
	/* 开始透传 */
	while(__esp8266_send_cmd(dev, "AT+CIPSEND\r\n", ">"))
	delay_ms(500);
	ESP8266_DEBUG("CIPSEND OK\r\n");

	/* 获取时间 */
	while(__esp8266_send_cmd(dev, "1\r\n", "GMT"))
	delay_ms(500);
	ESP8266_DEBUG("Time OK\r\n");

	// ESP8266_DEBUG((char *)g_rx_buf);

	if (__extract_time_from_buf(g_rx_buf, &time) == 0)
	{
		sprintf(time_str, "time: %04d-%02d-%02d %02d:%02d:%02d\n",
				time.year, time.month, time.day, time.hour, time.minute, time.second);

		*year = time.year;
		*month = time.month;
		*day = time.day;
		*hour = time.hour;
		*minute = time.minute;
		*second = time.second;
				
		ESP8266_DEBUG(time_str);
	}

	/* 退出透传 */
	__esp8266_send_cmd(dev, "+++", NULL);
	delay_ms(200);

	/* AT测试 */
	while(__esp8266_send_cmd(dev, "AT\r\n", "OK"))
	delay_ms(500);
	ESP8266_DEBUG("\r\nAT OK\r\n");

	return 0;
}

/******************************************************************************
 * @brief	ESP8266获取天气
 * @param	dev   :  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_get_weather(ESP8266Dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	/* 恢复出厂设置 */
	while(__esp8266_send_cmd(dev, "AT+RESTORE\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("RESTORE OK\r\n");

	/* 关闭回显 */
	while(__esp8266_send_cmd(dev, "ATE0\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("ATE0 OK\r\n");
    
	/* AT测试 */
	while(__esp8266_send_cmd(dev, "AT\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("AT OK\r\n");
	
	/* 设置客户端模式 */
	while(__esp8266_send_cmd(dev, "AT+CWMODE=1\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("CWMODE OK\r\n");
	
    /* 连接WiFi */
	while(__esp8266_send_cmd(dev, ESP8266_WIFI_INFO, "GOT IP"))
		delay_ms(500);
    ESP8266_DEBUG("WiFi OK\r\n");

	/* 设置透传模式 */
    while(__esp8266_send_cmd(dev, "AT+CIPMODE=1\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("CIPMODE OK\r\n");
    
    /* 连接TCP服务器 */
    while(__esp8266_send_cmd(dev, "AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80\r\n", "OK"))
		delay_ms(500);
    ESP8266_DEBUG("CIPSTART OK\r\n");

	/* 开始透传 */
	while(__esp8266_send_cmd(dev, "AT+CIPSEND\r\n", ">"))
	delay_ms(500);
	ESP8266_DEBUG("CIPSEND OK\r\n");

	/* 获取天气 */
	__esp8266_send_cmd(dev, "GET https://api.seniverse.com/v3/weather/now.json?key=SwmhHrSHKGC4OXf6v&location=shenyang&language=en&unit=c\r\n", NULL);
	delay_ms(500);
	ESP8266_DEBUG("Weather OK\r\n");

	ESP8266_DEBUG((char *)g_rx_buf);

	/* 退出透传 */
	__esp8266_send_cmd(dev, "+++", NULL);
	delay_ms(200);

	/* AT测试 */
	while(__esp8266_send_cmd(dev, "AT\r\n", "OK"))
	delay_ms(500);
	ESP8266_DEBUG("\r\nAT OK\r\n");

	return 0;
}

/******************************************************************************
 * @brief	去初始化ESP8266
 * @param	dev   :  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_deinit(ESP8266Dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}

/******************************************************************************
 * @brief	ESP8266串口初始化
 * @param	baud	:	波特率
 * @return	无
 ******************************************************************************/
static void __esp8266_uart_init(unsigned int baud)
{
    #if defined(STM32F10X_MD) || defined(STM32F10X_HD)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	GPIO_InitTypeDef gpio_initstruct;
    gpio_initstruct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_initstruct.GPIO_Pin = GPIO_Pin_2;
	gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_initstruct);
	
	gpio_initstruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_initstruct.GPIO_Pin = GPIO_Pin_3;
	gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_initstruct);
    
    USART_InitTypeDef usart_initstruct;
	usart_initstruct.USART_BaudRate = baud;
	usart_initstruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 无硬件流控
	usart_initstruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 接收和发送
	usart_initstruct.USART_Parity = USART_Parity_No;								// 无校验
	usart_initstruct.USART_StopBits = USART_StopBits_1;								// 1位停止位
	usart_initstruct.USART_WordLength = USART_WordLength_8b;						// 8位数据位
	USART_Init(USART2, &usart_initstruct);
	
	USART_Cmd(USART2, ENABLE);														// 使能串口
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);									// 使能接收中断
	
    NVIC_InitTypeDef nvic_initstruct;
	nvic_initstruct.NVIC_IRQChannel = USART2_IRQn;
	nvic_initstruct.NVIC_IRQChannelCmd = ENABLE;
	nvic_initstruct.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_initstruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_initstruct);
    
    #elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    
    USART_InitTypeDef usart_initstruct;
	usart_initstruct.USART_BaudRate = baud;
	usart_initstruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 无硬件流控
	usart_initstruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 接收和发送
	usart_initstruct.USART_Parity = USART_Parity_No;								// 无校验
	usart_initstruct.USART_StopBits = USART_StopBits_1;								// 1位停止位
	usart_initstruct.USART_WordLength = USART_WordLength_8b;						// 8位数据位
	USART_Init(USART3, &usart_initstruct);
	
	USART_Cmd(USART3, ENABLE);														// 使能串口
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);									// 使能接收中断
	
    NVIC_InitTypeDef nvic_initstruct;
	nvic_initstruct.NVIC_IRQChannel = USART3_IRQn;
	nvic_initstruct.NVIC_IRQChannelCmd = ENABLE;
	nvic_initstruct.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_initstruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_initstruct);
    
	#endif
}

/******************************************************************************
 * @brief	ESP8266串口发送数据
 * @param	USARTx	:	串口外设
 * @param	str		:	字符串
 * @param	len		:	长度
 * @return	无
 ******************************************************************************/
static void __esp8266_uart_send_string(USART_TypeDef *USARTx, unsigned char *str, unsigned short len)
{
	unsigned short count = 0;
	
	for(; count < len; count++)
	{
		USART_SendData(USARTx, *str++);									// 发送数据
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);		// 等待发送完成
	}
}

/******************************************************************************
 * @brief	ESP8266等待接收完成（循环调用检测是否接收完成）
 * @param	无
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_wait_recv(void)
{
	static uint16_t cnt_pre = 0;
	
	if(g_cnt == 0) 					// 如果接收计数为0 则说明没有处于接收数据中，所以直接跳出，结束函数
		return REV_WAIT;
		
	if(g_cnt == cnt_pre)			// 如果上一次的值和这次相同，则说明接收完毕
	{
		g_cnt = 0;					// 清0接收计数
			
		return REV_OK;				// 返回接收完成标志
	}
		
	cnt_pre = g_cnt;				// 置为相同
	
	return REV_WAIT;				// 返回接收未完成标志
}

/******************************************************************************
 * @brief	从缓冲区中提取时间
 * @param	buf	:	缓存区
 * @param	time	:	时间，DateTime_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __extract_time_from_buf(uint8_t *buf, Time_t *time)
{
    const char *date_prefix = "Date:";
    const uint8_t days_in_month[12] = {
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};
    const char *months[] = {
		"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    char *date_line = strstr((char*)buf, date_prefix);
    if (!date_line) return -1;

    char weekday[4], month[4];
    int year, day, hour, minute, second;
	uint8_t i;

    /* 例如：Date: Tue, 06 May 2025 12:38:08 GMT */
    uint8_t ret = sscanf(date_line, "Date: %3s, %d %3s %d %d:%d:%d",
                     weekday, &day, month, &year, &hour, &minute, &second);
    if (ret != 7) return -2;

    /* 月份字符串转数字 */
    uint8_t month_num = 0;
    for (i = 0; i < 12; ++i)
	{
        if (strncmp(month, months[i], 3) == 0)
		{
            month_num = i + 1;
            break;
        }
    }
    if (month_num == 0) return -3;

    /* 设置初始时间 */
    time->year = year;
    time->month = month_num;
    time->day = day;
    time->hour = hour;
    time->minute = minute;
    time->second = second;

    /* 加8小时处理跨天 */
    time->hour += 8;
    while (time->hour >= 24)
	{
        time->hour -= 24;
        time->day++;

        int max_day = days_in_month[time->month - 1];
        if (time->month == 2 && ((time->year % 4 == 0 && time->year % 100 != 0) || (time->year % 400 == 0)))
		{
            max_day = 29;
        }

        if (time->day > max_day)
		{
            time->day = 1;
            time->month++;
            if (time->month > 12)
			{
                time->month = 1;
                time->year++;
            }
        }
    }

    return 0;
}

/******************************************************************************
 * @brief	ESP8266串口中断回调函数
 * @param	无
 * @return	无
 ******************************************************************************/
void esp8266_uart_irq_callback(void)
{
	if(g_cnt >= sizeof(g_rx_buf))
	{
		g_cnt = 0; // 防止串口被刷爆
	}
	#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
    g_rx_buf[g_cnt++] = USART2->DR;
    #elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
    g_rx_buf[g_cnt++] = USART3->DR;
    #endif
}

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
/******************************************************************************
 * @brief	USART2串口中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // 接收中断
	{
		esp8266_uart_irq_callback();
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
	}
}
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
/******************************************************************************
 * @brief	USART3串口中断函数
 * @param	无
 * @return	无
 ******************************************************************************/
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // 接收中断
	{
		esp8266_uart_irq_callback();
		USART_ClearFlag(USART3, USART_FLAG_RXNE);
	}
}
#endif
