#include "esp8266.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	#if !FREERTOS
	static void __esp8266_dalay_ms(uint32_t ms)
	{
		while(ms--)
		{
			SysTick->LOAD = 72 * 1000;				// 设置定时器重装值
			SysTick->VAL = 0x00;					// 清空当前计数值
			SysTick->CTRL = 0x00000005;				// 设置时钟源为HCLK，启动定时器
			while(!(SysTick->CTRL & 0x00010000));	// 等待计数到0
			SysTick->CTRL = 0x00000004;				// 关闭定时器
		}
	}
	#else		
	static void __esp8266_dalay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif
	
#elif defined(STM32F40_41xxx)

	#if !FREERTOS
	static void __esp8266_dalay_ms(uint32_t ms)
	{
		while(ms--)
		{
			uint32_t temp;	    	 
			SysTick->LOAD = 1000 * 21; 					// 时间加载	  		 
			SysTick->VAL = 0x00;        				// 清空计数器
			SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; 	// 开始倒数 	 
			do
			{
				temp = SysTick->CTRL;
			}while((temp&0x01) && !(temp&(1<<16)));		// 等待时间到达   
			SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 	// 关闭计数器
			SysTick->VAL = 0X00;       					// 清空计数器 
		}
	}
	#else		
	static void __esp8266_dalay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif
	
#endif

static int __esp8266_send_cmd(ESP8266Dev_t *pDev, const char *cmd, const char *ack, uint16_t timeout);
static int __esp8266_restore(ESP8266Dev_t *pDev);
static int __esp8266_rst(ESP8266Dev_t *pDev);
static int __esp8266_set_mode(ESP8266Dev_t *pDev, uint8_t mode);
static int __esp8266_connect_to_wifi(ESP8266Dev_t *pDev);
static int __esp8266_set_connection_mode(ESP8266Dev_t *pDev, uint8_t mode);
static int __esp8266_start_tcp(ESP8266Dev_t *pDev);
static int __esp8266_enable_transparent_mode(ESP8266Dev_t *pDev);
static int __esp8266_debug(ESP8266Dev_t *pDev, const char *data);
static int __esp8266_send_data(ESP8266Dev_t *pDev, const char *data);
static bool __esp8266_recv_data_flag(ESP8266Dev_t *pDev);
static int __esp8266_recv_data(ESP8266Dev_t *pDev, char *recvStr);
static int __esp8266_config_tcp_passthrough(ESP8266Dev_t *pDev);
static int __esp8266_deinit(ESP8266Dev_t *pDev);

/******************************************************************************
 * @brief	初始化ESP8266
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int esp8266_init(ESP8266Dev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 恢复出厂设置 */
	__esp8266_debug(pDev, "\r\nInitializing ESP8266...\r\n");
	__esp8266_restore(pDev);
	
	/* 函数指针赋值 */
	pDev->send_cmd = __esp8266_send_cmd;
	pDev->restore = __esp8266_restore;
	pDev->rst = __esp8266_rst;
	pDev->set_mode = __esp8266_set_mode;
	pDev->connect_to_wifi = __esp8266_connect_to_wifi;
	pDev->set_connection_mode = __esp8266_set_connection_mode;
	pDev->start_tcp = __esp8266_start_tcp;
	pDev->enable_transparent_mode = __esp8266_enable_transparent_mode;
	pDev->debug = __esp8266_debug;
	pDev->send_data = __esp8266_send_data;
	pDev->recv_data_flag = __esp8266_recv_data_flag;
	pDev->recv_data = __esp8266_recv_data;
	pDev->config_tcp_passthrough = __esp8266_config_tcp_passthrough;
	pDev->deinit = __esp8266_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	ESP8266发送AT指令并等待应答
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @param	cmd		:  要发送的AT指令
 * @param	ack		:  应答
 * @param	timeout	:  超时时间（单位：毫秒）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_send_cmd(ESP8266Dev_t *pDev, const char *cmd, const char *ack, uint16_t timeout)
{
	uint32_t timeCount = 0;
	const char *successResponses[] = {"OK", "ALREADY CONNECTED", "WIFI CONNECTED"};
	char sendStr[BUFFER_LENGTH];
	char recvStr[BUFFER_LENGTH];
	
	while (timeCount < timeout)
	{
		/* 发送AT指令 */
		sprintf(sendStr, "%s\r\n", cmd);
		__esp8266_send_data(pDev, sendStr);
	
		/* 单次等待指令执行结果的时间 */
		uint32_t elapsedTime = 0;
		while (elapsedTime < 500) // 每次最多等待500ms
		{
			if (__esp8266_recv_data_flag(pDev))	// 判断是否接收到数据
			{
				__esp8266_recv_data(pDev, recvStr);	// 获取接收的数据
	
				/* 检查是否匹配应答或其他成功信息 */
				for (int i = 0; i < sizeof(successResponses) / sizeof(successResponses[0]); i++)
				{
					if (strstr(recvStr, successResponses[i]))
					{
						return 0; // 成功，匹配到应答或已连接
					}
				}
	
				/* 检查是否有明确的失败信息 */
				if (strstr(recvStr, "FAIL") || strstr(recvStr, "ERROR"))
				{
					return -1; // 失败
				}
	
				/* 如果接收到数据但不匹配，退出内层循环，重新发送指令 */
				break;
			}
	
			/* 延时10ms，累加已等待的时间 */
			__esp8266_dalay_ms(10);
			elapsedTime += 10;
			timeCount += 10;
	
			/* 检查是否已超时 */
			if (timeCount >= timeout)
			{
				return -1; // 超时返回
			}
		}
	}
	
	return -1; // 如果到达超时仍未匹配到应答，返回超时
}

/******************************************************************************
 * @brief	ESP8266恢复出厂设置
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_restore(ESP8266Dev_t *pDev)
{
    return __esp8266_send_cmd(pDev, "AT+RESTORE\r\n", "ready", 3000);
}

/******************************************************************************
 * @brief	ESP8266复位
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_rst(ESP8266Dev_t *pDev)
{
    return __esp8266_send_cmd(pDev, "AT+RST\r\n", "ready", 3000);
}

/******************************************************************************
 * @brief	ESP8266设置模式
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @param	mode	:  模式（1：STA	 2：AP  3：STA+AP）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_set_mode(ESP8266Dev_t *pDev, uint8_t mode)
{
    char cmd[20];
    sprintf(cmd, "AT+CWMODE=%d", mode);
    return __esp8266_send_cmd(pDev, cmd, "OK", 2000);
}

/******************************************************************************
 * @brief	ESP8266连接WiFi
 * @param	pDev		:  ESP8266Dev_t 结构体指针
 * @param	ssid		:  WiFi名称
 * @param	password	:  WiFi密码
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_connect_to_wifi(ESP8266Dev_t *pDev)
{
	char recvStr[BUFFER_LENGTH];
	
	/* 先检查当前的WiFi连接状态 */
	if (__esp8266_send_cmd(pDev, "AT+CWJAP?", "OK", 3000) == 0)
	{
		__esp8266_recv_data(pDev, recvStr);
		if (strstr(recvStr, pDev->info.ssid))
		{
			/* 如果已经连接到目标WiFi，直接返回成功 */
			__esp8266_send_data(pDev, "Already connected to WiFi.\r\n");
			return 0;
		}
	}
	
	/* 如果未连接到目标WiFi，发送连接指令 */
	char cmd[100];
	sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", pDev->info.ssid, pDev->info.password);
	
	/* 检查多种成功的可能响应 */
	if (__esp8266_send_cmd(pDev, cmd, "OK", 8000) == 0 ||
		__esp8266_send_cmd(pDev, cmd, "WIFI CONNECTED", 8000) == 0 ||
		__esp8266_send_cmd(pDev, cmd, "ALREADY CONNECTED", 8000) == 0)
	{
		__esp8266_send_data(pDev, "Successfully connected to WiFi.\r\n");
		return 0;  // 成功连接WiFi
	}
	else
	{
		__esp8266_send_data(pDev, "Failed to connect to WiFi.\r\n");
		return -1;  // 连接失败
	}
}

/******************************************************************************
 * @brief	ESP8266设置连接模式
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @param	mode	:  模式（0：单连接）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_set_connection_mode(ESP8266Dev_t *pDev, uint8_t mode)
{
    char cmd[100];
    sprintf(cmd, "AT+CIPMUX=%d", mode);
    return __esp8266_send_cmd(pDev, cmd, "OK", 5000);
}

/******************************************************************************
 * @brief	ESP8266启动TCP连接
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @param	ip		:  IP地址
 * @param	port	:  端口号
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_start_tcp(ESP8266Dev_t *pDev)
{
	char recvStr[BUFFER_LENGTH];
	char debugStr[BUFFER_LENGTH];
	char cmd[50];
	
	sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d", pDev->info.ip, pDev->info.port);
	
	/* 增加调试信息输出 */
	sprintf(debugStr, "Starting TCP connection to %s:%d...\r\n", pDev->info.ip, pDev->info.port);
	__esp8266_debug(pDev, debugStr);
	
	/* 检查返回的各种成功或失败信息 */
	int result = __esp8266_send_cmd(pDev, cmd, "OK", 8000);
	if (result == 0)
	{
		__esp8266_debug(pDev, "TCP connection started.\r\n");
		return 0;
	}
	else
	{
		/* 输出ESP8266返回的完整响应 */
		__esp8266_recv_data(pDev, recvStr);
		sprintf(debugStr, "TCP connection failed, response: %s\r\n", recvStr);
		__esp8266_debug(pDev, debugStr);
		return -1;
	}
}

/******************************************************************************
 * @brief	ESP8266启用透传模式
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_enable_transparent_mode(ESP8266Dev_t *pDev)
{
	char recvStr[BUFFER_LENGTH];
	char debugStr[BUFFER_LENGTH];
	
	/* 尝试设置透传模式 */
	if (__esp8266_send_cmd(pDev, "AT+CIPMODE=1", "OK", 1000) != 0)
	{
		/* 输出ESP8266返回的完整响应 */
		__esp8266_recv_data(pDev, recvStr);
		sprintf(debugStr, "Failed to set transparent mode, response: %s\r\n", recvStr);
		__esp8266_debug(pDev, debugStr);
		return -1; 
	}
	
	__esp8266_dalay_ms(100);
	
	/* 尝试启动透传模式的数据发送 */
	int result = __esp8266_send_cmd(pDev, "AT+CIPSEND", ">", 1000);
	if (result == 0)
	{
		__esp8266_debug(pDev, "Transparent mode enabled.\r\n");
		return 0;
	}
	else
	{
//		/* 输出ESP8266返回的完整响应 */
//		__esp8266_recv_data(pDev, recvStr);
//		sprintf(debugStr, "Failed to enable transparent mode, response: %s\r\n", recvStr);
//		__esp8266_debug(pDev, debugStr);
		return -1;
	}
}

/******************************************************************************
 * @brief	ESP8266调试串口打印数据
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @param	data	:  要打印的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_debug(ESP8266Dev_t *pDev, const char *data)
{
	pDev->info.pDebugUSART->printf(pDev->info.pDebugUSART, "%s", data);
	
    return 0; 
}

/******************************************************************************
 * @brief	ESP8266发送数据
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @param	data	:  要发送的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_send_data(ESP8266Dev_t *pDev, const char *data)
{
	pDev->info.pUSART->printf(pDev->info.pUSART, "%s", data);
	
    return 0; 
}

/******************************************************************************
 * @brief	ESP8266接收数据标志位
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @return	1, 表示接收到一个完整字符串数据包
 ******************************************************************************/
static bool __esp8266_recv_data_flag(ESP8266Dev_t *pDev)
{
    return pDev->info.pUSART->recv_string_flag(pDev->info.pUSART);
}

/******************************************************************************
 * @brief	ESP8266接收数据
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @param	recvStr	:  接收到的字符串数据包
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_recv_data(ESP8266Dev_t *pDev, char *recvStr)
{
	strcpy(recvStr, pDev->info.pUSART->recv_string(pDev->info.pUSART));
	
    return 0;
}

/******************************************************************************
 * @brief	ESP8266配置TCP透传
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_config_tcp_passthrough(ESP8266Dev_t *pDev)
{
	/* 设置为Station模式 */
	if (pDev->set_mode(pDev, 1) == 0)
	{
		__esp8266_debug(pDev, "Mode set to Station.\r\n");
	}
	else
	{
		__esp8266_debug(pDev, "Failed to set mode.\r\n");
		return -1;
	}
	
	/* 连接到WiFi */
	if (pDev->connect_to_wifi(pDev) == 0)
	{
		__esp8266_debug(pDev, "Connected to WiFi.\r\n");
	}
	else
	{
		__esp8266_debug(pDev, "Failed to connect to WiFi.\r\n");
		return -1;
	}
	
	/* 设置单连接 */
	if (pDev->set_connection_mode(pDev, 0) == 0) 
	{
		__esp8266_debug(pDev, "Set to Single Connection Mode.\r\n");
	}
	else
	{
		__esp8266_debug(pDev, "Failed to set Single Connection Mode.\r\n");
		return -1;
	}
	
	/* 启动TCP连接 */
	if (pDev->start_tcp(pDev) == 0)
	{
		__esp8266_debug(pDev, "TCP connection established.\r\n");
	}
	else
	{
		__esp8266_debug(pDev, "Failed to establish TCP connection.\r\n");
		return -1;
	}
	__esp8266_dalay_ms(1000);
	
	/* 启用透传模式 */
	if (pDev->enable_transparent_mode(pDev) == 0)
	{
		__esp8266_debug(pDev, "Transparent mode enabled.\r\n");
	}
//	else
//	{
//		__esp8266_debug(pDev, "Failed to enable transparent mode.\r\n");
//		return -1;
//	}
	
	__esp8266_dalay_ms(1000);
	__esp8266_send_data(pDev, "Hello ESP8266!\r\n");
	__esp8266_debug(pDev, "ESP8266 sent: Hello ESP8266!\r\n");
	__esp8266_dalay_ms(1000);
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化ESP8266
 * @param	pDev	:  ESP8266Dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp8266_deinit(ESP8266Dev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pDev->info.pUSART->deinit(pDev->info.pUSART);
	pDev->info.pDebugUSART->deinit(pDev->info.pDebugUSART);
	
	pDev->initFlag = false;
	
	return 0;
}
