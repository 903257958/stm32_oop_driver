#include <string.h>
#include <stdio.h>
#include "esp.h"

#if FREERTOS
	#include "FreeRTOS.h"
	#include "task.h"
#endif

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

	#if !FREERTOS
	static void __esp_delay_ms(uint32_t ms)
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
	static void __esp_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif
	
#elif defined(STM32F40_41xxx)

	#if !FREERTOS
	static void __esp_delay_ms(uint32_t ms)
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
	static void __esp_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif
	
#endif

#define AT_TX_BUF	256		// AT指令发送数据缓冲区
#define AT_RX_BUF	2048	// AT指令接收数据缓冲区

/* 函数声明 */
static int __esp_send_cmd(ESPDev_t *pDev, const char *cmd, const char *ack, char *recv, uint16_t timeout);
static int __esp_reset(ESPDev_t *pDev);
static int __esp_connect_to_wifi(ESPDev_t *pDev, const char *ssid, const char *password);
static int __esp_deinit(ESPDev_t *pDev);

/******************************************************************************
 * @brief	初始化ESP
 * @param	pDev	:  ESPDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int esp_init(ESPDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 复位 */
	__esp_reset(pDev);
	
	/* 函数指针赋值 */
	pDev->send_cmd = __esp_send_cmd;
	pDev->reset = __esp_reset;
	pDev->connect_to_wifi = __esp_connect_to_wifi;
	pDev->deinit = __esp_deinit;
	
	pDev->initFlag = true;
	esp_debug.printf(&esp_debug, "\r\nESP init ok!\r\n");
	return 0;
}

/******************************************************************************
 * @brief	ESP发送AT指令并等待应答
 * @param	pDev	:  ESPDev_t 结构体指针
 * @param	cmd		:  要发送的AT指令
 * @param	ack		:  应答
 * @param	recv	:  接收到的数据
 * @param	timeout	:  超时时间（单位：毫秒）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_send_cmd(ESPDev_t *pDev, const char *cmd, const char *ack, char *recv, uint16_t timeout)
{
	uint32_t time = 0;
	int8_t val;
	char sendStr[AT_TX_BUF];
	static char recvStr[AT_RX_BUF];	// AT指令接收到的数据
	
	/* 初始化完整响应缓冲区 */
	memset(recvStr, 0, sizeof(recvStr));

	/* 发送AT指令 */
	sprintf(sendStr, "%s\r\n", cmd);
	esp_usart.send_string(&esp_usart, sendStr);

	/* 不需要判断应答 */
	if (ack == NULL)
	{
		__esp_delay_ms(timeout);
		return AT_RX_OK;
	}

	/* 开始计时并接收数据 */
	while (time < timeout)
	{
		if (esp_usart.recv_string_flag(&esp_usart))
		{
			strcpy(recvStr, esp_usart.recv_string(&esp_usart));
			esp_usart.dma_recv_enable(&esp_usart);	// 处理完数据再次开启DMA接收

			/* 判断是否收到ACK应答 */
			if (ack && strstr(recvStr, ack))
			{
				val = AT_RX_OK;			// 收到期望的应答
				break;
			}

			/* 检查失败信息 */
			if (strstr(recvStr, "FAIL") || strstr(recvStr, "ERROR"))
			{
				val = AT_RX_ERROR;		// 失败
				break;
			}
			
			/* 累加已等待的时间 */
			__esp_delay_ms(1);
			time++;
			if (time >= timeout)
			{
				val = AT_RX_TIMEOUT;	// 超时
				break;
			}
		}
	}

	/* 保存接收到的数据 */
    strcpy(recv, recvStr);

	return val;
}

/******************************************************************************
 * @brief	ESP复位
 * @param	pDev	:  ESPDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_reset(ESPDev_t *pDev)
{
	/* 恢复出厂设置 */
    if (__esp_send_cmd(pDev, "AT+RESTORE", NULL, NULL, 2000) != 0)
	{
		esp_debug.printf(&esp_debug, "restore err\r\n");
		return -1;
	}

	/* 关闭回显 */
	if (__esp_send_cmd(pDev, "ATE0", "OK", NULL, 1000) != 0)
	{
		esp_debug.printf(&esp_debug, "ate0 err\r\n");
		return -2;
	}

	/* 命令配置不存入flash */
	if (__esp_send_cmd(pDev, "AT+SYSSTORE=0", "OK", NULL, 1000) != 0)
	{
		esp_debug.printf(&esp_debug, "sysstore err\r\n");
		return -3;
	}

	return 0;
}

/******************************************************************************
 * @brief	ESP连接WiFi
 * @param	pDev		:  ESPDev_t 结构体指针
 * @param	ssid		:  WiFi名称
 * @param	password	:  WiFi密码
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_connect_to_wifi(ESPDev_t *pDev, const char *ssid, const char *password)
{
	char cmd[AT_TX_BUF];

	sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);

	return __esp_send_cmd(pDev, cmd, "OK", NULL, 8000);
}

/******************************************************************************
 * @brief	去初始化ESP
 * @param	pDev	:  ESPDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_deinit(ESPDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pDev->initFlag = false;
	
	return 0;
}
