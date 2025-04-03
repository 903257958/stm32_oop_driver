#include "delay.h"
#include "esp.h"

#define AT_TX_BUF	256		// AT指令发送数据缓冲区
#define AT_RX_BUF	2048	// AT指令接收数据缓冲区

/* 函数声明 */
static int __esp_send_cmd(ESPDev_t *dev, const char *cmd, const char *ack, char *recv, uint16_t timeout);
static int __esp_reset(ESPDev_t *dev);
static int __esp_connect_to_wifi(ESPDev_t *dev, const char *ssid, const char *password);
static int __esp_deinit(ESPDev_t *dev);

/******************************************************************************
 * @brief	初始化ESP
 * @param	dev	:  ESPDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/											
int esp_init(ESPDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 复位 */
	__esp_reset(dev);
	
	/* 函数指针赋值 */
	dev->send_cmd = __esp_send_cmd;
	dev->reset = __esp_reset;
	dev->connect_to_wifi = __esp_connect_to_wifi;
	dev->deinit = __esp_deinit;
	
	dev->init_flag = true;
	esp_debug.printf(&esp_debug, "\r\nESP init ok!\r\n");
	return 0;
}

/******************************************************************************
 * @brief	ESP发送AT指令并等待应答
 * @param	dev		:  ESPDev_t 结构体指针
 * @param	cmd		:  要发送的AT指令
 * @param	ack		:  应答
 * @param	recv	:  接收到的数据
 * @param	timeout	:  超时时间（单位：毫秒）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_send_cmd(ESPDev_t *dev, const char *cmd, const char *ack, char *recv, uint16_t timeout)
{
	uint32_t time = 0;
	int8_t val;
	char send_str[AT_TX_BUF];
	static char recv_str[AT_RX_BUF];	// AT指令接收到的数据
	
	/* 初始化完整响应缓冲区 */
	memset(recv_str, 0, sizeof(recv_str));

	/* 发送AT指令 */
	sprintf(send_str, "%s\r\n", cmd);
	esp_uart.send_string(&esp_uart, send_str);

	/* 不需要判断应答 */
	if (ack == NULL)
	{
		delay_ms(timeout);
		return AT_RX_OK;
	}

	/* 开始计时并接收数据 */
	while (time < timeout)
	{
		if (esp_uart.recv_string_flag(&esp_uart))
		{
			strcpy(recv_str, esp_uart.recv_string(&esp_uart));
			esp_uart.dma_recv_enable(&esp_uart);	// 处理完数据再次开启DMA接收

			/* 判断是否收到ACK应答 */
			if (ack && strstr(recv_str, ack))
			{
				val = AT_RX_OK;			// 收到期望的应答
				break;
			}

			/* 检查失败信息 */
			if (strstr(recv_str, "FAIL") || strstr(recv_str, "ERROR"))
			{
				val = AT_RX_ERROR;		// 失败
				break;
			}
		}
		/* 累加已等待的时间 */
		delay_ms(1);
		time++;
		if (time >= timeout)
		{
			val = AT_RX_TIMEOUT;	// 超时
			break;
		}
	}

	/* 保存接收到的数据 */
    strcpy(recv, recv_str);

	return val;
}

/******************************************************************************
 * @brief	ESP复位
 * @param	dev	:  ESPDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_reset(ESPDev_t *dev)
{
	/* 恢复出厂设置 */
    if (__esp_send_cmd(dev, "AT+RESTORE", NULL, NULL, 2000) != 0)
	{
		esp_debug.printf(&esp_debug, "restore err\r\n");
		return -1;
	}

	/* 关闭回显 */
	if (__esp_send_cmd(dev, "ATE0", "OK", NULL, 1000) != 0)
	{
		esp_debug.printf(&esp_debug, "ate0 err\r\n");
		return -2;
	}

	/* 命令配置不存入flash */
	if (__esp_send_cmd(dev, "AT+SYSSTORE=0", "OK", NULL, 1000) != 0)
	{
		esp_debug.printf(&esp_debug, "sysstore err\r\n");
		return -3;
	}

	return 0;
}

/******************************************************************************
 * @brief	ESP连接WiFi
 * @param	dev			:  ESPDev_t 结构体指针
 * @param	ssid		:  WiFi名称
 * @param	password	:  WiFi密码
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_connect_to_wifi(ESPDev_t *dev, const char *ssid, const char *password)
{
	char cmd[AT_TX_BUF];

	sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);

	return __esp_send_cmd(dev, cmd, "OK", NULL, 8000);
}

/******************************************************************************
 * @brief	去初始化ESP
 * @param	dev	:  ESPDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __esp_deinit(ESPDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;
	
	return 0;
}
