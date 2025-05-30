#include "esp8266.h"

/* ESP8266串口设备 */
uart_dev_t uart_esp8266;

/* 函数声明 */
static int8_t __esp8266_send_cmd(esp8266_dev_t *dev, char *cmd, char *res, char *recv_data);
static void __esp8266_clear(uart_rx_cb_t *uart_rx_cb);
static int8_t __esp8266_cmd_recv_check(uart_rx_cb_t *uart_rx_cb, char *res, char *recv_data);
static int8_t __esp8266_deinit(esp8266_dev_t *dev);

/******************************************************************************
 * @brief	ESP8266初始化
 * @param	dev	:	esp8266_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t esp8266_init(esp8266_dev_t *dev)
{
	if (!dev)
		return -1;

	/* 初始化UART */
	uart_esp8266.config.uartx = dev->config.uartx;
	uart_esp8266.config.baud = 115200;
	uart_esp8266.config.tx_port = dev->config.rx_port;
	uart_esp8266.config.tx_pin = dev->config.rx_pin;
	uart_esp8266.config.rx_port = dev->config.tx_port;
	uart_esp8266.config.rx_pin = dev->config.tx_pin;
	uart_init(&uart_esp8266);
	
	dev->init_flag = true;

	/* 函数指针赋值 */
	dev->send_cmd = __esp8266_send_cmd;
	dev->deinit = __esp8266_deinit;
    
    ESP8266_DEBUG(" \r\nESP8266 init...\r\n");
	
    /* 恢复出厂设置 */
    ESP8266_DEBUG("\r\n1. Restore\r\n");
	// while (__esp8266_send_cmd(dev, "AT+RESTORE\r\n", "ready", NULL) != 0)
	// {
    // 	ESP8266_DEBUG("retrying...\r\n");
	// 	ESP8266_DELAY_MS(500);
	// }
	ESP8266_DELAY_MS(300);

	/* 关闭回显 */
    ESP8266_DEBUG("\r\n2. Turn off echo display\r\n");
	while (__esp8266_send_cmd(dev, "ATE0\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
    
	/* AT测试 */
    ESP8266_DEBUG("\r\n3. AT test\r\n");
	while(__esp8266_send_cmd(dev, "AT\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
	
	/* 设置客户端模式 */
    ESP8266_DEBUG("\r\n4. Set client mode\r\n");
	while(__esp8266_send_cmd(dev, "AT+CWMODE=1\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
	
    /* 连接WiFi */
    ESP8266_DEBUG("\r\n5. Connect WiFi\r\n");
	while(__esp8266_send_cmd(dev, dev->config.wifi_inf0, "GOT IP", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
	
	ESP8266_DEBUG("\r\nESP8266 successfully initialized!\r\n");

	return 0;
}

/******************************************************************************
 * @brief	ESP8266发送命令
 * @param	dev			:	esp8266_dev_t 结构体指针
 * @param	cmd			:	命令
 * @param	res			:	需要检查的返回指令
 * @param	recv_data	:	接收到的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_send_cmd(esp8266_dev_t *dev, char *cmd, char *res, char *recv_data)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint8_t flag = 1;

	ESP9266_SEND_DEBUG(cmd);
	uart_esp8266.printf(cmd);

	// 清理旧数据
	#if UART1_ENABLE
	if (uart_esp8266.config.uartx == USART1)	__esp8266_clear(&uart1_rx_cb);
	#endif
	#if UART2_ENABLE
	if (uart_esp8266.config.uartx == USART2)	__esp8266_clear(&uart2_rx_cb);
	#endif

	#if UART1_ENABLE
	if (uart_esp8266.config.uartx == USART1)	flag = __esp8266_cmd_recv_check(&uart1_rx_cb, res, recv_data);
	#endif
	#if UART2_ENABLE
	if (uart_esp8266.config.uartx == USART2)	flag = __esp8266_cmd_recv_check(&uart2_rx_cb, res, recv_data);
	#endif
	#if UART3_ENABLE
	if (uart_esp8266.config.uartx == USART3)	flag = __esp8266_cmd_recv_check(&uart3_rx_cb, res, recv_data);
	#endif
	#if UART4_ENABLE
	if (uart_esp8266.config.uartx == UART4)		flag = __esp8266_cmd_recv_check(&uart4_rx_cb, res, recv_data);
	#endif
	#if UART5_ENABLE
	if (uart_esp8266.config.uartx == UART5)		flag = __esp8266_cmd_recv_check(&uart5_rx_cb, res, recv_data);
	#endif
	#if UART6_ENABLE
	if (uart_esp8266.config.uartx == USART6)	flag = __esp8266_cmd_recv_check(&uart6_rx_cb, res, recv_data);
	#endif

	return flag;
}

/******************************************************************************
 * @brief	ESP8266清除旧数据
 * @param	dev			:	esp8266_dev_t 结构体指针
 * @param	uart_rx_cb	:	uart_rx_cb_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static void __esp8266_clear(uart_rx_cb_t *uart_rx_cb)
{
	while (uart_rx_cb->index_out != uart_rx_cb->index_in)
	{
		uart_rx_cb->index_out++;
		if (uart_rx_cb->index_out == uart_rx_cb->index_end)
		{
			uart_rx_cb->index_out = &uart_rx_cb->index_buf[0];
		}
	}
}

/******************************************************************************
 * @brief	ESP8266命令返回检查
 * @param	uart_rx_cb	:	uart_rx_cb_t 结构体指针
 * @param	res			:	需要检查的返回指令
 * @param	recv_data	:	接收到的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_cmd_recv_check(uart_rx_cb_t *uart_rx_cb, char *res, char *recv_data)
{
	uint8_t flag = 1;
	uint16_t timeout = 1000;
	char *recv_buf;
	uint16_t recv_len;

	if (res == NULL)
	{
		return 0;
	}

	while (timeout--)
	{
		if (uart_rx_cb->index_out != uart_rx_cb->index_in)							
		{
			if (strstr((const char *)uart_rx_cb->index_out->start, res) != NULL)		
			{
				flag = 0;

				recv_len = uart_rx_cb->index_out->end - uart_rx_cb->index_out->start + 1;	// 接收数据长度

				*(uart_rx_cb->index_out->end + 1) = '\0';	// 末尾加上'\0'，确保可以作为字符串处理

				/* 保存接收到的数据 */
				recv_buf = (char *)uart_rx_cb->index_out->start;
				if (recv_data != NULL)
				{
					strncpy(recv_data, (char *)recv_buf, recv_len + 1);
				}
				ESP9266_RECV_DEBUG(recv_buf);

				uart_rx_cb->index_out++;
				if (uart_rx_cb->index_out == uart_rx_cb->index_end)
				{
					uart_rx_cb->index_out = &uart_rx_cb->index_buf[0];
				}
				break;
			}
		}
		ESP8266_DELAY_MS(10);
	}

	return flag;
}

/******************************************************************************
 * @brief	去初始化ESP8266
 * @param	dev	:	esp8266_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_deinit(esp8266_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}
