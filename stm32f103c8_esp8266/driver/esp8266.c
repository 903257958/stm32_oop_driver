#include "esp8266.h"

/* ESP8266串口设备 */
static uart_dev_t uart_esp8266;

/* 函数声明 */
static int8_t __esp8266_cmd_recv_check(uart_rx_cb_t *uart_rx_cb, char *res, char *recv_data);
static void __esp8266_clear(uart_rx_cb_t *uart_rx_cb);
static int8_t __esp8266_send_cmd(esp8266_dev_t *dev, char *cmd, char *res, char *recv_data);
static int8_t __esp8266_set_tcp_transparent(esp8266_dev_t *dev, const char *ip, uint16_t port);
static int8_t __esp8266_tcp_send_data(esp8266_dev_t *dev, char *send_data);
static int16_t __esp8266_tcp_recv_data(esp8266_dev_t *dev, char *recv_data, uint16_t buf_len);
static int8_t __esp8266_exit_tcp_transparent(esp8266_dev_t *dev);
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

	uint8_t retry_cnt;

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
	dev->set_tcp_transparent = __esp8266_set_tcp_transparent;
	dev->tcp_send_data = __esp8266_tcp_send_data;
	dev->tcp_recv_data = __esp8266_tcp_recv_data;
	dev->exit_tcp_transparent = __esp8266_exit_tcp_transparent;
	dev->deinit = __esp8266_deinit;
    
    ESP8266_DEBUG(" \r\nESP8266 init...\r\n");
	
    /* 1. 恢复出厂设置 */
    ESP8266_DEBUG("\r\n1. Restore\r\n");
	// for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	// {
	// 	if (__esp8266_send_cmd(dev, "AT+RESTORE\r\n", "OK", NULL) == 0) break;
	// 	ESP8266_DEBUG("retrying...\r\n");
	// 	ESP8266_DELAY_MS(500);
	// 	if (retry_cnt == 2)
	// 	{
	// 		ESP8266_DEBUG("Failed to restore!\r\n");
	// 		return -2;
	// 	}
	// }
	ESP8266_DELAY_MS(500);

	/* 2. 关闭回显 */
    ESP8266_DEBUG("\r\n2. Turn off echo display\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "ATE0\r\n", "OK", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("Failed to turn off echo!\r\n");
			return -3;
		}
	}
    
	/* 3. AT测试 */
    ESP8266_DEBUG("\r\n3. AT test\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "AT\r\n", "OK", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("AT test failed!\r\n");
			return -4;
		}
	}
	
	/* 4. 设置客户端模式 */
    ESP8266_DEBUG("\r\n4. Set client mode\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "AT+CWMODE=1\r\n", "OK", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("Failed to set client mode!\r\n");
			return -5;
		}
	}
	
    /* 5. 连接WiFi */
    ESP8266_DEBUG("\r\n5. Connect WiFi\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, dev->config.wifi_inf0, "GOT IP", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("WiFi connection failed!\r\n");
			return -6;
		}
	}
	
	ESP8266_DEBUG("\r\nESP8266 successfully initialized!\r\n");

	return 0;
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
	uint16_t timeout_10ms = 500;
	char *recv_buf;
	uint16_t recv_len;

	if (res == NULL)
	{
		return 0;
	}

	while (timeout_10ms--)
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
				ESP8266_RECV_DEBUG(recv_buf);

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
 * @brief	ESP8266清除旧数据
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

	ESP8266_SEND_DEBUG(cmd);
	uart_esp8266.printf(cmd);

	// 清理旧数据
	#if UART1_ENABLE
	if (uart_esp8266.config.uartx == USART1)	__esp8266_clear(&uart1_rx_cb);
	#endif
	#if UART2_ENABLE
	if (uart_esp8266.config.uartx == USART2)	__esp8266_clear(&uart2_rx_cb);
	#endif
	#if UART3_ENABLE
	if (uart_esp8266.config.uartx == USART3)	__esp8266_clear(&uart3_rx_cb);
	#endif
	#if UART4_ENABLE
	if (uart_esp8266.config.uartx == UART4)		__esp8266_clear(&uart4_rx_cb);
	#endif
	#if UART5_ENABLE
	if (uart_esp8266.config.uartx == UART5)		__esp8266_clear(&uart5_rx_cb);
	#endif
	#if UART6_ENABLE
	if (uart_esp8266.config.uartx == USART6)	__esp8266_clear(&uart6_rx_cb);
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
 * @brief	ESP8266设置TCP透传（作为客户端）
 * @param	dev     :   esp8266_dev_t 结构体指针
 * @param   ip      :   服务器IP地址字符串（如 "192.168.4.1"）
 * @param   port    :   服务器端口（如 8080）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_set_tcp_transparent(esp8266_dev_t *dev, const char *ip, uint16_t port)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t retry_cnt;
    char cmd[64];
	
    ESP8266_DEBUG("\r\nESP8266 sets TCP transparent mode...\r\n");

	/* 1. 设置单连接 */
    ESP8266_DEBUG("\r\n1. Set as single connection\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "AT+CIPMUX=0\r\n", "OK", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("Setting as single connection failed!\r\n");
			return -2;
		}
	}
    
    /* 2. 连接TCP服务器 */
    ESP8266_DEBUG("\r\n2. Connect to TCP server\r\n");
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if ((__esp8266_send_cmd(dev, cmd, "OK", NULL) == 0) || (__esp8266_send_cmd(dev, cmd, "CONNECTED", NULL) == 0))
			break;

		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("Failed to connect to TCP server!\r\n");
			return -3;
		}
	}

	/* 3. 设置透传模式 */
    ESP8266_DEBUG("\r\n3. Set transparent transmission mode\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "AT+CIPMODE=1\r\n", "OK", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("Failed to set transparent transmission mode!\r\n");
			return -4;
		}
	}
	
	/* 4. 开始透传 */
	ESP8266_DEBUG("\r\n4. Start transparent transmission\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "AT+CIPSEND\r\n", ">", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("Starting transparent transmission failed!\r\n");
			return -5;
		}
	}

    ESP8266_DEBUG("\r\nESP8266 successfully set TCP transparent mode!\r\n");

	return 0;
}

/******************************************************************************
 * @brief	ESP8266 TCP透传发送数据
 * @param	dev			:	esp8266_dev_t 结构体指针
 * @param	send_data	:	发送的数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __esp8266_tcp_send_data(esp8266_dev_t *dev, char *send_data)
{
	if (!dev || !dev->init_flag)
		return -1;

	ESP8266_SEND_DEBUG(send_data);
	uart_esp8266.printf(send_data);

	return 0;
}

/******************************************************************************
 * @brief   ESP8266 TCP透传接收数据
 * @param   dev         :   esp8266_dev_t 结构体指针
 * @param   recv_data	:   存储接收数据的缓冲区
 * @param   buf_len		:   缓冲区长度
 * @return  接收到的数据长度，<0 表示错误
 ******************************************************************************/
int16_t __esp8266_tcp_recv_data(esp8266_dev_t *dev, char *recv_data, uint16_t buf_len)
{
    if (!dev || !dev->init_flag)
        return -1;

	if (!recv_data || buf_len == 0)
		return -2;

	uart_rx_cb_t *rx_cb = NULL;
	uint16_t data_len;

    /* 选择对应的串口接收缓冲区 */
    #if UART1_ENABLE
    if (uart_esp8266.config.uartx == USART1) rx_cb = &uart1_rx_cb;
    #endif
    #if UART2_ENABLE
    if (uart_esp8266.config.uartx == USART2) rx_cb = &uart2_rx_cb;
    #endif
    #if UART3_ENABLE
    if (uart_esp8266.config.uartx == USART3) rx_cb = &uart3_rx_cb;
    #endif
    #if UART4_ENABLE
    if (uart_esp8266.config.uartx == UART4)  rx_cb = &uart4_rx_cb;
    #endif
    #if UART5_ENABLE
    if (uart_esp8266.config.uartx == UART5)  rx_cb = &uart5_rx_cb;
    #endif
    #if UART6_ENABLE
    if (uart_esp8266.config.uartx == USART6) rx_cb = &uart6_rx_cb;
    #endif

    if (!rx_cb || rx_cb->index_out == rx_cb->index_in)
        return 0;  // 无数据可读

    data_len = rx_cb->index_out->end - rx_cb->index_out->start + 1;
    if (data_len > buf_len - 1)
        data_len = buf_len - 1;

    memcpy(recv_data, rx_cb->index_out->start, data_len);
    recv_data[data_len] = '\0';  // 添加结束符

    ESP8266_RECV_DEBUG(recv_data);

    /* 移动读取指针 */
    rx_cb->index_out++;
    if (rx_cb->index_out == rx_cb->index_end)
	{
		rx_cb->index_out = &rx_cb->index_buf[0];
	}

    return data_len;
}

/******************************************************************************
 * @brief   ESP8266退出TCP透传模式
 * @param   dev	:	esp8266_dev_t 结构体指针
 * @return  0表示成功，其他表示失败
 ******************************************************************************/
static int8_t __esp8266_exit_tcp_transparent(esp8266_dev_t *dev)
{
    if (!dev || !dev->init_flag)
        return -1;

	uint8_t retry_cnt;

    ESP8266_DEBUG("\r\nESP8266 exits TCP transparent mode...\r\n");

    /* 1. 空闲等待1秒 */
	ESP8266_DEBUG("\r\n1. Idle waiting for 1 second\r\n");
    ESP8266_DELAY_MS(1000);

    /* 2. 发送退出透传符号 "+++"，不能有回车换行 */
	ESP8266_DEBUG("\r\n2. Exit transparent transmission\r\n");
    uart_esp8266.printf("+++");

    /* 3. 再等待1秒 */
	ESP8266_DEBUG("\r\n3. Idle waiting for 1 second\r\n");
    ESP8266_DELAY_MS(1000);

	/* 4. 断开TCP连接 */
	ESP8266_DEBUG("\r\n4. Disconnect TCP connection\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "AT+CIPCLOSE\r\n", "OK", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("Failed to disconnect TCP connection!\r\n");
			return -2;
		}
	}

	/* 5. AT测试 */
	ESP8266_DEBUG("\r\n5. AT test\r\n");
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++)
	{
		if (__esp8266_send_cmd(dev, "AT\r\n", "OK", NULL) == 0) break;
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
		if (retry_cnt == 2)
		{
			ESP8266_DEBUG("AT test failed!\r\n");
			return -3;
		}
	}

    ESP8266_DEBUG("ESP8266 successfully exited TCP transparent mode!\r\n");

    return 0;
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
