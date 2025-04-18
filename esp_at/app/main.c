#include "main.h"

UARTDev_t esp_debug = {.config = {	// esp32调试串口
	USART1, 115200,
	GPIOA, GPIO_Pin_9,
	GPIOA, GPIO_Pin_10
}};

UARTDev_t esp_uart = {.config = {	// esp32通信串口
	USART2, 115200,
	GPIOA, GPIO_Pin_2,
	GPIOA, GPIO_Pin_3
}};

ESPDev_t esp32;

char recv_str[1024];

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
    delay_init(168);
	uart_init(&esp_debug);
	uart_dma_init(&esp_uart);
	esp_init(&esp32);

	/* 设置客户端模式 */
	if (esp32.send_cmd(&esp32, "AT+CWMODE=1", "OK", NULL, 2000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Set station mode OK!\r\n");
	}

	/* 连接WiFi */
	if (esp32.connect_to_wifi(&esp32, "ChinaNet-97xUee", "?771116sqh#") == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Connect WiFi OK!\r\n");
	}

	/* 获取天气数据 */
	if (esp32.send_cmd(&esp32, "AT+HTTPCGET=\"https://api.seniverse.com/v3/weather/now.json?key=SwmhHrSHKGC4OXf6v&location=zhangjiakou&language=en&unit=c\"", "OK", recv_str, 8000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Get HTTP OK!\r\n");
	}
	esp_debug.send_string(&esp_debug, recv_str);	// 打印天气数据

	/* 使能SNTP 服务器，设置中国时区 */
	if (esp32.send_cmd(&esp32, "AT+CIPSNTPCFG=1,8,\"cn.ntp.org.cn\",\"ntp.sjtu.edu.cn\"", "OK", NULL, 2000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Set SNTP OK!\r\n");
	}

	/* 获取NTP时间 */
	if (esp32.send_cmd(&esp32, "AT+CIPSNTPTIME?", "OK", recv_str, 2000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Get NTP OK!\r\n");
	}
	esp_debug.send_string(&esp_debug, recv_str);	// 打印时间数据

	while (1)
	{
		
    }
}
