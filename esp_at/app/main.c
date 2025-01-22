#include "main.h"

USARTDev_t esp_debug = {.info = {	// ESP8266调试串口
	USART1, 115200,
	GPIOA, GPIO_Pin_9,
	GPIOA, GPIO_Pin_10
}};

USARTDev_t esp_usart = {.info = {	// ESP8266通信串口
	USART2, 115200,
	GPIOA, GPIO_Pin_2,
	GPIOA, GPIO_Pin_3
}};

ESPDev_t esp8266;

char recvStr[1024];

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	usart_init(&esp_debug);
	usart_dma_init(&esp_usart);     // ESP8266通信串口必须配置为DMA模式
	esp_init(&esp8266);

	/* 设置客户端模式 */
	if (esp8266.send_cmd(&esp8266, "AT+CWMODE=1", "OK", NULL, 2000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Set station mode OK!\r\n");
	}

	/* 连接WiFi（这里需要替换为你自己的WiFi名称与密码） */
	if (esp8266.connect_to_wifi(&esp8266, "your_wifi_ssid", "your_wifi_password") == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Connect WiFi OK!\r\n");
	}

	/* 获取天气数据（这里需要你注册心知天气并且替换为你自己的密钥） */
	if (esp8266.send_cmd(&esp8266, "AT+HTTPCGET=\"https://api.seniverse.com/v3/weather/now.json?key=your_private_key&location=beijing&language=en&unit=c\"", "OK", recvStr, 8000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Get HTTP OK!\r\n");
	}
	esp_debug.send_string(&esp_debug, recvStr);	// 打印天气数据

	/* 使能SNTP 服务器，设置中国时区 */
	if (esp8266.send_cmd(&esp8266, "AT+CIPSNTPCFG=1,8,\"cn.ntp.org.cn\",\"ntp.sjtu.edu.cn\"", "OK", NULL, 2000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Set SNTP OK!\r\n");
	}

	/* 获取NTP时间 */
	if (esp8266.send_cmd(&esp8266, "AT+CIPSNTPTIME?", "OK", recvStr, 2000) == AT_RX_OK)
	{
		esp_debug.printf(&esp_debug, "Get NTP OK!\r\n");
	}
	esp_debug.send_string(&esp_debug, recvStr);	// 打印时间数据

	while (1)
	{
		
    }
}
