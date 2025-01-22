#include "main.h"

USARTDev_t usart1 = {.info = {USART1, 115200}};  // ESP8266调试串口
USARTDev_t usart2 = {.info = {USART2, 115200}};  // ESP8266通信串口
ESP8266Dev_t esp8266 = {.info = {&usart2, &usart1, "shouji", "thxd156369", "192.168.43.11", 8088}};
LEDDev_t led = {.info = {GPIOC, GPIO_Pin_13, GPIO_LEVEL_LOW}};

char gRecvData[100];

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	usart_init(&usart1);
	usart_init(&usart2);
	esp8266_init(&esp8266);
	led_init(&led);
	
	esp8266.config_tcp_passthrough(&esp8266);	// ESP8266配置TCP透传
	
	while (1)
	{
		if (esp8266.recv_data_flag(&esp8266))	// 透传模式下收到字符串数据包
		{
			/* 保存字符串数据包 */
			esp8266.recv_data(&esp8266, gRecvData);
			
			/* 打印字符串数据包 */
			esp8266.debug(&esp8266, "\r\nESP8266 recv: ");
			esp8266.debug(&esp8266, gRecvData);
			
			/* 判断控制LED指令 */
			if (strstr(gRecvData, "on"))
			{
				led.on(&led);
				esp8266.debug(&esp8266, "	-> led: on");
			}
			else if (strstr(gRecvData, "off"))
			{
				led.off(&led);
				esp8266.debug(&esp8266, "	-> led: off");
			}
			else if (strstr(gRecvData, "toggle"))
			{
				led.toggle(&led);
				esp8266.debug(&esp8266, "	-> led: toggle");
			}
		}
    }
}
