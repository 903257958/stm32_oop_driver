#include "main.h"

uart_dev_t    debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
esp8266_dev_t esp8266;

int main(void)
{
    uint8_t month, day, hour, minute, second = 0;
    uint16_t year = 0;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(72);
    uart_init(&debug);
	esp8266_init(&esp8266);
    
    esp8266.get_weather(&esp8266);
    esp8266.get_beijing_time(&esp8266, &year, &month, &day, &hour, &minute, &second);
    
    debug.printf("%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);

	while (1)
	{

	}
}
