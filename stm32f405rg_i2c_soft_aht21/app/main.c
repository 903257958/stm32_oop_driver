#include "main.h"

uart_dev_t  debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
aht21_dev_t aht21 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(168);
	uart_init(&debug);
    aht21_init(&aht21);
    
	while (1)
	{
		aht21.get_data(&aht21);
        
        debug.printf("Temperature: %.1fC, Humidity: %.1f%%\r\n", aht21.data.temperature, aht21.data.humidity);
        
        delay_ms(1000);
	}
}
