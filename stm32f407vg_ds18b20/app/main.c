#include "main.h"

uart_dev_t 	  debug   = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
ds18b20_dev_t ds18b20 = {.config = {GPIOB, GPIO_Pin_0}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	uart_init(&debug);
	ds18b20_init(&ds18b20);
	
	while(1)
	{		
		ds18b20.get_temperature(&ds18b20);
		
        debug.printf("Temp: %.2fC\r\n", ds18b20.temperature);
        
        delay_ms(500);
	}
}
