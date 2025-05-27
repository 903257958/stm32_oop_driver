#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
DHT11Dev_t dht11 = {.config = {GPIOA, GPIO_Pin_1}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(72);
	uart_init(&debug);
	dht11_init(&dht11);
	
	while(1)
	{		
		dht11.get_data(&dht11);
		
        debug.printf("Temp: %dC, Humi: %d%%\r\n", dht11.data.temperature, dht11.data.humidity);
        
        delay_ms(500);
	}
}
