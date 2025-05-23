#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};

KeyDev_t key1 = {.config = {GPIOA, GPIO_Pin_0, GPIO_LEVEL_HIGH}};
KeyDev_t key2 = {.config = {GPIOA, GPIO_Pin_1, GPIO_LEVEL_LOW}};
KeyDev_t key3 = {.config = {GPIOA, GPIO_Pin_4, GPIO_LEVEL_LOW}};

int main(void)
{
    delay_init(168);
	uart_init(&debug);
	key_init(&key1);
	key_init(&key2);
    key_init(&key3);
	
	while(1)
	{
		if(key1.is_press(&key1))
		{
			debug.printf("key1 pressed!\r\n");
		}
		if(key2.is_press(&key2))
		{
			debug.printf("key2 pressed!\r\n");
		}
        if(key3.is_press(&key3))
		{
			debug.printf("key3 pressed!\r\n");
		}
	}
}
