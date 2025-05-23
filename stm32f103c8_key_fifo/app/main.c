#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};

KeyDev_t key1 = {.config = {TIM2, GPIOB, GPIO_Pin_12, GPIO_LEVEL_LOW, 1}};
KeyDev_t key2 = {.config = {TIM2, GPIOA, GPIO_Pin_8, GPIO_LEVEL_LOW, 2}};
KeyDev_t key3 = {.config = {TIM2, GPIOB, GPIO_Pin_5, GPIO_LEVEL_LOW, 3}};

int main(void)
{
	uint8_t key_val;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay_init(72);
	uart_init(&debug);
	key_init(&key1);
	key_init(&key2);
    key_init(&key3);

	/* 检测按键，故意设置延时，按键状态会通过定时器中断存入环形缓冲区中，不会发生数据丢失 */
	while (1)
	{
		key_val = key_get_val();

		if (key_val == 1)
		{
			debug.printf("key1 pressed!\r\n");
            delay_ms(500);
		}
		else if (key_val == 2)
		{
			debug.printf("key2 pressed!\r\n");
			delay_ms(500);
		}
        else if (key_val == 3)
		{
			debug.printf("key3 pressed!\r\n");
			delay_ms(500);
		}
	}
}
