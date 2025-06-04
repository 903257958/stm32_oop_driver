#include "main.h"

uart_dev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
key_dev_t  key1  = {.config = {TIM2, GPIOC, GPIO_Pin_10, GPIO_LEVEL_LOW, 1}};

int main(void)
{
	uint8_t key_val;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay_init(168);
	uart_init(&debug);
	key_init(&key1);

	/* 检测按键，故意设置延时，按键状态会通过定时器中断存入环形缓冲区中，不会发生数据丢失 */
	while (1)
	{
		key_val = key_get_val();

		if (key_val == 1)
		{
			debug.printf("key1 pressed!\r\n");
            delay_ms(500);
		}
	}
}
