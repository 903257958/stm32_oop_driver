#include "main.h"

/* 定义2个LED设备 */
LEDDev_t led1 = {.info = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};
LEDDev_t led2 = {.info = {GPIOB, GPIO_Pin_1, GPIO_LEVEL_LOW}};

/* 定义2个按键设备 */
KeyDev_t key1 = {.info = {TIM2, GPIOA, GPIO_Pin_0, GPIO_LEVEL_HIGH, 1}};
KeyDev_t key2 = {.info = {TIM2, GPIOA, GPIO_Pin_1, GPIO_LEVEL_LOW, 2}};

int main(void)
{
	uint8_t key_val;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay_init(168);

	led_init(&led1);
	led_init(&led2);

	key_init(&key1);
	key_init(&key2);

	/* 检测按键并翻转LED，故意设置延时，按键状态会通过定时器中断存入环形缓冲区中，不会发生数据丢失 */
	while (1)
	{
		key_val = key_get_val();

		if (key_val == 1)
		{
			led1.toggle(&led1);
			delay_ms(500);
		}
		else if (key_val == 2)
		{
			led2.toggle(&led2);
			delay_ms(500);
		}
	}
}
