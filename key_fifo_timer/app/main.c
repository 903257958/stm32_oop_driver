#include "main.h"

/* 定义2个LED设备 */
LEDDev_t led1 = {.info = {GPIOC, GPIO_Pin_13, GPIO_LEVEL_LOW}};
LEDDev_t led2 = {.info = {GPIOB, GPIO_Pin_8, GPIO_LEVEL_LOW}};

/* 定义3个按键设备 */
KeyDev_t key1 = {.info = {GPIOC, GPIO_Pin_1, GPIO_LEVEL_LOW, 1}};
KeyDev_t key2 = {.info = {GPIOC, GPIO_Pin_2, GPIO_LEVEL_LOW, 2}};
KeyDev_t key3 = {.info = {GPIOC, GPIO_Pin_3, GPIO_LEVEL_LOW, 3}};

/* 为按键轮询提供时基，只需给出哪个定时器即可，按键驱动内部会配置该定时器 */
TimerDev_t timerKeyTick = {.info.timx = TIM2};

int main(void)
{
	uint8_t keyVal;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	led_init(&led1);
	led_init(&led2);

	key_init(&key1);
	key_init(&key2);
	key_init(&key3);

	/* 检测按键并翻转LED，故意设置延时，按键状态会通过定时器中断存入环形缓冲区中，不会发生数据丢失 */
	while (1)
	{
		keyVal = key_get_val();

		if (keyVal == 1)
		{
			led1.toggle(&led1);
			delay_ms(500);
		}
		else if (keyVal == 2)
		{
			led2.toggle(&led2);
			delay_ms(500);
		}
		else if (keyVal == 3)
		{
			led1.toggle(&led1);
			led2.toggle(&led2);
			delay_ms(500);
		}
	}
}
