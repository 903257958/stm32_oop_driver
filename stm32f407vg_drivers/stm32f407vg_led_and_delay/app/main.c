#include "main.h"

#if !USE_FREERTOS
led_dev_t led = {
    .config = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}
};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	led_init(&led);
	
	while (1)
	{
		led.toggle(&led);
		delay_ms(500);
	}
}

#else
timer_dev_t timer_delay = {
    .config = {TIM4, 83, 49999, NULL, NULL, NULL, NULL}  // 用于FreeRTOS下的微秒级延时，计数周期1us
};

led_dev_t led = {
    .config = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_HIGH}
};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(&timer_delay);
	led_init(&led);
	
	while (1)
	{
		led.toggle(&led);
		delay_ms(500);
	}
}

#endif
