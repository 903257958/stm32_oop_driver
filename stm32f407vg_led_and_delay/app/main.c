#include "main.h"

#if !USE_FREERTOS
LEDDev_t led = {.config = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};

int main(void)
{
    delay_init(168);
	led_init(&led);
	
	while(1)
	{
		led.toggle(&led);
		delay_ms(500);
	}
}

#else
TimerDev_t timer_delay = {.config = {TIM4, 71, 49999, NULL}}; // 用于FreeRTOS下的微秒级延时，计数周期1us
LEDDev_t led = {.config = {GPIOC, GPIO_Pin_13, GPIO_LEVEL_HIGH}};

int main(void)
{
    delay_init(&timer_delay);
	led_init(&led);
	
	while(1)
	{
		led.toggle(&led);
		delay_ms(500);
	}
}

#endif
