#include "main.h"

#if !USE_FREERTOS
LEDDev_t led = {.info = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};

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
TimerDev_t timer_delay = {.info = {TIM4, 83, 49999, NULL}}; // 用于FreeRTOS下的微秒级延时，计数周期1us，定时周期50ms
LEDDev_t led = {.info = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};

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
