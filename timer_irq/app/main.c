#include "main.h"

LEDDev_t led1 = {.info = {GPIOB, GPIO_Pin_1, GPIO_LEVEL_LOW}};
LEDDev_t led2 = {.info = {GPIOB, GPIO_Pin_2, GPIO_LEVEL_LOW}};

void led2_toggle(void)
{
	led2.toggle(&led2);
}

TimerDev_t timer1 = {.info = {TIM2, 83, 49999, NULL}};			// 计数周期1us，定时周期50ms
TimerDev_t timer2 = {.info = {TIM3, 8399, 4999, led2_toggle}};	// 计数周期100us，定时周期500ms

/* led1使用定时器延时闪烁，led2使用定时器中断闪烁 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	delay_init(168);

	led_init(&led1);
	led_init(&led2);
	
	timer_init(&timer1);
	timer_init(&timer2);
	
	while (1)
	{
		led1.toggle(&led1);
		timer1.delay_ms(&timer1, 500);
	}
}
