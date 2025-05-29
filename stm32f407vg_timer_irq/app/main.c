#include "main.h"

uart_dev_t 	debug  = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};

void timer2_irq_callback(void)
{
	debug.printf("timer2 irq 500ms!\r\n");
}

timer_dev_t timer1 = {.config = {TIM2, 83, 49999, NULL}};			        // 计数周期1us，定时周期50ms
timer_dev_t timer2 = {.config = {TIM3, 8399, 4999, timer2_irq_callback}};    // 计数周期100us，定时周期500ms

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	delay_init(168);
    uart_init(&debug);
	
	timer_init(&timer1);
	timer_init(&timer2);
	
	while (1)
	{
		timer1.delay_ms(&timer1, 500);
        debug.printf("timer1 delay 500ms!\r\n");
	}
}
