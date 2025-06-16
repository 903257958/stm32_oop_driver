#include "main.h"

static uint8_t uart1_tx_buf[256];
static uint8_t uart1_rx_buf[256];
uart_dev_t debug = {
    .config = {
        .uartx          = USART1,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_Pin_9,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_Pin_10,
        .tx_buf         = uart1_tx_buf,
        .rx_buf         = uart1_rx_buf,
        .tx_buf_size    = sizeof(uart1_tx_buf),
        .rx_buf_size    = sizeof(uart1_tx_buf),
        .rx_single_max  = 64
    }
};

void timer3_irq_callback(void *param)
{
    const char *name = (const char *)param;

	debug.printf("%s 500ms interrupt!\r\n", name);
}

timer_dev_t timer2 = {
    .config = {TIM2, 71, 49999, NULL, NULL, NULL, NULL}                 // 计数周期1us，定时周期50ms
};

timer_dev_t timer3 = {
    .config = {TIM3, 7199, 4999, timer3_irq_callback, "timer3", 0, 0}   // 计数周期100us，定时周期500ms
};

/* 测试：TIM2延时500ms打印，TIM3中断500ms打印 */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	delay_init(72);
    uart_init(&debug);
	timer_init(&timer2);
	timer_init(&timer3);
	
	while (1)
	{
		timer2.delay_ms(&timer2, 500);
        debug.printf("timer2 500ms delay!\r\n");
	}
}
