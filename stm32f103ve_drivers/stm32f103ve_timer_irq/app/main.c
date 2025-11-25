#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_timer.h"
#include <stddef.h>

static uart_dev_t uart_debug;
static uint8_t uart_debug_tx_buf[256];
static uint8_t uart_debug_rx_buf[256];
static const uart_cfg_t uart_debug_cfg = {
    .uart_periph     = USART1,
    .baud            = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_10,
    .tx_buf          = uart_debug_tx_buf,
    .rx_buf          = uart_debug_rx_buf,
    .tx_buf_size     = sizeof(uart_debug_tx_buf),
    .rx_buf_size     = sizeof(uart_debug_rx_buf),
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static timer_dev_t timer2;
static const timer_cfg_t timer2_cfg = {  // 计数周期1us，定时周期50ms
    .timer_periph       = TIM2, 
    .psc                = 71, 
    .arr                = 49999,
    .pre_priority       = NULL, 
    .sub_priority       = NULL,
    .use_irq            = false
};

static timer_dev_t timer3;
static const timer_cfg_t timer3_cfg = {  // 计数周期100us，定时周期500ms
    .timer_periph       = TIM3, 
    .psc                = 7199, 
    .arr                = 4999, 
    .pre_priority       = 0,
    .sub_priority       = 0,
    .use_irq            = true
};

void timer3_irq_callback(void *param)
{
    const char *name = (const char *)param;

	uart_debug.ops->printf("%s 500ms interrupt!\r\n", name);
}

/* 测试：TIM2延时500ms打印，TIM3中断500ms打印 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    drv_uart_init(&uart_debug, &uart_debug_cfg);
	drv_timer_init(&timer2, &timer2_cfg);
	drv_timer_init(&timer3, &timer3_cfg);

    timer3.irq_callback = timer3_irq_callback;
    timer3.irq_callback_param = "timer3";

	while (1) {
        timer2.ops->delay_ms(&timer2, 500);
        uart_debug.ops->printf("timer2 500ms delay!\r\n");
	}
}
