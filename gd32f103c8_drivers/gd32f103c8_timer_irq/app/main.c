#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_timer.h"
#include <stddef.h>

static uart_dev_t uart_debug;
static uint8_t uart_debug_tx_buf[512];
static uint8_t uart_debug_rx_buf[512];
static const uart_cfg_t uart_debug_cfg = {
    .uart_periph     = USART0,
    .baudrate        = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_PIN_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_PIN_10,
    .tx_buf          = uart_debug_tx_buf,
    .rx_buf          = uart_debug_rx_buf,
    .tx_buf_size     = sizeof(uart_debug_tx_buf),
    .rx_buf_size     = sizeof(uart_debug_rx_buf),
    .rx_single_max   = 256,
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static timer_dev_t timer1;
static const timer_cfg_t timer1_cfg = {  // 计数周期1us，定时周期50ms
    .timer_periph = TIMER1, 
    .psc          = 107, 
    .arr          = 49999,
    .pre_priority = NULL, 
    .sub_priority = NULL,
    .use_irq      = false
};

static timer_dev_t timer2;
static const timer_cfg_t timer2_cfg = {  // 计数周期100us，定时周期500ms
    .timer_periph = TIMER2, 
    .psc          = 10799, 
    .arr          = 4999,
    .pre_priority = 0, 
    .sub_priority = 0,
    .use_irq      = true
};

void timer2_irq_callback(void *param)
{
    const char *name = (const char *)param;

	uart_debug.ops->printf(&uart_debug, "%s 500ms interrupt!\r\n", name);
}

/* 测试：TIMER1延时500ms打印，TIMER2中断500ms打印 */
int main(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
	drv_timer_init(&timer1, &timer1_cfg);
	drv_timer_init(&timer2, &timer2_cfg);

    timer2.irq_callback = timer2_irq_callback;
    timer2.irq_callback_param = "timer2";
    
	while (1) {
        timer1.ops->delay_ms(&timer1, 500);
        uart_debug.ops->printf(&uart_debug, "timer1 500ms delay!\r\n");
	}
}
