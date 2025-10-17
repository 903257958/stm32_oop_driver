#include "bsp.h"

/* 硬件设备定义 */
static uint8_t uart0_tx_buf[2048];
static uint8_t uart0_rx_buf[2048];
uart_dev_t debug = {
    .config = {
        .uartx          = USART0,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_PIN_9,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_PIN_10,
        .tx_buf         = uart0_tx_buf,
        .rx_buf         = uart0_rx_buf,
        .tx_buf_size    = sizeof(uart0_tx_buf),
        .rx_buf_size    = sizeof(uart0_rx_buf),
        .rx_single_max  = 512
    }
};

void timer2_irq_callback(void *arg)
{
    const char *name = (const char *)arg;
	debug.printf("%s 500ms interrupt!\r\n", name);
}

timer_dev_t timer1 = {
    .config = {TIMER1, 107, 49999, NULL, NULL, NULL, NULL}                 // 计数周期1us，定时周期50ms
};

timer_dev_t timer2 = {
    .config = {TIMER2, 10799, 4999, timer2_irq_callback, "timer2", 0, 0}   // 计数周期100us，定时周期500ms
};

/**
 * @brief   初始化 BSP 硬件
 * @return  0 表示成功
 */
int bsp_init(void)
{
    uart_init(&debug);
    timer_drv_init(&timer1);
	timer_drv_init(&timer2);
	
    return 0;
}
