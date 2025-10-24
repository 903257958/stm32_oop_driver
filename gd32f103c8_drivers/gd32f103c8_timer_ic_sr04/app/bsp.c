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

sr04_dev_t sr04 = {
    .config = { TIMER1, 0, GPIOB, GPIO_PIN_0, GPIOA, GPIO_PIN_0 }
};

/**
 * @brief   初始化 BSP 硬件
 * @return  0 表示成功
 */
int bsp_init(void)
{
    uart_drv_init(&debug);
    sr04_drv_init(&sr04);
	
    return 0;
}
