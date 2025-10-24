#include "bsp.h"

/* 硬件设备定义 */
static uint8_t uart0_tx_buf[2048];
static uint8_t uart0_rx_buf[2048];
uart_dev_t uart0 = {
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

static uint8_t uart1_tx_buf[2048];
static uint8_t uart1_rx_buf[2048];
uart_dev_t uart1 = {
    .config = {
        .uartx          = USART1,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_PIN_2,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_PIN_3,
        .tx_buf         = uart1_tx_buf,
        .rx_buf         = uart1_rx_buf,
        .tx_buf_size    = sizeof(uart1_tx_buf),
        .rx_buf_size    = sizeof(uart1_tx_buf),
        .rx_single_max  = 512
    }
};

static uint8_t uart2_tx_buf[2048];
static uint8_t uart2_rx_buf[2048];
uart_dev_t uart2 = {
    .config = {
        .uartx          = USART2,
        .baud           = 115200,
        .tx_port        = GPIOB,
        .tx_pin         = GPIO_PIN_10,
        .rx_port        = GPIOB,
        .rx_pin         = GPIO_PIN_11,
        .tx_buf         = uart2_tx_buf,
        .rx_buf         = uart2_rx_buf,
        .tx_buf_size    = sizeof(uart2_tx_buf),
        .rx_buf_size    = sizeof(uart2_rx_buf),
        .rx_single_max  = 512
    }
};

/**
 * @brief   初始化 BSP 硬件
 * @return  0 表示成功
 */
int bsp_init(void)
{
    uart_drv_init(&uart0);
	uart_drv_init(&uart1);
	uart_drv_init(&uart2);
    
    return 0;
}
