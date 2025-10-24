#include "bsp.h"

/* 硬件设备定义 */
static uint8_t uart0_tx_buf[256];
static uint8_t uart0_rx_buf[256];
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
        .rx_single_max  = 128
    }
};

static void cnt_handler(void)
{
    static int cnt = 0;

    debug.printf("cnt = %d\r\n", ++cnt);
}

exti_dev_t cnt[] = {
    { .config = { GPIOA, GPIO_PIN_0, EXTI_TRIG_FALLING, 0, 0, cnt_handler, NULL } },
    { .config = { GPIOA, GPIO_PIN_8, EXTI_TRIG_FALLING, 0, 0, cnt_handler, NULL } },
};

/**
 * @brief   初始化 BSP 硬件
 * @return  0 表示成功
 */
int bsp_init(void)
{
    uart_drv_init(&debug);
    for (int i = 0; i < sizeof(cnt) / sizeof(cnt[0]); i++)
        exti_drv_init(&cnt[i]);
    
    return 0;
}
