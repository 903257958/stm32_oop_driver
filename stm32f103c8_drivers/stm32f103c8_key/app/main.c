#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_key.h"

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

static key_dev_t key;
static const key_cfg_t key_cfg = {
    .delay_ms    = delay_ms,
    .port        = GPIOB,
    .pin         = GPIO_Pin_12,
    .press_level = GPIO_LEVEL_LOW
};

int main(void)
{
    bool status;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_key_init(&key, &key_cfg);

	while (1) {
        key.ops->get_status(&key, &status);
        if (status)
            uart_debug.ops->printf("key pressed!\r\n");
	}
}
