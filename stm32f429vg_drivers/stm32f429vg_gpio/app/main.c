#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_gpio.h"

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

static gpio_dev_t gpio[4];
static const gpio_cfg_t gpio_cfg[4] = {
    [0] = {
        .port = GPIOD,
        .pin  = GPIO_Pin_8,
        .mode = OUT_PP
    },
    [1] = {
        .port = GPIOB,
        .pin  = GPIO_Pin_15,
        .mode = OUT_PP
    },
    [2] = {
        .port = GPIOE,
        .pin  = GPIO_Pin_14,
        .mode = IN_PU
    },
    [3] = {
        .port = GPIOE,
        .pin  = GPIO_Pin_15,
        .mode = IN_PU
    }
};

int main(void)
{
    uint8_t level;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    for (uint8_t i = 0; i < 4; i++)
        drv_gpio_init(&gpio[i], &gpio_cfg[i]);
    
    gpio[0].ops->reset(&gpio[0]);
    gpio[1].ops->reset(&gpio[1]);

	while (1) {
        gpio[2].ops->read(&gpio[2], &level);
        uart_debug.ops->printf("gpio[2]: %d\r\n", level);

        gpio[3].ops->read(&gpio[3], &level);
        uart_debug.ops->printf("gpio[3]: %d\r\n", level);
        delay_ms(1000);
	}
}
