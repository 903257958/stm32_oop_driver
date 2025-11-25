#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_sr04.h"

static uart_dev_t uart_debug;
static uint8_t uart_debug_tx_buf[256];
static uint8_t uart_debug_rx_buf[256];
static const uart_cfg_t uart_debug_cfg = {
    .uart_periph     = USART0,
    .baud            = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_PIN_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_PIN_10,
    .tx_buf          = uart_debug_tx_buf,
    .rx_buf          = uart_debug_rx_buf,
    .tx_buf_size     = sizeof(uart_debug_tx_buf),
    .rx_buf_size     = sizeof(uart_debug_rx_buf),
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static sr04_dev_t sr04;
static const sr04_cfg_t sr04_cfg = {
    .timer_periph = TIMER1, 
    .ic_channel   = 1, 
    .trig_port    = GPIOA, 
    .trig_pin     = GPIO_PIN_0, 
    .echo_port    = GPIOA, 
    .echo_pin     = GPIO_PIN_1, 
    .pre_priority = 1,
    .sub_priority = 0, 
    .delay_us     = delay_us, 
    .delay_ms     = delay_ms
};

int main(void)
{
    float distance;

    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
	drv_sr04_init(&sr04, &sr04_cfg);
    
	while (1) {
        sr04.ops->get_distance(&sr04, &distance);
        uart_debug.ops->printf("Distance: %.2f cm\r\n", distance);
        delay_ms(100);
	}
}
