#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_rtc.h"

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

static rtc_dev_t rtc;
static const rtc_time_t time_set = {
    .year   = 2025,
    .month  = 11,
    .day    = 10,
    .hour   = 23,
    .minute = 59,
    .second = 56
};

int main(void)
{
    rtc_time_t time;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_rtc_init(&rtc);
    
    rtc.ops->set_time(&rtc, &time_set);

	while (1) {
        rtc.ops->get_time(&rtc, &time);

        uart_debug.ops->printf("\r\nDate: %04d-%02d-%02d-%d\r\n", 
                               time.year, time.month, time.day, time.week);
		uart_debug.ops->printf("Time: %02d:%02d:%02d\r\n", 
                               time.hour, time.minute, time.second);
        delay_ms(1000);
	}
}
