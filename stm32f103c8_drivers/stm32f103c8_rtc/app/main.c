#include "main.h"

static uint8_t uart1_tx_buf[256];
static uint8_t uart1_rx_buf[256];
uart_dev_t debug = {
    .config = {
        .uartx          = USART1,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_Pin_9,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_Pin_10,
        .tx_buf         = uart1_tx_buf,
        .rx_buf         = uart1_rx_buf,
        .tx_buf_size    = sizeof(uart1_tx_buf),
        .rx_buf_size    = sizeof(uart1_tx_buf),
        .rx_single_max  = 64
    }
};
rtc_dev_t  rtc 	 = {.config = {2025, 1, 1, 23, 59, 55}};

int main(void)
{
	// rtc_time_t rtc_time = {2000, 1, 1, 23, 59, 55};

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	uart_init(&debug);
	rtc_init(&rtc);
	
	// rtc.set_time(&rtc, &rtc_time);

	while (1)
	{
		rtc.get_time(&rtc);

		debug.printf("\r\nDate:%04d-%02d-%02d-%d\r\n", rtc.time.year, rtc.time.month, rtc.time.day, rtc.time.week);
		debug.printf("Time:%02d:%02d:%02d\r\n", rtc.time.hour, rtc.time.minute, rtc.time.second);

		delay_ms(1000);
	}
}
