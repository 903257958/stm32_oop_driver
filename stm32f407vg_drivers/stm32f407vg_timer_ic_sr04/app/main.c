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
        .rx_buf_size    = sizeof(uart1_rx_buf),
        .rx_single_max  = 64
    }
};

sr04_dev_t sr04 = {
    .config = {TIM4, 3, GPIOB, GPIO_Pin_0, GPIOB, GPIO_Pin_8}
};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(168);
	uart_init(&debug);
    sr04_init(&sr04);
	
	while (1)
	{
        sr04.get_distance(&sr04);

        debug.printf("Distance: %.2f cm\r\n", sr04.distance_cm);

        delay_ms(100);
	}
}
