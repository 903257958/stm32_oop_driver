#include "main.h"

static uint8_t uart1_tx_buf[2048];
static uint8_t uart1_rx_buf[2048];
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
        .rx_single_max  = 512
    }
};
aht21_dev_t aht21 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	uart_init(&debug);
    aht21_init(&aht21);
    
	while (1)
	{
		aht21.get_data(&aht21);
        
        debug.printf("Temperature: %.1fC, Humidity: %.1f%%\r\n", aht21.data.temperature, aht21.data.humidity);
        
        delay_ms(1000);
	}
}
