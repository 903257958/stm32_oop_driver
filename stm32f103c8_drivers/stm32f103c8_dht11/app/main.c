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
dht11_dev_t dht11 = {.config = {GPIOA, GPIO_Pin_1}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(72);
	uart_init(&debug);
	dht11_init(&dht11);
	
	while (1)
	{		
		dht11.get_data(&dht11);
		
        debug.printf("Temp: %d°C, Humi: %d%%\r\n", dht11.data.temperature, dht11.data.humidity);
        
        delay_ms(500);
	}
}
