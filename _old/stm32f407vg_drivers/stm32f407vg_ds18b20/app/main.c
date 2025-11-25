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
ds18b20_dev_t ds18b20 = {.config = {GPIOB, GPIO_Pin_0}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	uart_init(&debug);
	ds18b20_init(&ds18b20);
	
	while(1)
	{		
		ds18b20.get_temperature(&ds18b20);
		
        debug.printf("Temp: %.2f°C\r\n", ds18b20.temperature);
        
        delay_ms(500);
	}
}
