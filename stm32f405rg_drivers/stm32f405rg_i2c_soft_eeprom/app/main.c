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
eeprom_dev_t at24c02 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
    uint16_t i;
    uint8_t read_data[256];

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	uart_init(&debug);
	eeprom_init(&at24c02);

    for (i = 0; i < 256; i++)
    {
        at24c02.write_byte(&at24c02, i, i);
        delay_ms(2);
    }

    at24c02.read_data(&at24c02, 0, 256, read_data);

    for (i = 0; i < 256; i++)
    {
        debug.printf("%d ", read_data[i]);
    }
    debug.printf("\r\n");
    
	while (1)
	{
        
	}
}
