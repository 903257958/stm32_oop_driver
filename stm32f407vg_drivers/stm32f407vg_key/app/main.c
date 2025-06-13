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
key_dev_t  key1  = {.config = {GPIOA, GPIO_Pin_0, GPIO_LEVEL_HIGH}};
key_dev_t  key2  = {.config = {GPIOA, GPIO_Pin_1, GPIO_LEVEL_LOW}};
key_dev_t  key3  = {.config = {GPIOA, GPIO_Pin_4, GPIO_LEVEL_LOW}};

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	uart_init(&debug);
	key_init(&key1);
	key_init(&key2);
    key_init(&key3);
	
	while (1)
	{
		if(key1.is_press(&key1))
		{
			debug.printf("key1 pressed!\r\n");
		}
		if(key2.is_press(&key2))
		{
			debug.printf("key2 pressed!\r\n");
		}
        if(key3.is_press(&key3))
		{
			debug.printf("key3 pressed!\r\n");
		}
	}
}
