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
key_dev_t  key1  = {.config = {TIM2, GPIOA, GPIO_Pin_0, GPIO_LEVEL_HIGH, 1}};
key_dev_t  key2  = {.config = {TIM2, GPIOA, GPIO_Pin_1, GPIO_LEVEL_LOW, 2}};
key_dev_t  key3  = {.config = {TIM2, GPIOA, GPIO_Pin_4, GPIO_LEVEL_LOW, 3}};

int main(void)
{
	uint8_t key_val;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
	uart_init(&debug);
	key_init(&key1);
	key_init(&key2);
    key_init(&key3);

	/* 检测按键，故意设置延时，按键状态会通过定时器中断存入环形缓冲区中，不会发生数据丢失 */
	while (1)
	{
		key_val = key_get_val();

		if (key_val == 1)
		{
			debug.printf("key1 pressed!\r\n");
            delay_ms(500);
		}
		else if (key_val == 2)
		{
			debug.printf("key2 pressed!\r\n");
			delay_ms(500);
		}
        else if (key_val == 3)
		{
			debug.printf("key3 pressed!\r\n");
			delay_ms(500);
		}
	}
}
