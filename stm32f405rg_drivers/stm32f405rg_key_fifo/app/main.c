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
key_dev_t  key1  = {.config = {TIM2, GPIOC, GPIO_Pin_10, GPIO_LEVEL_LOW, 1}};

int main(void)
{
	uint8_t key_val;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay_init(168);
	uart_init(&debug);
	key_init(&key1);

	/* 检测按键，故意设置延时，按键状态会通过定时器中断存入环形缓冲区中，不会发生数据丢失 */
	while (1)
	{
		key_val = key_get_val();

		if (key_val == 1)
		{
			debug.printf("key1 pressed!\r\n");
            delay_ms(500);
		}
	}
}
