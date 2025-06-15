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

void forward_callback(void *param)
{
    static int forward_cnt = 0;
    const char *name = (const char *)param;

    debug.printf("%s forward cnt = %d\r\n", name, ++forward_cnt);
}

void reverse_callback(void *param)
{
    static int reverse_cnt = 0;
    const char *name = (const char *)param;

    debug.printf("%s reverse cnt = %d\r\n", name, ++reverse_cnt);
}

encoder_dev_t encoder[2] = {
    {.config = {GPIOB, GPIO_Pin_0,  GPIOB, GPIO_Pin_1,  forward_callback, "encoder0", reverse_callback, "encoder0"}},
    {.config = {GPIOB, GPIO_Pin_10, GPIOB, GPIO_Pin_11, forward_callback, "encoder1", reverse_callback, "encoder1"}}
};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(168);
	uart_init(&debug);
    encoder_init(&encoder[0]);
    encoder_init(&encoder[1]);
	
	while (1)
	{
        
	}
}
