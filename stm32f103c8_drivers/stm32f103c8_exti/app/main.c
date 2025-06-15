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

void cnt_handler(void)
{
    static int cnt = 0;

    debug.printf("cnt = %d\r\n", ++cnt);
}

exti_dev_t cnt[3] = {
    {.config = {GPIOB, GPIO_Pin_0,  EXTI_Trigger_Falling,        0, 0, cnt_handler, NULL        }},
    {.config = {GPIOB, GPIO_Pin_1,  EXTI_Trigger_Rising,         0, 0, NULL,        cnt_handler }},
    {.config = {GPIOB, GPIO_Pin_10, EXTI_Trigger_Rising_Falling, 0, 0, cnt_handler, cnt_handler }}
};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(72);
	uart_init(&debug);
    exti_init(&cnt[0]);
    exti_init(&cnt[1]);
    exti_init(&cnt[2]);
	
	while (1)
	{
        
	}
}
