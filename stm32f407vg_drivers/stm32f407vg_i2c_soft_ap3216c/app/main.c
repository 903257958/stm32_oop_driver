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
ap3216c_dev_t ap3216c = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	uart_init(&debug);
    ap3216c_init(&ap3216c);
	
	while (1)
	{
        ap3216c.get_data(&ap3216c);

        debug.printf("light: %d, proximity: %d, infrared: %d\r\n", 
                    ap3216c.data.light, ap3216c.data.proximity, ap3216c.data.infrared);

        delay_ms(500);
	}
}
