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
max30102_dev_t max30102 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7, GPIOB, GPIO_Pin_9}};

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(168);
	uart_init(&debug);
    max30102_init(&max30102);

    max30102.software_init(&max30102);
    
	while(1)
	{
		max30102.get_data(&max30102);
        
        debug.printf("Heart rate: %d, Blood oxygen: %d%%\r\n", max30102.data.heart_rate, max30102.data.blood_oxygen);
	}
}
