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
bmp280_dev_t bmp280 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
    uint8_t id;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	uart_init(&debug);
	bmp280_init(&bmp280);

    bmp280.get_id(&bmp280, &id);
    debug.printf("ID: 0x%x\r\n", id);
    
	while (1)
	{
        bmp280.get_data(&bmp280);

        debug.printf("temperature: %.2f°C, pressure: %.2fhPa\r\n", bmp280.data.temperature, bmp280.data.pressure);

        delay_ms(1000);
	}
}
