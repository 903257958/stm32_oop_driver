#include "main.h"

static uint8_t uart1_tx_buf[2048];
static uint8_t uart1_rx_buf[2048];
uart_dev_t uart1 = {
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
        .rx_single_max  = 512
    }
};
adc_dev_t  adc1_4 = {.config = {ADC1, ADC_Channel_4, GPIOA, GPIO_Pin_4}};
adc_dev_t  adc1_5 = {.config = {ADC1, ADC_Channel_5, GPIOA, GPIO_Pin_5}};

int main(void)
{
	uint16_t adc1_4_val, adc1_5_val;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(72);
	uart_init(&uart1);
    adc_init(&adc1_4);
	adc_init(&adc1_5);
	
	while (1)
	{
        adc1_4.get_val(&adc1_4, &adc1_4_val);
        adc1_5.get_val(&adc1_5, &adc1_5_val);

		uart1.printf("adc1_4: %d, adc1_5: %d\r\n", adc1_4_val, adc1_5_val);

		delay_ms(500);
	}
}
