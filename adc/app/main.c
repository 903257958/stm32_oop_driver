#include "main.h"

UARTDev_t uart1 = {.config = {
	USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10
}};

ADCDev_t adc1_5 = {.config = {
	ADC1, ADC_Channel_5, GPIOA, GPIO_Pin_5
}};

uint16_t adc1_val;
char str[50];

int main(void)
{
	delay_init(168);
	uart_init(&uart1);
	adc_init(&adc1_5);
	
	while(1)
	{
		adc1_5.get_val(&adc1_5, &adc1_val);

		sprintf(str, "adc1_5: %d\r\n", adc1_val);
		uart1.send_string(&uart1, str);

		delay_ms(500);
	}
}
