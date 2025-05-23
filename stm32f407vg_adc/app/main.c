#include "main.h"

UARTDev_t uart1 = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};

ADCDev_t adc1_4 = {.config = {ADC1, ADC_Channel_4, GPIOA, GPIO_Pin_4}};
ADCDev_t adc1_5 = {.config = {ADC1, ADC_Channel_5, GPIOA, GPIO_Pin_5}};

uint16_t adc1_4_val, adc1_5_val;

int main(void)
{
	delay_init(168);
	uart_init(&uart1);
    adc_init(&adc1_4);
	adc_init(&adc1_5);
	
	while(1)
	{
        adc1_4.get_val(&adc1_4, &adc1_4_val);
        adc1_5.get_val(&adc1_5, &adc1_5_val);

		uart1.printf("adc1_4: %d, adc1_5: %d\r\n", adc1_4_val, adc1_5_val);

		delay_ms(500);
	}
}
