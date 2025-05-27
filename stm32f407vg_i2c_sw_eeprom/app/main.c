#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
EEPROMDev_t at24c02 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

uint16_t i;
uint8_t read_data[256];

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
	uart_init(&debug);
	eeprom_init(&at24c02);

    for (i = 0; i < 256; i++)
    {
        at24c02.write_byte(&at24c02, i, i);
        delay_ms(2);
    }

    at24c02.read_data(&at24c02, 0, 256, read_data);

    delay_ms(50);
    
    for (i = 0; i < 256; i++)
    {
        debug.printf("byte %d: %d\r\n", i, read_data[i]);
    }
    debug.printf("\r\n");
    
	while(1)
	{
        delay_ms(1000);
	}
}
