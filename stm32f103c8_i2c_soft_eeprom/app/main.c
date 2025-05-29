#include "main.h"

uart_dev_t   debug   = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
eeprom_dev_t at24c02 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7}};

int main(void)
{
    uint16_t i;
    uint8_t read_data[256];

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(72);
	uart_init(&debug);
	eeprom_init(&at24c02);

    for (i = 0; i < 256; i++)
    {
        at24c02.write_byte(&at24c02, i, i);
        delay_ms(2);
    }

    at24c02.read_data(&at24c02, 0, 256, read_data);

    for (i = 0; i < 256; i++)
    {
        debug.printf("%d ", read_data[i]);
    }
    debug.printf("\r\n");
    
	while (1)
	{
        
	}
}
