#include "main.h"

eeprom_dev_t at24c02 = {.config = {GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7}};

int main(void)
{
    uint16_t i;
    uint8_t read_data[256];

	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

	uart0_init(921600);
	eeprom_init(&at24c02);

    for (i = 0; i < 256; i++)
    {
        at24c02.write_byte(&at24c02, i, i);
        delay_ms(2);
    }

    at24c02.read_data(&at24c02, 0, 256, read_data);

    for (i = 0; i < 256; i++)
    {
        uart0_printf("%d ", read_data[i]);
    }
    uart0_printf("\r\n");
    
	while (1)
	{

	}
}
