#include "main.h"

uart_dev_t    debug   = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
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
