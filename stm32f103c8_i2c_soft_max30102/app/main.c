#include "main.h"

uart_dev_t	   debug 	= {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
max30102_dev_t max30102 = {.config = {GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7, GPIOB, GPIO_Pin_9}};

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	delay_init(72);
	uart_init(&debug);
    max30102_init(&max30102);

    max30102.software_init(&max30102);
    
	while(1)
	{
		max30102.get_data(&max30102);
        
        debug.printf("Heart rate: %d, Blood oxygen: %d%%\r\n", max30102.data.heart_rate, max30102.data.blood_oxygen);
	}
}
