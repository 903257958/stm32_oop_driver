#include "main.h"

oled_dev_t oled = {.config = {
	SPI1,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_7,
	GPIOB, GPIO_Pin_0,
	GPIOA, GPIO_Pin_6,
	GPIOB, GPIO_Pin_4,
	2,
	SPI_MODE_0
}};

int main(void)
{
	int i;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
    delay_init(168);
	oled_init(&oled);
	
	while (1)
	{
		i++;
		oled.show_num(&oled, 0, 0, i, 8, OLED_8X16);
		oled.update(&oled);
	}
}
