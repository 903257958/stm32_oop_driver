#include "main.h"

OLEDDev_t oled = {.config = {
	SPI1,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_7,
	GPIOC, GPIO_Pin_5,
	GPIOC, GPIO_Pin_4,
	GPIOC, GPIO_Pin_1,
	2,
	SPI_MODE_0
}};

int i;

int main(void)
{
    delay_init(168);
	oled_init(&oled);
	
	while(1)
	{
		i++;
		oled.show_num(&oled, 0, 0, i, 8, OLED_8X16);
		oled.update(&oled);

	}
}
