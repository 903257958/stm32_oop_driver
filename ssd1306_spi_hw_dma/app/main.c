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

int main(void)
{
	delay_init(168);
	oled_init(&oled);
	oled.show_string(&oled, 0, 0, "Hello,world!", OLED_8X16);
	oled.show_string(&oled, 0, 16, "Hello,world!", OLED_8X16);
	oled.show_string(&oled, 0, 32, "Hello,world!", OLED_8X16);
	oled.show_string(&oled, 0, 48, "Hello,world!", OLED_8X16);
	oled.update(&oled);
	
	while(1)
	{
		
	}
}
