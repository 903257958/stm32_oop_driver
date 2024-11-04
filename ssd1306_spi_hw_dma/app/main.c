#include "main.h"

OLEDDev_t oled = {.info = {
	SPI2,
	GPIOB, GPIO_Pin_0,
	GPIOB, GPIO_Pin_1,
	GPIOB, GPIO_Pin_12,
	2,
	SPI_MODE_0
}};

int main(void)
{
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
