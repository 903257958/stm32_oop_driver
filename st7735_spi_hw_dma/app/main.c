#include "main.h"

LCDDev_t lcd = {.info = {
	SPI2,
	GPIOB, GPIO_Pin_0,
	GPIOB, GPIO_Pin_1,
	GPIOB, GPIO_Pin_12,
	GPIOC, GPIO_Pin_0,
	2,
	SPI_MODE_3,
	LCD_USE_DMA
}};

int main(void)
{
	lcd_init(&lcd);
	
	lcd.fill(&lcd, BLUE);
	lcd.update(&lcd);
	
	lcd.show_string(&lcd, 50, 40, "Hello", YELLOW, BLUE, LCD_12X24);
	lcd.show_chinese(&lcd, 56, 64, "你好", YELLOW, BLUE, LCD_24X24);
	lcd.update(&lcd);
	
	uint16_t i;
	
	while(1)
	{
		for(i = 0; i < 4; i++)
		{
			lcd.show_image(&lcd, 40 * i, 0, 40, 40, imageTest);
			lcd.update(&lcd);
			delay_ms(100);
			lcd.fill_area(&lcd, 40 * i, 0, 40, 40, BLUE);
		}
		
		lcd.show_image(&lcd, 120, 44, 40, 40, imageTest);
		lcd.update(&lcd);
		delay_ms(100);
		lcd.fill_area(&lcd, 120, 44, 40, 40, BLUE);
		
		for(i = 0; i < 4; i++)
		{
			lcd.show_image(&lcd, 120 - 40 * i, 88, 40, 40, imageTest);
			lcd.update(&lcd);
			delay_ms(100);
			lcd.fill_area(&lcd, 120 - 40 * i, 88, 40, 40, BLUE);
		}
		
		lcd.show_image(&lcd, 0, 44, 40, 40, imageTest);
		lcd.update(&lcd);
		delay_ms(100);
		lcd.fill_area(&lcd, 0, 44, 40, 40, BLUE);
	}
}
