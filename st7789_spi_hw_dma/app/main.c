#include "main.h"

LCDDev_t lcd = {.info = {
	SPI1,
	GPIOA, GPIO_Pin_9,
	GPIOA, GPIO_Pin_8,
	GPIOB, GPIO_Pin_5,
	GPIOC, GPIO_Pin_7,
	TIM3, 2,
	VERTICAL_REVERSE
}};

int main(void)
{
	lcd_init(&lcd);
	
	lcd.clear(&lcd, YELLOW);

	lcd.show_char(&lcd, 0, 50, 'a', LIGHTBLUE, WHITE, LCD_16X32, 0);
	lcd.show_char(&lcd, 0, 100, 'x', BRRED, WHITE, LCD_16X32, 1);
	lcd.show_string(&lcd, 0, 150, "ABC", RED, WHITE, LCD_16X32, 0);
	lcd.show_string(&lcd, 0, 200, "DEF", GREEN, WHITE, LCD_16X32, 1);
	lcd.show_chinese(&lcd, 100, 0, "你好", RED, WHITE, LCD_32X32, 0);
	lcd.show_num(&lcd, 100, 50, 123456, 6, BLUE, RED, LCD_16X32, 0);
	lcd.show_hex_num(&lcd, 100, 100, 0x1234, 6, BLUE, RED, LCD_16X32, 0);
	lcd.show_float_num(&lcd, 70, 150, 212.34, 3, 4, YELLOW, BLACK, LCD_16X32, 0);
	lcd.show_float_num(&lcd, 70, 200, -456.78, 3, 2, YELLOW, BLACK, LCD_16X32, 0);
	
	while (1)
	{
		
	}
}
