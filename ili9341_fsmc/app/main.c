#include "main.h"

LCDDev_t lcd;

int main(void)
{
	lcd_init(&lcd);

	lcd.clear(&lcd, RED);
	delay_ms(100);
	lcd.clear(&lcd, GREEN);
	delay_ms(100);
	lcd.clear(&lcd, BLUE);
	delay_ms(100);
	lcd.clear(&lcd, BLACK);

	lcd.show_string(&lcd, 0, 0, "Hello,world!", BLUE, WHITE, LCD_6X12, 1);
	lcd.show_string(&lcd, 0, 12, "Hello,world!", YELLOW, WHITE, LCD_8X16, 0);
	lcd.show_string(&lcd, 0, 12 + 16, "Hello,world!", GREEN, WHITE, LCD_12X24, 1);
	lcd.show_string(&lcd, 0, 12 + 16 + 24, "Hello,world!", BROWN, RED, LCD_16X32, 0);
	lcd.show_chinese(&lcd, 0, 12 + 16 + 24 + 32, "你好", GREEN, RED, LCD_32X32, 1);
	lcd.show_image(&lcd, 0, 12 + 16 + 24 + 32 + 32, 128, 128, imageTest);

	while(1)
	{
		
	}
}
