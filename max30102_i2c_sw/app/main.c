#include "main.h"

LCDDev_t lcd;

MAX30102Dev_t max30102 = {.info = {
    GPIOB, GPIO_Pin_6,
    GPIOB, GPIO_Pin_7,
	GPIOB, GPIO_Pin_9
}};

int main(void)
{
	delay_init(168);
	lcd_init(&lcd);
    max30102_init(&max30102);

	lcd.clear(&lcd, BLACK);
	lcd.show_string(&lcd, 0, 0, "heart rate:", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 0, 20, "blood oxygen:", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 124, 20, "%", WHITE, BLACK, LCD_8X16, 0);

	while(1)
	{
		max30102.get_data(&max30102);

		lcd.show_num(&lcd, 100, 0, max30102.heart_rate, 3, WHITE, BLACK, LCD_8X16, 0);
		lcd.show_num(&lcd, 108, 20, max30102.blood_oxygen, 2, WHITE, BLACK, LCD_8X16, 0);
	}
}
