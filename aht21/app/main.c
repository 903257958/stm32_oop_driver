#include "main.h"

LCDDev_t lcd = {.info = {
	SPI1,
	GPIOA, GPIO_Pin_5,
	GPIOA, GPIO_Pin_7,
	GPIOC, GPIO_Pin_5,
	GPIOA, GPIO_Pin_4,
	GPIOC, GPIO_Pin_4,
	GPIOA, GPIO_Pin_3,
	TIM2, 4,
	VERTICAL_FORWARD
}};

AHT21Dev_t aht21 = {.info = {
    GPIOB, GPIO_Pin_6,
    GPIOB, GPIO_Pin_7
}};

int main(void)
{
	delay_init(168);
	lcd_init(&lcd);
    
    aht21_init(&aht21);
	
	lcd.clear(&lcd, BLACK);
	lcd.show_string(&lcd, 30, 0, "Humi:", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_string(&lcd, 30, 30, "Temp:", WHITE, BLACK, LCD_12X24, 0);
	
	while (1)
	{
		aht21.get_data(&aht21);

		lcd.show_float_num(&lcd, 90, 0, aht21.humi, 3, 1, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_float_num(&lcd, 90, 30, aht21.temp, 3, 1, WHITE, BLACK, LCD_12X24, 0);

		delay_ms(900);		
	}
}
