#include "main.h"

LCDDev_t lcd;

BMP280Dev_t bmp280 = {.info = {
    GPIOB, GPIO_Pin_6,
    GPIOB, GPIO_Pin_7,
}};

uint8_t g_id;

int main(void)
{
	delay_init(168);
	lcd_init(&lcd);
    bmp280_init(&bmp280);

	lcd.clear(&lcd, BLACK);
	
	bmp280.get_id(&bmp280, &g_id);
	lcd.show_string(&lcd, 0, 0, "ID: 0x", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_hex_num(&lcd, 72, 0, g_id, 2, WHITE, BLACK, LCD_12X24, 0);

	lcd.show_string(&lcd, 0, 30, "temperature:", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_string(&lcd, 0, 60, "pressure:", WHITE, BLACK, LCD_12X24, 0);
	lcd.show_string(&lcd, 0, 90, "altitude:", WHITE, BLACK, LCD_12X24, 0);

	while(1)
	{
		bmp280.get_data(&bmp280);
		
		lcd.show_float_num(&lcd, 156, 30, bmp280.data.temperature, 6, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_float_num(&lcd, 156, 60, bmp280.data.pressure, 8, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_float_num(&lcd, 156, 90, bmp280.data.altitude, 8, WHITE, BLACK, LCD_12X24, 0);
	}
}
