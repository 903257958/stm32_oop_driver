#include "main.h"

LCDDev_t lcd;
RTCDev_t rtc;

char dateBuf[20];
char timeBuf[20];

int main(void)
{
	lcd_init(&lcd);
	rtc_init(&rtc);
	
	lcd.clear(&lcd, BLACK);

	while(1)
	{
		rtc.get_time(&rtc, &rtc.time);
		rtc.get_date(&rtc, &rtc.date);
		sprintf((char *)dateBuf, "Date:20%02d-%02d-%02d-%d", rtc.date.year, rtc.date.month, rtc.date.date, rtc.date.week); 
		sprintf((char *)timeBuf, "Time:%02d:%02d:%02d", rtc.time.hour, rtc.time.minute, rtc.time.second); 
		lcd.show_string(&lcd, 0, 0, dateBuf, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_string(&lcd, 0, 32, timeBuf, WHITE, BLACK, LCD_12X24, 0);
	}
}
