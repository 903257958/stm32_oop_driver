#include "main.h"

LCDDev_t lcd;
RTCDev_t rtc;

RTCTime_t time = {23, 59, 56};
RTCDate_t date = {25, 1, 5, 7};
char dateBuf[20];
char timeBuf[20];

int main(void)
{
	lcd_init(&lcd);
	rtc_init(&rtc);
	
	lcd.clear(&lcd, BLACK);

	// rtc.set_time(&rtc, &time);
	// rtc.set_date(&rtc, &date);

	while(1)
	{
		rtc.get_time(&rtc, &time);
		rtc.get_date(&rtc, &date);
		sprintf((char *)dateBuf, "Date:20%02d-%02d-%02d-%d", date.year, date.month, date.date, date.week); 
		sprintf((char *)timeBuf, "Time:%02d:%02d:%02d", time.hour, time.minute, time.second); 
		lcd.show_string(&lcd, 0, 0, dateBuf, WHITE, BLACK, LCD_12X24, 0);
		lcd.show_string(&lcd, 0, 32, timeBuf, WHITE, BLACK, LCD_12X24, 0);
	}
}
