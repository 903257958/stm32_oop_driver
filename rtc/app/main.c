#include "main.h"

UARTDev_t debug = {.config = {
    USART1, 115200,
    GPIOA, GPIO_Pin_9,
    GPIOA, GPIO_Pin_10
}};
RTCDev_t rtc;

int main(void)
{
	delay_init(168);
	uart_init(&debug);
	rtc_init(&rtc);
	
	while(1)
	{
		rtc.get_time(&rtc, &rtc.time);
		rtc.get_date(&rtc, &rtc.date);

		debug.printf(&debug, "\r\nDate:20%02d-%02d-%02d-%d\r\n", rtc.date.year, rtc.date.month, rtc.date.date, rtc.date.week);
		debug.printf(&debug, "Time:%02d:%02d:%02d\r\n", rtc.time.hour, rtc.time.minute, rtc.time.second);

		delay_ms(1000);
	}
}
