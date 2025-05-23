#include "main.h"

UARTDev_t debug = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};

RTCDev_t rtc = {.config = {2025, 1, 1, 23, 59, 55}};

RTCTime_t rtc_time = {2000, 1, 1, 23, 59, 55};

int main(void)
{
	delay_init(168);
	uart_init(&debug);
	rtc_init(&rtc);
	
	// rtc.set_time(&rtc, &rtc_time);

	while(1)
	{
		rtc.get_time(&rtc);

		debug.printf("\r\nDate:%04d-%02d-%02d-%d\r\n", rtc.time.year, rtc.time.month, rtc.time.day, rtc.time.week);
		debug.printf("Time:%02d:%02d:%02d\r\n", rtc.time.hour, rtc.time.minute, rtc.time.second);

		delay_ms(1000);
	}
}
