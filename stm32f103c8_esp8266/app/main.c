#include "main.h"

uart_dev_t    debug   = {.config = {USART1, 115200, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10}};
esp8266_dev_t esp8266 = {.config = {USART2, GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3, "AT+CWJAP=\"shouji\",\"thxd156369\"\r\n"}};

int main(void)
{
    /* 注意：需要增加startup_stm32f10x_md.s中的栈大小！ */

    uint8_t i;
    wifi_time_info_t time;
    wifi_weather_info_t weather;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(72);
    uart_init(&debug);
	esp8266_init(&esp8266);

    /* 获取时间 */
    esp8266_get_time(&esp8266, &time);

    /* 获取天气 */
    esp8266_get_weather(&esp8266, "beijing", &weather);

    /* 打印时间 */
    debug.printf("\r\nTime: %04d-%02d-%02d %02d:%02d:%02d\r\n", 
                time.year, time.month, time.day, time.hour, time.minute, time.second);

    /* 打印天气 */
    debug.printf("\r\n%s Weather:\r\n", weather.city);
    debug.printf("%s: %s %d°C\r\n", weather.last_update, weather.weather_now, weather.temp_now);
    for (i = 0; i < weather.daily_cnt; i++)
    {
        debug.printf("%s: %s %d/%d°C\r\n", 
            weather.daily[i].date, weather.daily[i].weather, weather.daily[i].temp_high, weather.daily[i].temp_low);
    }

	while (1)
	{

	}
}
