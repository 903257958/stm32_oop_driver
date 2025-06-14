#include "main.h"

static uint8_t uart1_tx_buf[2048];
static uint8_t uart1_rx_buf[2048];
uart_dev_t debug = {
    .config = {
        .uartx          = USART1,
        .baud           = 115200,
        .tx_port        = GPIOA,
        .tx_pin         = GPIO_Pin_9,
        .rx_port        = GPIOA,
        .rx_pin         = GPIO_Pin_10,
        .tx_buf         = uart1_tx_buf,
        .rx_buf         = uart1_rx_buf,
        .tx_buf_size    = sizeof(uart1_tx_buf),
        .rx_buf_size    = sizeof(uart1_tx_buf),
        .rx_single_max  = 512
    }
};

esp8266_dev_t esp8266 = {
    .config = {USART2, GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3, "AT+CWJAP=\"shouji\",\"thxd156369\"\r\n"}
};

int main(void)
{
    /* 注意：需要增加startup_stm32f40_41xxx.s中的栈大小！ */

    uint8_t i;
    wifi_time_info_t time;
    wifi_weather_info_t weather;
    char tcp_recv_buf[128];
    int16_t tcp_recv_len;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    delay_init(168);
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

    /* 设置TCP透传 */
    esp8266.set_tcp_transparent(&esp8266, "10.70.6.178", 8088);

    /* TCP透传发送数据 */
    esp8266.tcp_send_data(&esp8266, "Hello\r\n");

    /* 退出TCP透传 */
    // esp8266.exit_tcp_transparent(&esp8266);

	while (1)
	{
        /* TCP透传接收数据 */
        tcp_recv_len = esp8266.tcp_recv_data(&esp8266, tcp_recv_buf, sizeof(tcp_recv_buf));
        if (tcp_recv_len > 0)
        {
            debug.printf("TCP transparent received %d bytes: %s\r\n", tcp_recv_len, tcp_recv_buf);
        }
	}
}
