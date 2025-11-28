#include <stdio.h>
#include "drv_delay.h"
#include "drv_uart.h"
#include "drv_esp8266.h"
#include "utils.h"

static uart_dev_t uart_debug;
static uint8_t uart_debug_tx_buf[512];
static uint8_t uart_debug_rx_buf[512];
static const uart_cfg_t uart_debug_cfg = {
    .uart_periph     = USART1,
    .baudrate        = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_9,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_10,
    .tx_buf          = uart_debug_tx_buf,
    .rx_buf          = uart_debug_rx_buf,
    .tx_buf_size     = sizeof(uart_debug_tx_buf),
    .rx_buf_size     = sizeof(uart_debug_rx_buf),
    .rx_single_max   = 256,
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static uart_dev_t uart_esp8266;
static uint8_t uart_esp8266_tx_buf[1024];
static uint8_t uart_esp8266_rx_buf[1025];   // >= rx_single_max + 1
static const uart_cfg_t uart_esp8266_cfg = {
    .uart_periph     = USART2,
    .baudrate        = 115200,
    .tx_port         = GPIOA,
    .tx_pin          = GPIO_Pin_2,
    .rx_port         = GPIOA,
    .rx_pin          = GPIO_Pin_3,
    .tx_buf          = uart_esp8266_tx_buf,
    .rx_buf          = uart_esp8266_rx_buf,
    .tx_buf_size     = sizeof(uart_esp8266_tx_buf),
    .rx_buf_size     = sizeof(uart_esp8266_rx_buf),
    .rx_single_max   = 1024,    // 需要获取天气预报功能时尽量不小于 1024
    .rx_pre_priority = 0,
    .rx_sub_priority = 0
};

static esp8266_dev_t esp8266;
static uint8_t esp8266_tx_buf[256];
static uint8_t esp8266_rx_buf[1024];    // 需要获取天气预报功能时尽量不小于 1024
int esp8266_uart_send_data(uint8_t *data, uint32_t len)
{
    return uart_esp8266.ops->send_data(&uart_esp8266, data, len);
}

int esp8266_uart_recv_data(uint8_t **data, uint32_t *len)
{
    return uart_esp8266.ops->recv_data(&uart_esp8266, data, len);
}

static esp8266_uart_ops_t esp8266_uart_ops = {
    .send_data = esp8266_uart_send_data,
    .recv_data = esp8266_uart_recv_data
};

void esp8266_log_vprintf(const char *format, va_list args)
{
    uart_debug.ops->vprintf(&uart_debug, format, args);
}

static esp8266_log_ops_t esp8266_log_ops = {
    .vprintf = esp8266_log_vprintf
};

static esp8266_cfg_t esp8266_cfg = {
    .uart_ops      = &esp8266_uart_ops,
    .log_ops       = &esp8266_log_ops,
    .delay_ms      = delay_ms,
    .tx_buf        = esp8266_tx_buf,
    .rx_buf        = esp8266_rx_buf,
    .tx_buf_size   = sizeof(esp8266_tx_buf),
    .rx_buf_size   = sizeof(esp8266_rx_buf)
};

/**
 * @brief	ESP8266 WiFi 测试
 */
void esp8266_wifi_test(const char *ssid, const char *pwd)
{
    esp8266.ops->wifi_connect(&esp8266, ssid, pwd);
    delay_ms(5000);
    esp8266.ops->wifi_disconnect(&esp8266);
}

/**
 * @brief	ESP8266 TCP 透传（作为客户端）测试
 */
void esp8266_tcp_transmit_test(const char *ssid, const char *pwd, const char *ip)
{
    uint8_t *tcp_recv_data;
    uint32_t tcp_recv_len;

    esp8266.ops->wifi_connect(&esp8266, ssid, pwd);
    esp8266.ops->tcp_transmit_connect(&esp8266, ip, 8088);
    esp8266.ops->tcp_send_data(&esp8266, "Hello", 5);
    delay_ms(100);
    esp8266.ops->tcp_send_data(&esp8266, "World", 5);
    delay_ms(100);

    while (1) {
        if (esp8266.ops->tcp_recv_data(&esp8266, &tcp_recv_data, &tcp_recv_len, 5000) == 0) {
            uart_debug.ops->printf(&uart_debug, "\r\nTCP recv: ");
            uart_debug.ops->send_data(&uart_debug, tcp_recv_data, tcp_recv_len);
            uart_debug.ops->printf(&uart_debug, "\r\n");
            if (tcp_recv_len == 1 && tcp_recv_data[0] == '@')
                break;
        }
	}

    esp8266.ops->tcp_transmit_disconnect(&esp8266);
    esp8266.ops->wifi_disconnect(&esp8266);
}

/**
 * @brief	ESP8266 获取时间（北京）测试
 */
void esp8266_get_time_test(const char *ssid, const char *pwd)
{
    uint8_t *tcp_recv_data;
    uint32_t tcp_recv_len;
    time_info_t time_info;

    esp8266.ops->wifi_connect(&esp8266, ssid, pwd);
    esp8266.ops->tcp_transmit_connect(&esp8266, "www.beijing-time.org", 80);
    esp8266.ops->tcp_send_data(&esp8266, "1\r\n", 3);

    if (esp8266.ops->tcp_recv_data(&esp8266, &tcp_recv_data, &tcp_recv_len, 5000) == 0) {
        uart_debug.ops->printf(&uart_debug, "\r\nTCP recv:\r\n");
        uart_debug.ops->send_data(&uart_debug, tcp_recv_data, tcp_recv_len);
        uart_debug.ops->printf(&uart_debug, "\r\n");
        parse_time(tcp_recv_data, &time_info);
    }

    esp8266.ops->tcp_transmit_disconnect(&esp8266);
    esp8266.ops->wifi_disconnect(&esp8266);

    uart_debug.ops->printf(&uart_debug, 
                        "\r\nTime: %04d-%02d-%02d %02d:%02d:%02d\r\n", 
                        time_info.year, 
                        time_info.month, 
                        time_info.day, 
                        time_info.hour, 
                        time_info.minute, 
                        time_info.second);
}

/**
 * @brief	ESP8266 获取天气测试
 * @param[in] ssid WiFi 名称
 * @param[in] pwd  WiFi 密码
 * @param[in] key  心知天气 API
 * @param[in] city 城市
 */
void esp8266_get_weather_test(const char *ssid, const char *pwd, const char *key, const char *city)
{
    uint8_t *tcp_recv_data;
    uint32_t tcp_recv_len;
    weather_info_t weather_info;
    char send[256];

    esp8266.ops->wifi_connect(&esp8266, ssid, pwd);
    esp8266.ops->tcp_transmit_connect(&esp8266, "api.seniverse.com", 80);
    
    snprintf(send, sizeof(send),
             "GET https://api.seniverse.com/v3/weather/now.json?key=%s&location=%s&language=en&unit=c\r\n",
             key, city);
    esp8266.ops->tcp_send_data(&esp8266, (uint8_t *)send, sizeof(send));

    if (esp8266.ops->tcp_recv_data(&esp8266, &tcp_recv_data, &tcp_recv_len, 5000) == 0) {
        uart_debug.ops->printf(&uart_debug, "\r\nTCP recv:\r\n");
        uart_debug.ops->send_data(&uart_debug, tcp_recv_data, tcp_recv_len);
        uart_debug.ops->printf(&uart_debug, "\r\n");
        parse_cur_weather((const char *)tcp_recv_data, &weather_info);
	}

    delay_ms(500);

    snprintf(send, sizeof(send),
             "GET https://api.seniverse.com/v3/weather/daily.json?key=%s&location=%s&language=en&unit=c&start=0&days=5\r\n",
             key, city);
    esp8266.ops->tcp_send_data(&esp8266, (uint8_t *)send, sizeof(send));

    if (esp8266.ops->tcp_recv_data(&esp8266, &tcp_recv_data, &tcp_recv_len, 5000) == 0) {
        uart_debug.ops->printf(&uart_debug, "\r\nTCP recv:\r\n");
        uart_debug.ops->send_data(&uart_debug, tcp_recv_data, tcp_recv_len);
        uart_debug.ops->printf(&uart_debug, "\r\n");
        parse_weather_forecast_data((const char *)tcp_recv_data, &weather_info);
	}

    esp8266.ops->tcp_transmit_disconnect(&esp8266);
    esp8266.ops->wifi_disconnect(&esp8266);

    uart_debug.ops->printf(&uart_debug, "\r\n%s Weather:\r\n", weather_info.city);
    uart_debug.ops->printf(&uart_debug, "%s: %s %d°C\r\n", weather_info.last_update, weather_info.weather_now, weather_info.temp_now);
    for (uint8_t i = 0; i < weather_info.daily_cnt; i++)
    {
        uart_debug.ops->printf(&uart_debug, "%s: %s %d/%d°C\r\n", 
                               weather_info.daily[i].date, 
                               weather_info.daily[i].weather, 
                               weather_info.daily[i].temp_high, 
                               weather_info.daily[i].temp_low);
    }
}

/* 
 * 注意：
 * 如果要获取天气，需要适当增大启动文件中的栈大小！（默认 0x00000400 不够用） 
 * 连接 WiFi 时，如果使用手机热点， AP 频段应设置为 2.4 GHz
 * 心知天气 API 请自行官网注册申请: www.seniverse.com
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    drv_uart_init(&uart_debug, &uart_debug_cfg);
    drv_uart_init(&uart_esp8266, &uart_esp8266_cfg);
    drv_esp8266_init(&esp8266, &esp8266_cfg);

    /* 逐项功能测试（建议每次只解除一项注释运行） */
    // esp8266_wifi_test("你的WiFi名称", "你的WiFi密码");
    // esp8266_tcp_transmit_test("你的WiFi名称", "你的WiFi密码", "服务器IP地址");
    // esp8266_get_time_test("你的WiFi名称", "你的WiFi密码");
    esp8266_get_weather_test("你的WiFi名称", "你的WiFi密码", "你的心知天气API", "Beijing"); // 城市名可修改（全拼首字母大写）

	while (1);
}
