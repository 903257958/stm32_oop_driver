#include "wifi.h"

/* 函数声明 */
static int8_t __extract_time_data(uint8_t *buf, wifi_time_info_t *time);
static int8_t __extract_current_weather_data(const char *json_str, wifi_weather_info_t *weather);
static int8_t __extract_weather_forecast_data(const char *json_str, wifi_weather_info_t *weather);

/******************************************************************************
 * @brief	ESP8266获取时间
 * @param	dev		:  esp8266_dev_t 结构体指针
 * @param	time	:  wifi_time_info_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t esp8266_get_time(esp8266_dev_t *dev, wifi_time_info_t *time)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	char recv_data[512];

    ESP8266_DEBUG("\r\nESP8266 is obtaining Beijing time...\r\n");

	/* 1. 设置单连接 */
    ESP8266_DEBUG("\r\n1. Set single connection\r\n");
    while (dev->send_cmd(dev, "AT+CIPMUX=0\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
    
    /* 2. 连接TCP服务器 */
    ESP8266_DEBUG("\r\n2. Connect to TCP server\r\n");
    while (dev->send_cmd(dev, "AT+CIPSTART=\"TCP\",\"www.beijing-time.org\",80\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}

	/* 3. 设置透传模式 */
    ESP8266_DEBUG("\r\n3. Set transparent transmission mode\r\n");
    while (dev->send_cmd(dev, "AT+CIPMODE=1\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
	
	/* 4. 开始透传 */
	ESP8266_DEBUG("\r\n4. Start transparent transmission\r\n");
	while (dev->send_cmd(dev, "AT+CIPSEND\r\n", ">", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}

	/* 5. 获取时间数据 */
	ESP8266_DEBUG("\r\n5. Obtain time data\r\n");
	while (dev->send_cmd(dev, "1\r\n", "GMT", recv_data) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}

	/* 6. 提取时间数据 */
	ESP8266_DEBUG("\r\n6. Extract time data\r\n");
	__extract_time_data((uint8_t *)recv_data, time);

	/* 7. 退出透传 */
	ESP8266_DEBUG("\r\n7. Exit transparent transmission\r\n");
	dev->send_cmd(dev, "+++", NULL, NULL);
	ESP8266_DELAY_MS(1000);

	/* 8. AT测试 */
	ESP8266_DEBUG("\r\n8. AT test\r\n");
	while (dev->send_cmd(dev, "AT\r\n", "OK", NULL))
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
	ESP8266_DELAY_MS(500);

	/* 9. 断开TCP连接 */
	ESP8266_DEBUG("\r\n9. Close TCP connection\r\n");
	while (dev->send_cmd(dev, "AT+CIPCLOSE\r\n", "OK", NULL))
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}

    ESP8266_DEBUG("\r\nESP8266 successfully obtained Beijing time!\r\n");

	return 0;
}

/******************************************************************************
 * @brief	ESP8266获取天气
 * @param	dev   	:  esp8266_dev_t 结构体指针
 * @param	city   	:  城市名
 * @param	weather	:  wifi_weather_info_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t esp8266_get_weather(esp8266_dev_t *dev, const char *city, wifi_weather_info_t *weather)
{
	if (!dev || !dev->init_flag)
		return -1;

	char current_weather_buf[512];
	char weather_forecast_buf[1024];
    char now_query[256];
    char forecast_query[256];
	
	ESP8266_DEBUG("\r\nESP8266 obtains weather information...\r\n");

	/* 1. 设置单连接 */
    ESP8266_DEBUG("\r\n1. Set single connection\r\n");
    while (dev->send_cmd(dev, "AT+CIPMUX=0\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
    
    /* 2. 连接TCP服务器 */
    ESP8266_DEBUG("\r\n2. Connect to TCP server\r\n");
    while (dev->send_cmd(dev, "AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}

	/* 3. 设置透传模式 */
    ESP8266_DEBUG("\r\n3. Set transparent transmission mode\r\n");
    while (dev->send_cmd(dev, "AT+CIPMODE=1\r\n", "OK", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
	
	/* 4. 开始透传 */
	ESP8266_DEBUG("\r\n4. Start transparent transmission\r\n");
	while (dev->send_cmd(dev, "AT+CIPSEND\r\n", ">", NULL) != 0)
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}

	/* 5. 获取当前天气 */
    ESP8266_DEBUG("\r\n5. Get current weather information\r\n");
    snprintf(now_query, sizeof(now_query),
             "GET https://api.seniverse.com/v3/weather/now.json?key=SwmhHrSHKGC4OXf6v&location=%s&language=en&unit=c\r\n",
             city);
             
    while (dev->send_cmd(dev, now_query, "results", current_weather_buf) != 0)  
    {
        ESP8266_DEBUG("retrying...\r\n");
        ESP8266_DELAY_MS(500);
    }

    /* 6. 提取当前天气数据 */
	ESP8266_DEBUG("\r\n6. Extract current weather data\r\n");
	__extract_current_weather_data((const char *)current_weather_buf, weather);
	ESP8266_DELAY_MS(500);

    /* 7. 获取天气预报 */
	ESP8266_DEBUG("\r\n7. Get weather forecast\r\n");
    snprintf(forecast_query, sizeof(forecast_query),
             "GET https://api.seniverse.com/v3/weather/daily.json?key=SwmhHrSHKGC4OXf6v&location=%s&language=en&unit=c&start=0&days=5\r\n",
             city);

    while (dev->send_cmd(dev, forecast_query, "results", weather_forecast_buf) != 0)  
    {
        ESP8266_DEBUG("retrying...\r\n");
        ESP8266_DELAY_MS(500);
    }

    /* 8. 提取天气预报数据 */
	ESP8266_DEBUG("\r\n8. Extract weather forecast data\r\n");
	__extract_weather_forecast_data((const char *)weather_forecast_buf, weather);
    ESP8266_DELAY_MS(500);

	/* 9. 退出透传 */
	ESP8266_DEBUG("\r\n9. Exit transparent transmission\r\n");
	dev->send_cmd(dev, "+++", NULL, NULL);
	ESP8266_DELAY_MS(1000);

	/* 10. AT测试 */
	ESP8266_DEBUG("\r\n10. AT test\r\n");
	while (dev->send_cmd(dev, "AT\r\n", "OK", NULL))
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}
	ESP8266_DELAY_MS(500);
	
	/* 11. 断开TCP连接 */
	ESP8266_DEBUG("\r\n11. Close TCP connection\r\n");
	while (dev->send_cmd(dev, "AT+CIPCLOSE\r\n", "OK", NULL))
	{
		ESP8266_DEBUG("retrying...\r\n");
		ESP8266_DELAY_MS(500);
	}

    ESP8266_DEBUG("\r\nESP8266 successfully obtained weather information!\r\n");

	return 0;
}

/******************************************************************************
 * @brief	提取时间数据
 * @param	buf		:	缓存区
 * @param	time	:	wifi_time_info_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __extract_time_data(uint8_t *buf, wifi_time_info_t *time)
{
    const char *date_prefix = "Date:";
    const uint8_t days_in_month[12] = {
		31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
	};
    const char *months[] = {
		"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    char *date_line = strstr((char*)buf, date_prefix);
    if (!date_line) return -1;

    char weekday[4], month[4];
    int year, day, hour, minute, second;
	uint8_t i;

    /* 例如：Date: Tue, 06 May 2025 12:38:08 GMT */
    uint8_t ret = sscanf(date_line, "Date: %3s, %d %3s %d %d:%d:%d",
                     weekday, &day, month, &year, &hour, &minute, &second);
    if (ret != 7) return -2;

    /* 月份字符串转数字 */
    uint8_t month_num = 0;
    for (i = 0; i < 12; ++i)
	{
        if (strncmp(month, months[i], 3) == 0)
		{
            month_num = i + 1;
            break;
        }
    }
    if (month_num == 0) return -3;

    /* 设置初始时间 */
    time->year = year;
    time->month = month_num;
    time->day = day;
    time->hour = hour;
    time->minute = minute;
    time->second = second;

    /* 加8小时处理跨天 */
    time->hour += 8;
    while (time->hour >= 24)
	{
        time->hour -= 24;
        time->day++;

        int max_day = days_in_month[time->month - 1];
        if (time->month == 2 && ((time->year % 4 == 0 && time->year % 100 != 0) || (time->year % 400 == 0)))
		{
            max_day = 29;
        }

        if (time->day > max_day)
		{
            time->day = 1;
            time->month++;
            if (time->month > 12)
			{
                time->month = 1;
                time->year++;
            }
        }
    }

    return 0;
}

/******************************************************************************
 * @brief	提取当前天气数据
 * @param	json_str	:	接收到的JSON数据
 * @param	weather		:	wifi_weather_info_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __extract_current_weather_data(const char *json_str, wifi_weather_info_t *weather)
{
    if (!json_str || !weather)
        return -1;

    memset(weather, 0, sizeof(wifi_weather_info_t));

    /* 解析city */
    const char *loc_ptr = strstr(json_str, "\"name\":\"");
    if (loc_ptr)
        sscanf(loc_ptr + 8, "%31[^\"]", weather->city);

    /* 解析last_update */
    const char *update_ptr = strstr(json_str, "\"last_update\":\"");
    if (update_ptr)
    {
        update_ptr += strlen("\"last_update\":\"");
        const char *end_quote = strchr(update_ptr, '"');
        if (end_quote)
        {
            int len = (int)(end_quote - update_ptr);
            if (len > 31)
                len = 31;
            strncpy(weather->last_update, update_ptr, len);
            weather->last_update[len] = '\0';
        }
    }

    /* 解析weather_now */
    const char *text_ptr = strstr(json_str, "\"text\":\"");
    if (text_ptr)
        sscanf(text_ptr + 8, "%15[^\"]", weather->weather_now);

    /* 解析temp_now */
    const char *temp_ptr = strstr(json_str, "\"temperature\":\"");
    if (temp_ptr)
    {
        char temp[16];
        sscanf(temp_ptr + 15, "%15[^\"]", temp);
        weather->temp_now = atoi(temp);
    }

    return 0;
}

/******************************************************************************
 * @brief	提取天气预报数据
 * @param	json_str	:	接收到的JSON数据
 * @param	weather		:	wifi_weather_info_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __extract_weather_forecast_data(const char *json_str, wifi_weather_info_t *weather)
{
    if (!json_str || !weather)
        return -1;

    /* 解析daily数组 */
    const char *daily_ptr = strstr(json_str, "\"daily\":[");
    if (!daily_ptr)
        return -2;

    daily_ptr += 8; // 跳过 "daily":[

    int day_index = 0;

    while (*daily_ptr && day_index < WEATHER_FORECAST_DAY_NUM)
    {
        const char *day_start = strchr(daily_ptr, '{');
        if (!day_start)
            break;

        const char *day_end = strchr(day_start, '}');
        if (!day_end)
            break;

        int day_len = (int)(day_end - day_start + 1);
        char day_block[512] = {0};
        if (day_len >= sizeof(day_block))
            day_len = sizeof(day_block) - 1;
        strncpy(day_block, day_start, day_len);
        day_block[day_len] = '\0';

        /* 解析date */
        const char *date_ptr = strstr(day_block, "\"date\":\"");
        if (date_ptr)
            sscanf(date_ptr + 8, "%15[^\"]", weather->daily[day_index].date);

        /* 解析weather */
        const char *td_ptr = strstr(day_block, "\"text_day\":\"");
        if (td_ptr)
            sscanf(td_ptr + 12, "%15[^\"]", weather->daily[day_index].weather);

        /* 解析temp_high */
        const char *temp_high_ptr = strstr(day_block, "\"high\":\"");
        if (temp_high_ptr)
        {
            char temp[16];
            sscanf(temp_high_ptr + 8, "%15[^\"]", temp);
            weather->daily[day_index].temp_high = atoi(temp);
        }

        /* 解析temp_low */
        const char *temp_low_ptr = strstr(day_block, "\"low\":\"");
        if (temp_low_ptr)
        {
            char temp[16];
            sscanf(temp_low_ptr + 7, "%15[^\"]", temp);
            weather->daily[day_index].temp_low = atoi(temp);
        }

		day_index++;
        daily_ptr = day_end + 1;
    }

    weather->daily_cnt = day_index;
    return 0;
}
