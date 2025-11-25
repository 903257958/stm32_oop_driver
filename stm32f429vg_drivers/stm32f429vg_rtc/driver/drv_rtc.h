#ifndef DRV_RTC_H
#define DRV_RTC_H

#include <stdint.h>
#include <stdbool.h>

/* 时间数据结构体 */
typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  week;
} rtc_time_t;

typedef struct rtc_dev rtc_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*set_time)(rtc_dev_t *dev, const rtc_time_t *time);
    int (*get_time)(rtc_dev_t *dev, rtc_time_t *time);
	int (*deinit)(rtc_dev_t *dev);
} rtc_ops_t;

/* 设备结构体 */
struct rtc_dev {
	const rtc_ops_t *ops;
};

/**
 * @brief   初始化 RTC 驱动
 * @param[out] dev rtc_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_rtc_init(rtc_dev_t *dev);

#endif
