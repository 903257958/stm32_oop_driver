#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

/* PWM 操作接口结构体 */
typedef struct {
	int (*set_psc)(uint16_t psc);
	int (*set_arr)(uint16_t arr);
	int (*set_compare)(uint16_t compare);
} servo_pwm_ops_t;

/* 配置结构体 */
typedef struct {
	const servo_pwm_ops_t *pwm_ops;
} servo_cfg_t;

typedef struct servo_dev servo_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*set_angle)(servo_dev_t *dev, float angle);
	int (*deinit)(servo_dev_t *dev);
} servo_ops_t;

/* 设备结构体 */
struct servo_dev {
	servo_cfg_t cfg;
	const servo_ops_t *ops;
};

/**
 * @brief   初始化舵机设备驱动
 * @param[out] dev servo_dev_t 结构体指针
 * @param[in]  cfg servo_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_servo_init(servo_dev_t *dev, const servo_cfg_t *cfg);

#endif
