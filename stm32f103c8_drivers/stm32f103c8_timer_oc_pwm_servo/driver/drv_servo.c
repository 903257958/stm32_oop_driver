#include "drv_servo.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)
#define TIMER_FREQ	72000000

#elif defined(STM32F40_41xxx) || defined(STM32F40_41xxx)
#define TIMER_FREQ	84000000

#elif defined(STM32F429_439xx)
#define TIMER_FREQ	90000000

#endif

/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int servo_set_angle_impl(servo_dev_t *dev, float angle);
static int servo_deinit_impl(servo_dev_t *dev);

/* 操作接口表 */
static const servo_ops_t servo_ops = {
	.set_angle = servo_set_angle_impl,
	.deinit    = servo_deinit_impl
};

/**
 * @brief   初始化舵机设备驱动
 * @param[out] dev servo_dev_t 结构体指针
 * @param[in]  cfg servo_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_servo_init(servo_dev_t *dev, const servo_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;

	dev->cfg = *cfg;
	dev->ops = &servo_ops;

	cfg->pwm_ops->set_psc(TIMER_FREQ / 1000000 - 1);
	cfg->pwm_ops->set_arr(20000 - 1);
	return 0;
}

/**
 * @brief   舵机设置角度
 * @param[in] dev   servo_dev_t 结构体指针
 * @param[in] angle 角度，范围：0~180°
 * @return	0 表示成功，其他值表示失败
 */
static int servo_set_angle_impl(servo_dev_t *dev, float angle)
{
	if (!dev)
		return -EINVAL;

	return dev->cfg.pwm_ops->set_compare(angle / 180 * 2000 + 500);
}

/**
 * @brief   去初始化舵机
 * @param[in] dev servo_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int servo_deinit_impl(servo_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
