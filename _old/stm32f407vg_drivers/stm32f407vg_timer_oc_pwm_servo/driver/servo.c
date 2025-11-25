#include "servo.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)

#define TIMER_FREQ	72000000

#elif defined(STM32F40_41xxx) || defined(STM32F40_41xxx)

#define TIMER_FREQ	84000000

#elif defined(STM32F429_439xx)

#define TIMER_FREQ	90000000

#endif

#endif

/* Servo私有数据结构体 */
typedef struct {
	pwm_dev_t pwm;	// PWM
} servo_priv_data_t;	

/* 函数声明 */									
static int8_t __servo_set_angle(servo_dev_t *dev, float angle);
static int8_t __servo_deinit(servo_dev_t *dev);

/******************************************************************************
 * @brief	初始化Servo
 * @param	dev	:	servo_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t servo_init(servo_dev_t *dev)
{
	if (!dev)
		return -1;

	/* 保存私有数据 */
	dev->priv_data = (servo_priv_data_t *)malloc(sizeof(servo_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	servo_priv_data_t *priv_data = (servo_priv_data_t *)dev->priv_data;

	priv_data->pwm.config.timx = dev->config.timx;
	priv_data->pwm.config.oc_channel = dev->config.oc_channel;
	priv_data->pwm.config.arr = 20000 - 1;
	priv_data->pwm.config.psc = TIMER_FREQ / 1000000 - 1;
	priv_data->pwm.config.port = dev->config.port;
	priv_data->pwm.config.pin = dev->config.pin;
	
	/* 初始化PWM */
	pwm_init(&priv_data->pwm);
	
	/* 函数指针赋值 */
	dev->set_angle = __servo_set_angle;
	dev->deinit = __servo_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	Servo设置角度
 * @param	dev		:	servo_dev_t 结构体指针
 * @param	angle	:	角度，范围0~180°
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __servo_set_angle(servo_dev_t *dev, float angle)
{
	if (!dev || !dev->init_flag)
		return -1;

	servo_priv_data_t *priv_data = (servo_priv_data_t *)dev->priv_data;

	priv_data->pwm.set_compare(&priv_data->pwm, angle / 180 * 2000 + 500);

	return 0;
}

/******************************************************************************
 * @brief	去初始化Servo
 * @param	dev	:	servo_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __servo_deinit(servo_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	servo_priv_data_t *priv_data = (servo_priv_data_t *)dev->priv_data;

	/* 去初始化PWM */
	priv_data->pwm.deinit(&priv_data->pwm);

	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
