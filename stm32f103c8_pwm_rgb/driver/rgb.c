#include "rgb.h"

#if defined(STM32F10X_MD) || defined(STM32F10X_HD)

#define TIMER_FREQ	72000000

#elif defined(STM32F40_41xxx) || defined(STM32F40_41xxx)

#define TIMER_FREQ	84000000

#elif defined(STM32F429_439xx)

#define TIMER_FREQ	90000000

#endif

#define PWM_FREQ		1000	// PWM频率1kHz，适合RGB
#define PWM_RESOLUTION 	255		// 色彩精度为8位

/* RGB私有数据结构体 */
typedef struct {
	PWMDev_t red_pwm;	// R PWM
	PWMDev_t green_pwm;	// G PWM
	PWMDev_t blue_pwm;	// B PWM
}RGBPrivData_t;	

/* 函数声明 */				
static int8_t __rgb_red(RGBDev_t *dev);
static int8_t __rgb_yellow(RGBDev_t *dev);
static int8_t __rgb_green(RGBDev_t *dev);
static int8_t __rgb_blue(RGBDev_t *dev);
static int8_t __rgb_white(RGBDev_t *dev);
static int8_t __rgb_off(RGBDev_t *dev);
static int8_t __rgb_set_color(RGBDev_t *dev, uint8_t red, uint8_t green, uint8_t blue);
static int8_t __rgb_next_rainbow_color(struct RGBDev *dev, uint8_t *red, uint8_t *green, uint8_t *blue);
static int8_t __rgb_deinit(RGBDev_t *dev);

/******************************************************************************
 * @brief	初始化RGB
 * @param	dev	:	RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t rgb_init(RGBDev_t *dev)
{
	if (!dev)
		return -1;

	/* 保存私有数据 */
	dev->priv_data = (RGBPrivData_t *)malloc(sizeof(RGBPrivData_t));
	if (!dev->priv_data)
		return -1;
	
	RGBPrivData_t *priv_data = (RGBPrivData_t *)dev->priv_data;

	priv_data->red_pwm.config.timx = dev->config.red_timx;
	priv_data->red_pwm.config.oc_channel = dev->config.red_oc_channel;
	priv_data->red_pwm.config.arr = PWM_RESOLUTION;
	priv_data->red_pwm.config.psc = (TIMER_FREQ / (PWM_FREQ * PWM_RESOLUTION)) - 1;
	priv_data->red_pwm.config.port = dev->config.red_port;
	priv_data->red_pwm.config.pin = dev->config.red_pin;
	
	priv_data->green_pwm.config.timx = dev->config.green_timx;
	priv_data->green_pwm.config.oc_channel = dev->config.green_oc_channel;
	priv_data->green_pwm.config.arr = PWM_RESOLUTION;
	priv_data->green_pwm.config.psc = (TIMER_FREQ / (PWM_FREQ * PWM_RESOLUTION)) - 1;
	priv_data->green_pwm.config.port = dev->config.green_port;
	priv_data->green_pwm.config.pin = dev->config.green_pin;

	priv_data->blue_pwm.config.timx = dev->config.blue_timx;
	priv_data->blue_pwm.config.oc_channel = dev->config.blue_oc_channel;
	priv_data->blue_pwm.config.arr = PWM_RESOLUTION;
	priv_data->blue_pwm.config.psc = (TIMER_FREQ / (PWM_FREQ * PWM_RESOLUTION)) - 1;
	priv_data->blue_pwm.config.port = dev->config.blue_port;
	priv_data->blue_pwm.config.pin = dev->config.blue_pin;
	
	/* 初始化PWM */
	pwm_init(&priv_data->red_pwm);
	pwm_init(&priv_data->green_pwm);
	pwm_init(&priv_data->blue_pwm);
	
	/* 函数指针赋值 */
	dev->red = __rgb_red;
	dev->yellow = __rgb_yellow;
	dev->green = __rgb_green;
	dev->blue = __rgb_blue;
	dev->white = __rgb_white;
	dev->off = __rgb_off;
	dev->set_color = __rgb_set_color;
	dev->next_rainbow_color = __rgb_next_rainbow_color;
	dev->deinit = __rgb_deinit;
	
	dev->init_flag = true;

	__rgb_off(dev);
	
	return 0;
}

/******************************************************************************
 * @brief	RGB红色
 * @param	dev		：	RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_red(RGBDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	__rgb_set_color(dev, 255, 0, 0);

	return 0;
}

/******************************************************************************
 * @brief	RGB黄色
 * @param	dev		：	RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_yellow(RGBDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	__rgb_set_color(dev, 255, 255, 0);

	return 0;
}

/******************************************************************************
 * @brief	RGB绿色
 * @param	dev		：	RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_green(RGBDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	__rgb_set_color(dev, 0, 255, 0);

	return 0;
}

/******************************************************************************
 * @brief	RGB蓝色
 * @param	dev		：	RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_blue(RGBDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	__rgb_set_color(dev, 0, 0, 255);

	return 0;
}

/******************************************************************************
 * @brief	RGB白色
 * @param	dev		：	RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_white(RGBDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	__rgb_set_color(dev, 255, 255, 255);

	return 0;
}

/******************************************************************************
 * @brief	RGB熄灭
 * @param	dev		：	RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_off(RGBDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	__rgb_set_color(dev, 0, 0, 0);

	return 0;
}

/******************************************************************************
 * @brief	RGB设置颜色
 * @param	dev		：	RGBDev_t 结构体指针
 * @param	red		：	红色RGB值，范围：0~255
 * @param	green	：	绿色RGB值，范围：0~255
 * @param	blue	：	蓝色RGB值，范围：0~255
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_set_color(RGBDev_t *dev, uint8_t red, uint8_t green, uint8_t blue)
{
	if (!dev || !dev->init_flag)
		return -1;

	RGBPrivData_t *priv_data = (RGBPrivData_t *)dev->priv_data;

	if (dev->config.off_level == GPIO_LEVEL_HIGH)
	{
		if (red == 0 && green == 0 && blue == 0)
		{
			priv_data->red_pwm.set_compare(&priv_data->red_pwm, PWM_RESOLUTION + 1);
			priv_data->green_pwm.set_compare(&priv_data->green_pwm, PWM_RESOLUTION + 1);
			priv_data->blue_pwm.set_compare(&priv_data->blue_pwm, PWM_RESOLUTION + 1);
		}
		else
		{
			priv_data->red_pwm.set_compare(&priv_data->red_pwm, 255 - red);
			priv_data->green_pwm.set_compare(&priv_data->green_pwm, 255 - green);
			priv_data->blue_pwm.set_compare(&priv_data->blue_pwm, 255 - blue);
		}
	}
	else
	{
		priv_data->red_pwm.set_compare(&priv_data->red_pwm, red);
		priv_data->green_pwm.set_compare(&priv_data->green_pwm, green);
		priv_data->blue_pwm.set_compare(&priv_data->blue_pwm, blue);
	}

	return 0;
}

/******************************************************************************
 * @brief	RGB彩虹的下一个颜色，用于彩色循环
 * @param	dev		：	RGBDev_t 结构体指针
 * @param	red		：	红色RGB值，范围：0~255
 * @param	green	：	绿色RGB值，范围：0~255
 * @param	blue	：	蓝色RGB值，范围：0~255
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_next_rainbow_color(struct RGBDev *dev, uint8_t *red, uint8_t *green, uint8_t *blue)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	static float h = 0.0f;
    float s = 1.0f;
    float v = 1.0f;

    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;

    float r1, g1, b1;

    if (h < 60)      { r1 = c; g1 = x; b1 = 0; }
    else if (h < 120){ r1 = x; g1 = c; b1 = 0; }
    else if (h < 180){ r1 = 0; g1 = c; b1 = x; }
    else if (h < 240){ r1 = 0; g1 = x; b1 = c; }
    else if (h < 300){ r1 = x; g1 = 0; b1 = c; }
    else             { r1 = c; g1 = 0; b1 = x; }

    *red = (r1 + m) * 255;
    *green = (g1 + m) * 255;
    *blue = (b1 + m) * 255;

    h += 1.0f;
    if (h >= 360.0f)
        h = 0;

	return 0;
}

/******************************************************************************
 * @brief	去初始化RGB
 * @param	dev   :  RGBDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __rgb_deinit(RGBDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	RGBPrivData_t *priv_data = (RGBPrivData_t *)dev->priv_data;

	/* 去初始化PWM */
	priv_data->red_pwm.deinit(&priv_data->red_pwm);
	priv_data->green_pwm.deinit(&priv_data->green_pwm);
	priv_data->blue_pwm.deinit(&priv_data->blue_pwm);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
