#include "gpio.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__gpio_config_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
												else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
												else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
												else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
												else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
												else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
												else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
											}

#define	__gpio_config_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_InitStructure.GPIO_Pin = pin ; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define	__gpio_config_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_InitStructure.GPIO_Pin = pin ; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}
												
#define	__gpio_config_in_pn(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_InitStructure.GPIO_Pin = pin ; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define	__gpio_config_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_InitStructure.GPIO_Pin = pin ; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define	__gpio_config_out_od(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_InitStructure.GPIO_Pin = pin ; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define __gpio_read_in_bit(port, pin)	GPIO_ReadInputDataBit(port, pin)

#define __gpio_read_out_bit(port, pin)	GPIO_ReadOutputDataBit(port, pin)

#define	__gpio_write_bit(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__gpio_config_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
												else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
												else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
												else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
												else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
												else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
												else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
											}

#define	__gpio_config_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
											GPIO_InitStructure.GPIO_Pin = pin; \
											GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define	__gpio_config_in_pd(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
											GPIO_InitStructure.GPIO_Pin = pin; \
											GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define	__gpio_config_in_pn(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
											GPIO_InitStructure.GPIO_Pin = pin; \
											GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define	__gpio_config_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
											GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
											GPIO_InitStructure.GPIO_Pin = pin; \
											GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define	__gpio_config_out_od(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
											GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; \
											GPIO_InitStructure.GPIO_Pin = pin; \
											GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
											GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											GPIO_Init(port, &GPIO_InitStructure); \
										}

#define __gpio_read_in_bit(port, pin)	GPIO_ReadInputDataBit(port, pin)

#define __gpio_read_out_bit(port, pin)	GPIO_ReadOutputDataBit(port, pin)

#define	__gpio_write_bit(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#endif

/* 函数声明 */
static int __gpio_set(GPIODev_t *dev);
static int __gpio_reset(GPIODev_t *dev);
static int __gpio_read(GPIODev_t *dev, uint8_t *status);
static int __gpio_write(GPIODev_t *dev, uint8_t status);
static int __gpio_toggle(GPIODev_t *dev);
static int __gpio_deinit(GPIODev_t *dev);

/******************************************************************************
 * @brief	初始化GPIO
 * @param	dev	:  GPIODev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int gpio_init(GPIODev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 配置时钟与GPIO */	
	__gpio_config_clock_enable(dev->config.port);

	if (dev->config.mode == GPIO_MODE_IN_PU)
	{
		__gpio_config_in_pu(dev->config.port, dev->config.pin);
	}
	else if (dev->config.mode == GPIO_MODE_IN_PD)
	{
		__gpio_config_in_pd(dev->config.port, dev->config.pin);
	}
	else if (dev->config.mode == GPIO_MODE_IN_PN)
	{
		__gpio_config_in_pn(dev->config.port, dev->config.pin);
	}
	else if (dev->config.mode == GPIO_MODE_OUT_PP)
	{
		__gpio_config_out_pp(dev->config.port, dev->config.pin);
	}
	else if (dev->config.mode == GPIO_MODE_OUT_OD)
	{
		__gpio_config_out_od(dev->config.port, dev->config.pin);
	}
	
	/* 函数指针赋值 */
	dev->set = __gpio_set;
	dev->reset = __gpio_reset;
	dev->read = __gpio_read;
	dev->write = __gpio_write;
	dev->toggle = __gpio_toggle;
	dev->deinit = __gpio_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	GPIO置位
 * @param	dev   :  GPIODev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __gpio_set(GPIODev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	if (dev->config.mode != GPIO_MODE_OUT_PP && dev->config.mode != GPIO_MODE_OUT_OD)
		return -2;
	
	__gpio_write_bit(dev->config.port, dev->config.pin, GPIO_LEVEL_HIGH);	// GPIO高电平
	
	return 0;
}

/******************************************************************************
 * @brief	GPIO复位
 * @param	dev   :  GPIODev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __gpio_reset(GPIODev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	if (dev->config.mode != GPIO_MODE_OUT_PP && dev->config.mode != GPIO_MODE_OUT_OD)
		return -2;
	
	__gpio_write_bit(dev->config.port, dev->config.pin, GPIO_LEVEL_LOW);	// GPIO低电平
	
	return 0;
}

/******************************************************************************
 * @brief	GPIO读电平
 * @param	dev		:  GPIODev_t 结构体指针
 * @param	status	:  电平值
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __gpio_read(GPIODev_t *dev, uint8_t *status)
{
	if (!dev || !dev->init_flag)
		return -1;

	if (dev->config.mode == GPIO_MODE_OUT_PP || dev->config.mode == GPIO_MODE_OUT_OD)
	{
		*status = __gpio_read_out_bit(dev->config.port, dev->config.pin);
	}
	else
	{
		*status = __gpio_read_in_bit(dev->config.port, dev->config.pin);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	GPIO写电平
 * @param	dev		:  GPIODev_t 结构体指针
 * @param	status	:  电平值
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __gpio_write(GPIODev_t *dev, uint8_t status)
{
	if (!dev || !dev->init_flag)
		return -1;

	if (dev->config.mode != GPIO_MODE_OUT_PP && dev->config.mode != GPIO_MODE_OUT_OD)
		return -2;

	__gpio_write_bit(dev->config.port, dev->config.pin, status);

	return 0;
}

/******************************************************************************
 * @brief	翻转GPIO
 * @param	dev   :  GPIODev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __gpio_toggle(GPIODev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	if (dev->config.mode != GPIO_MODE_OUT_PP && dev->config.mode != GPIO_MODE_OUT_OD)
		return -2;
	
	if(__gpio_read_out_bit(dev->config.port, dev->config.pin) == GPIO_LEVEL_HIGH)
	{
		__gpio_write_bit(dev->config.port, dev->config.pin, GPIO_LEVEL_LOW);
	}
	else
	{
		__gpio_write_bit(dev->config.port, dev->config.pin, GPIO_LEVEL_HIGH);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化GPIO
 * @param	dev   :  GPIODev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __gpio_deinit(GPIODev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
