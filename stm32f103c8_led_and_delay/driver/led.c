#include "led.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__led_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}

#define	__led_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__led_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__led_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}

#define	__led_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__led_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#endif

/* LED私有数据结构体 */
typedef struct {
	bool status;		// 状态，false灭/true亮，默认灭
}LEDPrivData_t;

static int8_t __led_on(LEDDev_t *dev);
static int8_t __led_off(LEDDev_t *dev);
static int8_t __led_get_status(LEDDev_t *dev);
static int8_t __led_toggle(LEDDev_t *dev);
static int8_t __led_deinit(LEDDev_t *dev);

/******************************************************************************
 * @brief	初始化LED
 * @param	dev	:  LEDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t led_init(LEDDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 初始化私有数据 */
	dev->priv_data = (LEDPrivData_t *)malloc(sizeof(LEDPrivData_t));
	if (!dev->priv_data)
		return -1;
	
	LEDPrivData_t *priv_data = (LEDPrivData_t *)dev->priv_data;
	
	/* 配置时钟与GPIO */	
	__led_config_gpio_clock_enable(dev->config.port);
	__led_config_io_out_pp(dev->config.port, dev->config.pin);
	
	/* 函数指针赋值 */
	dev->on = __led_on;
	dev->off = __led_off;
	dev->get_status = __led_get_status;
	dev->toggle = __led_toggle;
	dev->deinit = __led_deinit;
	
    dev->init_flag = true;
    
	/* 默认关闭 */
	priv_data->status = false;
	__led_off(dev);

	return 0;
}

/******************************************************************************
 * @brief	打开LED
 * @param	dev   :  LEDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __led_on(LEDDev_t *dev)
{
	LEDPrivData_t *priv_data = (LEDPrivData_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	 __led_io_write(dev->config.port, dev->config.pin, !dev->config.off_level);	// LED亮
	priv_data->status = true;												// 保存此时LED的状态
	
	return 0;
}

/******************************************************************************
 * @brief	关闭LED
 * @param	dev   :  LEDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __led_off(LEDDev_t *dev)
{
	LEDPrivData_t *priv_data = (LEDPrivData_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	 __led_io_write(dev->config.port, dev->config.pin, dev->config.off_level);	// LED灭
	priv_data->status = false;												// 保存此时LED的状态
	
	return 0;
}

/******************************************************************************
 * @brief	获取LED的状态
 * @param	dev   :  LEDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __led_get_status(LEDDev_t *dev)
{
	LEDPrivData_t *priv_data = (LEDPrivData_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	return priv_data->status;
}

/******************************************************************************
 * @brief	翻转LED
 * @param	dev   :  LEDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __led_toggle(LEDDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	if(dev->get_status(dev))
	{
		dev->off(dev);
	}
	else
	{
		dev->on(dev);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化LED
 * @param	dev   :  LEDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __led_deinit(LEDDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
