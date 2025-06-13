#include "vibration_motor.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__vibration_motor_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}

#define	__vibration_motor_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
															GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
															GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
															GPIO_InitStructure.GPIO_Pin = pin; \
															GPIO_Init(port, &GPIO_InitStructure); \
														}

#define	__vibration_motor_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__vibration_motor_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}

#define	__vibration_motor_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
															GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
															GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
															GPIO_InitStructure.GPIO_Pin = pin; \
															GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
															GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
															GPIO_Init(port, &GPIO_InitStructure); \
														}

#define	__vibration_motor_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#endif

/* 振动马达私有数据结构体 */
typedef struct {
	bool status;	// 状态，false关/true开，默认关
} vibration_motor_priv_data_t;

/* 函数声明 */
static int8_t __vibration_motor_on(vibration_motor_dev_t *dev);
static int8_t __vibration_motor_off(vibration_motor_dev_t *dev);
static bool __vibration_motor_get_status(vibration_motor_dev_t *dev);
static int8_t __vibration_motor_toggle(vibration_motor_dev_t *dev);
static int8_t __vibration_motor_deinit(vibration_motor_dev_t *dev);

/******************************************************************************
 * @brief	初始化振动马达
 * @param	dev	:	vibration_motor_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t vibration_motor_init(vibration_motor_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 初始化私有数据 */
	dev->priv_data = (vibration_motor_priv_data_t *)malloc(sizeof(vibration_motor_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	vibration_motor_priv_data_t *priv_data = (vibration_motor_priv_data_t *)dev->priv_data;
	
	/* 配置时钟与GPIO */	
	__vibration_motor_io_clock_enable(dev->config.port);
	__vibration_motor_config_io_out_pp(dev->config.port, dev->config.pin);
	
	/* 函数指针赋值 */
	dev->on = __vibration_motor_on;
	dev->off = __vibration_motor_off;
	dev->get_status = __vibration_motor_get_status;
	dev->toggle = __vibration_motor_toggle;
	dev->deinit = __vibration_motor_deinit;
	
    dev->init_flag = true;
    
	/* 默认关闭 */
	priv_data->status = false;
	__vibration_motor_off(dev);

	return 0;
}

/******************************************************************************
 * @brief	打开振动马达
 * @param	dev	:	vibration_motor_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __vibration_motor_on(vibration_motor_dev_t *dev)
{
	vibration_motor_priv_data_t *priv_data = (vibration_motor_priv_data_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	 __vibration_motor_io_write(dev->config.port, dev->config.pin, GPIO_LEVEL_HIGH);	// 振动马达开
	priv_data->status = true;															// 保存此时振动马达的状态
	
	return 0;
}

/******************************************************************************
 * @brief	关闭振动马达
 * @param	dev	:	vibration_motor_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __vibration_motor_off(vibration_motor_dev_t *dev)
{
	vibration_motor_priv_data_t *priv_data = (vibration_motor_priv_data_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	 __vibration_motor_io_write(dev->config.port, dev->config.pin, GPIO_LEVEL_LOW);	// 振动马达关
	priv_data->status = false;														// 保存此时振动马达的状态
	
	return 0;
}

/******************************************************************************
 * @brief	获取振动马达的状态
 * @param	dev	:	vibration_motor_dev_t 结构体指针
 * @return	状态
 ******************************************************************************/
static bool __vibration_motor_get_status(vibration_motor_dev_t *dev)
{
	vibration_motor_priv_data_t *priv_data = (vibration_motor_priv_data_t *)dev->priv_data;
	
	return priv_data->status;
}

/******************************************************************************
 * @brief	翻转振动马达
 * @param	dev	:	vibration_motor_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __vibration_motor_toggle(vibration_motor_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	if (__vibration_motor_get_status(dev))
	{
		__vibration_motor_off(dev);
	}
	else
	{
		__vibration_motor_on(dev);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化振动马达
 * @param	dev	:	vibration_motor_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __vibration_motor_deinit(vibration_motor_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
