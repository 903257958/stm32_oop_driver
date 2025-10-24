#include "led.h"

#ifdef USE_STDPERIPH_DRIVER

/**************************** STM32F1 系列 ****************************/
#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define led_io_clock_enable(port)                   								   \
    do {                                             								   \
        if (port == GPIOA)       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); \
        else if (port == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); \
        else if (port == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); \
        else if (port == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); \
        else if (port == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); \
        else if (port == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); \
        else if (port == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); \
    } while (0)

#define led_config_io_out_pp(port, pin)            	  	  \
    do {                                            	  \
        GPIO_InitTypeDef GPIO_InitStructure;        	  \
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; \
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
        GPIO_InitStructure.GPIO_Pin   = pin;        	  \
        GPIO_Init(port, &GPIO_InitStructure);             \
    } while (0)

#define led_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)


/**************************** STM32F4 系列 ****************************/
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define led_io_clock_enable(port)                   								   \
    do {                                            								   \
        if (port == GPIOA)       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); \
        else if (port == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); \
        else if (port == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); \
        else if (port == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); \
        else if (port == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); \
        else if (port == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); \
        else if (port == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); \
    } while (0)

#define led_config_io_out_pp(port, pin)            		  \
    do {                                             	  \
        GPIO_InitTypeDef GPIO_InitStructure;        	  \
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;    \
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    \
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; \
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
        GPIO_InitStructure.GPIO_Pin   = pin;        	  \
        GPIO_Init(port, &GPIO_InitStructure);       	  \
    } while (0)

#define led_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)


/**************************** GD32F1 系列 ****************************/
#elif defined(GD32F10X_MD) || defined(GD32F10X_HD)

#define led_io_clock_enable(port)                  				     \
    do {                                           				     \
        if (port == GPIOA)       rcu_periph_clock_enable(RCU_GPIOA); \
        else if (port == GPIOB)  rcu_periph_clock_enable(RCU_GPIOB); \
        else if (port == GPIOC)  rcu_periph_clock_enable(RCU_GPIOC); \
        else if (port == GPIOD)  rcu_periph_clock_enable(RCU_GPIOD); \
        else if (port == GPIOE)  rcu_periph_clock_enable(RCU_GPIOE); \
        else if (port == GPIOF)  rcu_periph_clock_enable(RCU_GPIOF); \
        else if (port == GPIOG)  rcu_periph_clock_enable(RCU_GPIOG); \
    } while (0)

#define led_config_io_out_pp(port, pin)	\
        gpio_init(port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin)

#define led_io_write(port, pin, value)	gpio_bit_write(port, pin, (bit_status)value)

#endif  /* MCU SERIES SELECTION */

#endif  /* USE_STDPERIPH_DRIVER */

typedef struct {
	bool status;	// 状态：false 灭，true 亮，默认灭
} led_priv_data_t;

static int led_on(led_dev_t *dev);
static int led_off(led_dev_t *dev);
static int led_get_status(led_dev_t *dev, bool *status);
static int led_toggle(led_dev_t *dev);
static int led_drv_deinit(led_dev_t *dev);

/**
 * @brief   初始化 LED 设备
 * @param[in,out] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int led_drv_init(led_dev_t *dev)
{
	if (!dev)
		return -1;
	
	dev->priv_data = (led_priv_data_t *)malloc(sizeof(led_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	led_priv_data_t *priv_data = (led_priv_data_t *)dev->priv_data;
	
	led_io_clock_enable(dev->config.port);
	led_config_io_out_pp(dev->config.port, dev->config.pin);
	
	dev->on = led_on;
	dev->off = led_off;
	dev->get_status = led_get_status;
	dev->toggle = led_toggle;
	dev->deinit = led_drv_deinit;
	
    dev->init_flag = true;
    
	/* 默认关闭 */
	priv_data->status = false;
	led_off(dev);

	return 0;
}

/**
 * @brief   打开 LED
 * @param[in] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_on(led_dev_t *dev)
{
	led_priv_data_t *priv_data = (led_priv_data_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	led_io_write(dev->config.port, dev->config.pin, !dev->config.off_level);
	priv_data->status = true;	// 保存此时LED的状态
	
	return 0;
}

/**
 * @brief   关闭 LED
 * @param[in] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_off(led_dev_t *dev)
{
	led_priv_data_t *priv_data = (led_priv_data_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	led_io_write(dev->config.port, dev->config.pin, dev->config.off_level);
	priv_data->status = false;	// 保存此时LED的状态
	
	return 0;
}

/**
 * @brief   获取 LED 的状态
 * @param[in]  dev    led_dev_t 结构体指针
 * @param[out] status 状态：false 灭，true 亮
 * @return	0 表示成功，其他值表示失败
 */
static int led_get_status(led_dev_t *dev, bool *status)
{
	led_priv_data_t *priv_data = (led_priv_data_t *)dev->priv_data;

	if (!dev || !dev->init_flag)
    	return -1;
	
	*status = priv_data->status;

	return 0;
}

/**
 * @brief   翻转 LED
 * @param[in] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_toggle(led_dev_t *dev)
{
	bool status;

	if (!dev || !dev->init_flag)
		return -1;
	
	dev->get_status(dev, &status);
	if (status)
		dev->off(dev);
	else
		dev->on(dev);
	
	return 0;
}

/**
 * @brief   去初始化 LED 设备
 * @param[in,out] dev led_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int led_drv_deinit(led_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	free(dev->priv_data);
    dev->priv_data = NULL;
	dev->init_flag = false;
	
	return 0;
}
