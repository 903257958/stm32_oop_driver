#include "dht11.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__dht11_io_clock_enable(port)	{	if(port == GPIOA)      {RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
											else if(port == GPIOB) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
											else if(port == GPIOC) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
											else if(port == GPIOD) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
											else if(port == GPIOE) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
											else if(port == GPIOF) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
											else if(port == GPIOG) {RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
										}

#define	__dht11_config_io_in_pu(port, pin)  {	GPIO_InitTypeDef	GPIO_InitStructure; \
											    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
											    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
										        GPIO_InitStructure.GPIO_Pin = pin; \
											    GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__dht11_config_io_out_pp(port, pin) {	GPIO_InitTypeDef	GPIO_InitStructure; \
											    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
											    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
										        GPIO_InitStructure.GPIO_Pin = pin; \
											    GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __dht11_set_io_high(port, pin)	GPIO_SetBits(port, pin)
											  
#define __dht11_set_io_low(port, pin)	GPIO_ResetBits(port, pin)
											  
#define __dht11_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)
	
#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx) || defined(STM32F411xE)
	
#define	__dht11_io_clock_enable(port)	{	if(port == GPIOA)      {RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
											else if(port == GPIOB) {RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
											else if(port == GPIOC) {RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
											else if(port == GPIOD) {RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
											else if(port == GPIOE) {RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
											else if(port == GPIOF) {RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
											else if(port == GPIOG) {RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
										}

#define	__dht11_config_io_in_pu(port, pin)  {	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__dht11_config_io_out_pp(port, pin) {	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __dht11_set_io_high(port, pin)	GPIO_SetBits(port, pin)
											  
#define __dht11_set_io_low(port, pin)	GPIO_ResetBits(port, pin)
											  
#define __dht11_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)
	
#endif

#endif
                                            
/* 函数声明 */
static int8_t __dht11_reset(dht11_dev_t *dev);
static int8_t __dht11_check(dht11_dev_t *dev);
static uint8_t __dht11_read_bit(dht11_dev_t *dev);
static uint8_t __dht11_read_byte(dht11_dev_t *dev);
static int8_t __dht11_get_data(dht11_dev_t *dev);
static int8_t __dht11_deinit(dht11_dev_t *dev);
	
/******************************************************************************
 * @brief	DHT11初始化
 * @param	dev :   dht11_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t dht11_init(dht11_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 配置时钟与GPIO */
	__dht11_io_clock_enable(dev->config.port);
	__dht11_config_io_out_pp(dev->config.port, dev->config.pin);
	
	/* 函数指针赋值 */
	dev->get_data = __dht11_get_data;
	dev->deinit = __dht11_deinit;
	
	dev->init_flag = true;
    
    __dht11_reset(dev);
	return __dht11_check(dev);
}

/******************************************************************************
 * @brief	DHT11复位
 * @param	dev	:   dht11_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __dht11_reset(dht11_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
    
	__dht11_config_io_out_pp(dev->config.port, dev->config.pin);
    __dht11_set_io_low(dev->config.port, dev->config.pin);
    DHT11_DELAY_MS(20);
    __dht11_set_io_high(dev->config.port, dev->config.pin);
    DHT11_DELAY_US(13);

    return 0;
}

/******************************************************************************
 * @brief	DHT11检查
 * @param	dev	:   dht11_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __dht11_check(dht11_dev_t *dev)
{
    if (!dev || !dev->init_flag)
		return -1;
    
    uint8_t retry = 0;
	__dht11_config_io_in_pu(dev->config.port, dev->config.pin);
    
    while ((__dht11_io_read(dev->config.port, dev->config.pin)) && retry < 100)
	{
		retry++;
		DHT11_DELAY_US(1);
	};	 
	
	if (retry >= 100)
	{
		return -2;
	}
	else
	{
		retry = 0;
	}

    while (!(__dht11_io_read(dev->config.port, dev->config.pin)) && retry < 100)
	{
		retry++;
		DHT11_DELAY_US(1);
	};
	if (retry >= 100) return -3;	    
	return 0;
}

/******************************************************************************
 * @brief	DHT11读取一位数据
 * @param	dev	:   dht11_dev_t 结构体指针
 * @return	读取到的一位数据
 ******************************************************************************/
static uint8_t __dht11_read_bit(dht11_dev_t *dev)			 
{
 	uint8_t retry = 0;

	while ((__dht11_io_read(dev->config.port, dev->config.pin)) && retry < 100)  // 等待变为低电平
	{
		retry++;
		DHT11_DELAY_US(1);
	}

	retry = 0;

	while (!(__dht11_io_read(dev->config.port, dev->config.pin)) && retry < 100) // 等待变高电平
	{
		retry++;
		DHT11_DELAY_US(1);
	}

	DHT11_DELAY_US(40);// 等待40us

	if (__dht11_io_read(dev->config.port, dev->config.pin))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * @brief	DHT11读一个字节
 * @param	dev	:   dht11_dev_t 结构体指针
 * @return	读出的一个字节
 ******************************************************************************/
static uint8_t __dht11_read_byte(dht11_dev_t *dev)
{      
	uint8_t i, dat;
	dat = 0;
    
	for (i = 0; i < 8; i++) 
	{
		dat <<= 1; 
		dat |= __dht11_read_bit(dev);
	}
    
	return dat;
}

/******************************************************************************
 * @brief	DHT11获取数据
 * @param	dev	:   dht11_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __dht11_get_data(dht11_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
    
    uint8_t buf[5];
	uint8_t i;
    
	__dht11_reset(dev);
    
	if (__dht11_check(dev) == 0)
	{
		for (i = 0; i < 5; i++)// 读取40位数据
		{
			buf[i] = __dht11_read_byte(dev);
		}
		if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
		{
			dev->data.humidity = buf[0];
			dev->data.temperature = buf[2];
		}
	}
	else
	{
		return -2;
	}
    
	return 0;
}

/******************************************************************************
 * @brief	去初始化DHT11
 * @param	dev	:   dht11_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __dht11_deinit(dht11_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}
