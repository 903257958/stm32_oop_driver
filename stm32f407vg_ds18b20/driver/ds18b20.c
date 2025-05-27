#include "delay.h"
#include "ds18b20.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__ds18b20_config_clock_enable(port)		{	if(port == GPIOA)     	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}

#define	__ds18b20_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef	GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin ; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

#define	__ds18b20_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef	GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_InitStructure.GPIO_Pin = pin ; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

#define __ds18b20_set_io_high(port, pin)	GPIO_SetBits(port, pin)
											  
#define __ds18b20_set_io_low(port, pin)		GPIO_ResetBits(port, pin)
											  
#define __ds18b20_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#define __ds18b20_gpio_deinit(port)	GPIO_DeInit(port)
	
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	
#define	__ds18b20_config_clock_enable(port)		{	if(port == GPIOA)     	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}

#define	__ds18b20_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

#define	__ds18b20_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
													GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
													GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
													GPIO_InitStructure.GPIO_Pin = pin; \
													GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
													GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
													GPIO_Init(port, &GPIO_InitStructure); \
												}

#define __ds18b20_set_io_high(port, pin)	GPIO_SetBits(port, pin)
											  
#define __ds18b20_set_io_low(port, pin)		GPIO_ResetBits(port, pin)
											  
#define __ds18b20_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#define __ds18b20_gpio_deinit(port)	GPIO_DeInit(port)
	
#endif
	
static int8_t __ds18b20_get_temperature(DS18B20Dev_t *dev);
static int8_t __ds18b20_deinit(DS18B20Dev_t *dev);
	
/******************************************************************************
 * @brief      DS18B20初始化
 * @param[in]  dev	:  DS18B20Dev_t结构体指针
 * @return     0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t ds18b20_init(DS18B20Dev_t *dev)
{
	if (!dev)
        return -1;
   
	/* 配置时钟与GPIO */	
    __ds18b20_config_clock_enable(dev->config.port); 
    __ds18b20_config_io_out_pp(dev->config.port, dev->config.pin);
	
	/* 函数指针赋值 */
	dev->get_temperature = __ds18b20_get_temperature;
	dev->deinit = __ds18b20_deinit;
	
	dev->init_flag = true;
    return 0;
}

/******************************************************************************
 * @brief      DS18B20复位
 * @param[in]  dev   :  DS18B20Dev_t结构体指针
 * @return     0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ds18b20_reset(DS18B20Dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;  // 初始化引脚之后
	
	uint8_t rst_val = 0;
	
	__ds18b20_config_io_out_pp(dev->config.port, dev->config.pin);
	
	__ds18b20_set_io_high(dev->config.port, dev->config.pin);				// 先置高电平
	delay_us(5);
	
	__ds18b20_set_io_low(dev->config.port, dev->config.pin);				// 主机首先发出一个480us的低电平脉冲
	delay_us(480);
	
	__ds18b20_set_io_high(dev->config.port, dev->config.pin);				// 然后释放总线变为高电平
	delay_us(60);
	
	__ds18b20_config_io_in_pu(dev->config.port, dev->config.pin);
	
	rst_val = __ds18b20_io_read(dev->config.port, dev->config.pin);			// 如果有低电平出现说明总线上有器件已做出应答
	delay_us(480);															// 保证480us的延时
	
	__ds18b20_config_io_out_pp(dev->config.port, dev->config.pin);
	__ds18b20_set_io_high(dev->config.port, dev->config.pin);				// 再次置高电平，结束
	
	return rst_val;
}

/******************************************************************************
 * @brief      DS18B20写数据
 * @param[in]  dev  :  DS18B20Dev_t结构体指针
 * @param[in]  data   	:  要写入的8位数据
 * @return     0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ds18b20_write_data(DS18B20Dev_t *dev, uint8_t data)
{	
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t i;
	
	for(i = 0;i < 8;i++)
	{
		__ds18b20_config_io_out_pp(dev->config.port, dev->config.pin);
		
		__ds18b20_set_io_low(dev->config.port, dev->config.pin);			// 首先置低电平
		delay_us(15);
		
		if(data & (0x01 << i))											// 写数据，低位先行
		{
			__ds18b20_set_io_high(dev->config.port, dev->config.pin);		// 如果要写的数据为1，则置高电平
		}
		
		delay_us(45);
		__ds18b20_set_io_high(dev->config.port, dev->config.pin);			// 延时后置高电平
	}
	__ds18b20_set_io_high(dev->config.port, dev->config.pin);				// 再次置高电平，结束
	
	return 0;
}

/******************************************************************************
 * @brief      DS18B20读数据
 * @param[in]  dev  :  DS18B20Dev_t结构体指针
 * @return     读出的8位数据
 ******************************************************************************/
static uint8_t __ds18b20_read_data(DS18B20Dev_t *dev)
{
	uint8_t data = 0;
	uint8_t i;
	
	for(i = 0;i < 8;i++)
	{
		__ds18b20_config_io_out_pp(dev->config.port, dev->config.pin);

		__ds18b20_set_io_low(dev->config.port, dev->config.pin);						// 首先置低电平2us
		delay_us(2);
		__ds18b20_set_io_high(dev->config.port, dev->config.pin);						// 然后置高电平2us
		delay_us(2);
		
		__ds18b20_config_io_in_pu(dev->config.port, dev->config.pin);
		
		data |= (__ds18b20_io_read(dev->config.port, dev->config.pin)) << i;			// 读数据，低位先行
		delay_us(60);
	}
	return data;
}

/******************************************************************************
 * @brief      DS18B20获取温度
 * @param[in]  dev  :  DS18B20Dev_t结构体指针
 * @return     温度值
 ******************************************************************************/
static int8_t __ds18b20_get_temperature(DS18B20Dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	uint16_t temp_tmp;
	uint8_t temp_high, temp_low;
	float temp;
	static float temp_last = -100;
	
	__ds18b20_reset(dev);
	__ds18b20_write_data(dev, 0xcc);		// 跳过读序号列号
	__ds18b20_write_data(dev, 0x44);		// 启动温度转换
	
	delay_ms(750);
	
	__ds18b20_reset(dev);
	__ds18b20_write_data(dev, 0xcc);		// 跳过读序号列号
	__ds18b20_write_data(dev, 0xbe);		// 读取温度寄存器
	
	temp_low = __ds18b20_read_data(dev);	// 保存温度低8位数据
	temp_high = __ds18b20_read_data(dev);	// 保存温度高8位数据
	
	temp_tmp = (temp_high << 8) | temp_low;	// 转换为16位温度数据
	temp = (float)temp_tmp * 0.0625f;
	
	if(temp > -55 && temp < 125)			// 温度值计算
	{
		temp_last = temp;
		dev->temperature = temp;
	}
	else
	{
		dev->temperature = temp_last;
	}

	return 0;
}

/******************************************************************************
 * @brief      去初始化DS18B20
 * @param[in]  dev   :  DS18B20Dev_t结构体指针
 * @return     0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __ds18b20_deinit(DS18B20Dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}
