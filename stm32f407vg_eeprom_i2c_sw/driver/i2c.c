#include "delay.h"
#include "i2c.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__i2c_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												}
													
#define	__i2c_config_io_out_od(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__i2c_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)
												
#define __i2c_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__i2c_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}
													
#define	__i2c_config_io_out_od(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}
											
#define	__i2c_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)
												
#define __i2c_io_read(port, pin)	GPIO_ReadInputDataBit(port, pin)

#elif defined(GD32F10X_MD) || defined(GD32F10X_HD)

#define	__i2c_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{rcu_periph_clock_enable(RCU_GPIOA);} \
													else if(port == GPIOB)	{rcu_periph_clock_enable(RCU_GPIOB);} \
													else if(port == GPIOC)	{rcu_periph_clock_enable(RCU_GPIOC);} \
													else if(port == GPIOD)	{rcu_periph_clock_enable(RCU_GPIOD);} \
													else if(port == GPIOE)	{rcu_periph_clock_enable(RCU_GPIOE);} \
													else if(port == GPIOF)	{rcu_periph_clock_enable(RCU_GPIOF);} \
													else if(port == GPIOG)	{rcu_periph_clock_enable(RCU_GPIOG);} \
												}
													
#define	__i2c_config_io_out_od(port, pin)	gpio_init(port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, pin)
												
#define	__i2c_io_write(port, pin, value)	gpio_bit_write(port, pin, (bit_status)value)
												
#define __i2c_io_read(port, pin)	gpio_input_bit_get(port, pin)

#endif
		
/* 函数声明 */
static int8_t __i2c_scl_write(I2CDev_t *dev, uint8_t level);
static int8_t __i2c_sda_write(I2CDev_t *dev, uint8_t level);
static uint8_t __i2c_sda_read(I2CDev_t *dev);
static int8_t __i2c_start(I2CDev_t *dev);
static int8_t __i2c_stop(I2CDev_t *dev);
static int8_t __i2c_send_byte(I2CDev_t *dev, uint8_t byte);
static uint8_t __i2c_recv_byte(I2CDev_t *dev);
static int8_t __i2c_send_ack(I2CDev_t *dev, uint8_t ack);
static uint8_t __i2c_recv_ack(I2CDev_t *dev);
static int8_t __i2c_read_reg(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
static int8_t __i2c_read_regs(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
static int8_t __i2c_write_reg(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
static int8_t __i2c_write_regs(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
static int8_t __i2c_deinit(I2CDev_t *dev);
	
/******************************************************************************
 * @brief	初始化软件I2C
 * @param	dev	:  I2CDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t i2c_init(I2CDev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 配置时钟与GPIO */
	__i2c_config_gpio_clock_enable(dev->config.scl_port);
	__i2c_config_gpio_clock_enable(dev->config.sda_port);
	
	__i2c_config_io_out_od(dev->config.scl_port, dev->config.scl_pin);	// SCL开漏输出
	__i2c_config_io_out_od(dev->config.sda_port, dev->config.sda_pin);	// SDA开漏输出
	
	/* 函数指针赋值 */
	dev->start = __i2c_start;
	dev->stop = __i2c_stop;
	dev->send_byte = __i2c_send_byte;
	dev->recv_byte = __i2c_recv_byte;
	dev->send_ack = __i2c_send_ack;
	dev->recv_ack = __i2c_recv_ack;
	dev->deinit = __i2c_deinit;
	dev->read_reg = __i2c_read_reg;
	dev->read_regs = __i2c_read_regs;
	dev->write_reg = __i2c_write_reg;
	dev->write_regs = __i2c_write_regs;
	
	/* 起始SCL与SDA均置高电平 */
	__i2c_scl_write(dev, 1);
	__i2c_sda_write(dev, 1);
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	软件I2C SCL线置高/低电平
 * @param	dev	:  I2CDev_t 结构体指针
 * @param	level	:  要写入的电平值，取1/0
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_scl_write(I2CDev_t *dev, uint8_t level)
{
	if (!dev->init_flag)
		return -1;
	
	__i2c_io_write(dev->config.scl_port, dev->config.scl_pin, level);
	delay_us(1);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C SDA线置高/低电平
 * @param	dev	:  I2CDev_t 结构体指针
 * @param	level	:  要写入的电平值，取1/0
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_sda_write(I2CDev_t *dev, uint8_t level)
{
	if (!dev->init_flag)
		return -1;
	
	__i2c_io_write(dev->config.sda_port, dev->config.sda_pin, level);
	delay_us(1);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C SDA线读高/低电平
 * @param	dev	:  I2CDev_t 结构体指针
 * @return	SDA的电平值，为1/0
 ******************************************************************************/
static uint8_t __i2c_sda_read(I2CDev_t *dev)
{
	uint8_t level;
	level = __i2c_io_read(dev->config.sda_port, dev->config.sda_pin);
	delay_us(1);
	
	return level;
}

/******************************************************************************
 * @brief	软件I2C起始
 * @param	dev	:  I2CDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_start(I2CDev_t *dev)
{
	if (!dev)
		return -1;
	
	__i2c_sda_write(dev, 1);
	__i2c_scl_write(dev, 1);
	__i2c_sda_write(dev, 0);
	__i2c_scl_write(dev, 0);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C停止
 * @param	dev	:  I2CDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_stop(I2CDev_t *dev)
{
	if (!dev)
		return -1;
	
	__i2c_scl_write(dev, 0);
	__i2c_sda_write(dev, 0);
	__i2c_scl_write(dev, 1);
	__i2c_sda_write(dev, 1);
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C发送一个字节
 * @param	dev	:  I2CDev_t 结构体指针
 * @param	byte	:  要发送的字节
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_send_byte(I2CDev_t *dev, uint8_t byte)
{
	if (!dev)
		return -1;
	
	for(uint8_t i = 0;i < 8;i++)
	{
		__i2c_sda_write(dev, (byte & (0x80 >> i)));		// 高位先行
		__i2c_scl_write(dev, 1);						// SCL原来是低电平，放完数据拉高SCL，等待从机读取
		__i2c_scl_write(dev, 0);						// 从机读取完毕，SCL置低电平，完成一位发送
	}
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C接收一个字节
 * @param	dev	:  I2CDev_t 结构体指针
 * @return	接收到的字节
 ******************************************************************************/
static uint8_t __i2c_recv_byte(I2CDev_t *dev)
{
	uint8_t data = 0x00;
	int8_t i;
	
	__i2c_sda_write(dev, 1);					// 主机释放SDA，等待从机放入数据
	for(i = 7; i >= 0; i--)
	{
		__i2c_scl_write(dev, 1);				// SCL原来是低电平，从机放完数据SCL置高电平，主机开始读取
		data |= (__i2c_sda_read(dev) << i);		// 高位先行
		__i2c_scl_write(dev, 0);				// 主机读取完毕，SCL置低电平，完成一位接收
	}
	
	return data;
}

/******************************************************************************
 * @brief	软件I2C发送应答位
 * @param	dev	:  I2CDev_t 结构体指针
 * @param	ack		:  要发送的应答位
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_send_ack(I2CDev_t *dev, uint8_t ack)
{
	if (!dev)
		return -1;
	
	__i2c_sda_write(dev, ack);
	__i2c_scl_write(dev, 1);		// SCL原来是低电平，放完数据拉高SCL，等待从机读取
	__i2c_scl_write(dev, 0);		// 从机读取完毕，SCL置低电平，完成发送
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C接收应答位
 * @param	dev	:  I2CDev_t 结构体指针
 * @return	接收到的应答位，0为应答，1为非应答
 ******************************************************************************/
static uint8_t __i2c_recv_ack(I2CDev_t *dev)
{
	uint8_t ack;
	
	__i2c_sda_write(dev, 1);		// 主机释放SDA，等待从机放入数据
	__i2c_scl_write(dev, 1);		// SCL原来是低电平，从机放完数据SCL置高电平，主机开始读取
	ack = __i2c_sda_read(dev);
	__i2c_scl_write(dev, 0);		// 主机读取完毕，SCL置低电平，完成接收
	
	return ack;
}

/******************************************************************************
 * @brief	软件I2C读寄存器
 * @param	dev			:   I2CDev_t 结构体指针
 * @param	dev_addr    :   I2C设备从机地址
 * @param	reg_addr    :   要读的寄存器地址
 * @param	data    	:   要读的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_read_reg(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	__i2c_start(dev);								// I2C起始
	__i2c_send_byte(dev, dev_addr << 1);			// 发送从机地址，读写位为0，表示即将写入
	__i2c_recv_ack(dev);							// 接收应答
	__i2c_send_byte(dev, reg_addr);					// 发送寄存器地址
	__i2c_recv_ack(dev);							// 接收应答
	
	__i2c_start(dev);								// I2C重复起始
	__i2c_send_byte(dev, (dev_addr << 1) | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	__i2c_recv_ack(dev);							// 接收应答
	*data = __i2c_recv_byte(dev);					// 接收指定寄存器的数据
	__i2c_send_ack(dev, 1);							// 发送应答，给从机非应答，终止从机的数据输出
	__i2c_stop(dev);								// I2C终止		
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C读多个寄存器
 * @param	dev    		:   I2CDev_t 结构体指针
 * @param	dev_addr    :   I2C设备从机地址
 * @param	reg_addr    :   要读的寄存器首地址
 * @param	num			:   要读的寄存器个数
 * @param	data    	:   要读的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_read_regs(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[])
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint16_t i;
	
	__i2c_start(dev);								// I2C起始
	__i2c_send_byte(dev, dev_addr << 1);			// 发送从机地址，读写位为0，表示即将写
	__i2c_recv_ack(dev);							// 接收应答

	__i2c_send_byte(dev, reg_addr); 				// 发送寄存器地址
	__i2c_recv_ack(dev);

	__i2c_start(dev); 								// 重复起始条件
    __i2c_send_byte(dev, (dev_addr << 1) | 0x01); 	// 读模式
	__i2c_recv_ack(dev);

	for (i = 0; i < num; i++)
    {
        data[i] = __i2c_recv_byte(dev);				// 接收数据
        __i2c_send_ack(dev, (i == num - 1) ? 1 : 0);	
    }

	__i2c_stop(dev);								// I2C终止
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C写寄存器
 * @param	dev			:	I2CDev_t 结构体指针
 * @param	dev_addr    :  	I2C设备从机地址
 * @param	reg_addr    :  	要写入的寄存器地址
 * @param	data		:	要写入寄存器的数据，范围：0x00~0xFF
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_write_reg(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	if (!dev || !dev->init_flag)
		return -1;

	__i2c_start(dev);						// I2C起始							
	__i2c_send_byte(dev, dev_addr << 1);	// 发送从机地址，读写位为0，表示即将写入
	__i2c_recv_ack(dev);					// 接收应答
	__i2c_send_byte(dev, reg_addr);			// 发送寄存器地址
	__i2c_recv_ack(dev);					// 接收应答
	__i2c_send_byte(dev, data);				// 发送要写入寄存器的数据
	__i2c_recv_ack(dev);					// 接收应答
	__i2c_stop(dev);						// I2C终止	
	
	return 0;
}

/******************************************************************************
 * @brief	软件I2C写多个寄存器
 * @param	dev			:  I2CDev_t 结构体指针
 * @param	dev_addr    :  I2C设备从机地址
 * @param	reg_addr    :  要写入的寄存器首地址
 * @param	num			:  要写入的寄存器个数
 * @param	data		:  要写入的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_write_regs(I2CDev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[])
{
	if (!dev || !dev->init_flag)
		return -1;

	uint16_t i;

    __i2c_start(dev);								// I2C起始
	__i2c_send_byte(dev, dev_addr << 1);			// 发送从机地址，读写位为0，表示即将写入
	__i2c_recv_ack(dev);							// 接收应答
	__i2c_send_byte(dev, reg_addr);	        		// 发送寄存器地址
	__i2c_recv_ack(dev);							// 接收应答

	for (i = 0; i < num; i++)
	{
		__i2c_send_byte(dev, data[i]);				// 发送要写入寄存器的数据
		__i2c_recv_ack(dev);						// 接收应答
	}
	
	__i2c_stop(dev);								// I2C终止

	return 0;
}

/******************************************************************************
 * @brief	去初始化软件I2C
 * @param	dev   :  I2CDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __i2c_deinit(I2CDev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}
