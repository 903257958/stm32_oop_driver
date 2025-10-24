#include "i2c_soft.h"

#ifdef USE_STDPERIPH_DRIVER

/**************************** STM32F1 系列 ****************************/
#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define i2c_soft_io_clock_enable(port)                     						   	   \
    do {                                                   						   	   \
        if (port == GPIOA)       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); \
        else if (port == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); \
        else if (port == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); \
        else if (port == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); \
        else if (port == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); \
        else if (port == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); \
        else if (port == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); \
    } while (0)

#define i2c_soft_config_io_out_od(port, pin)               	\
    do {                                                   	\
        GPIO_InitTypeDef GPIO_InitStructure;               	\
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;  	\
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  	\
        GPIO_InitStructure.GPIO_Pin   = pin;               	\
        GPIO_Init(port, &GPIO_InitStructure);              	\
    } while (0)

#define i2c_soft_io_write(port, pin, val)	GPIO_WriteBit(port, pin, (BitAction)(val))
#define i2c_soft_io_read(port, pin)			GPIO_ReadInputDataBit(port, pin)

/**************************** STM32F4 系列 ****************************/
#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define i2c_soft_io_clock_enable(port)                     							   \
    do {                                                   							   \
        if (port == GPIOA)       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); \
        else if (port == GPIOB)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); \
        else if (port == GPIOC)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); \
        else if (port == GPIOD)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); \
        else if (port == GPIOE)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); \
        else if (port == GPIOF)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); \
        else if (port == GPIOG)  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); \
    } while (0)

#define i2c_soft_config_io_out_od(port, pin)               	\
    do {                                                   	\
        GPIO_InitTypeDef GPIO_InitStructure;               	\
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     	\
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;     	\
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	\
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	\
        GPIO_InitStructure.GPIO_Pin   = pin;              	\
        GPIO_Init(port, &GPIO_InitStructure);             	\
    } while (0)

#define i2c_soft_io_write(port, pin, val)	GPIO_WriteBit(port, pin, (BitAction)(val))
#define i2c_soft_io_read(port, pin)			GPIO_ReadInputDataBit(port, pin)

/**************************** GD32F1 系列 ****************************/
#elif defined(GD32F10X_MD) || defined(GD32F10X_HD)

#define i2c_soft_io_clock_enable(port)                     			 \
    do {                                                   			 \
        if (port == GPIOA)       rcu_periph_clock_enable(RCU_GPIOA); \
        else if (port == GPIOB)  rcu_periph_clock_enable(RCU_GPIOB); \
        else if (port == GPIOC)  rcu_periph_clock_enable(RCU_GPIOC); \
        else if (port == GPIOD)  rcu_periph_clock_enable(RCU_GPIOD); \
        else if (port == GPIOE)  rcu_periph_clock_enable(RCU_GPIOE); \
        else if (port == GPIOF)  rcu_periph_clock_enable(RCU_GPIOF); \
        else if (port == GPIOG)  rcu_periph_clock_enable(RCU_GPIOG); \
    } while (0)

#define i2c_soft_config_io_out_od(port, pin)	\
    	gpio_init(port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, pin)

#define i2c_soft_io_write(port, pin, val)	gpio_bit_write(port, pin, (bit_status)(val))
#define i2c_soft_io_read(port, pin)			gpio_input_bit_get(port, pin)

#endif	/* MCU SERIES SELECTION */

#endif  /* USE_STDPERIPH_DRIVER */

static int i2c_soft_scl_write(i2c_soft_dev_t *dev, uint8_t level);
static int i2c_soft_sda_write(i2c_soft_dev_t *dev, uint8_t level);
static int i2c_soft_sda_read(i2c_soft_dev_t *dev, uint8_t *level);
static int i2c_soft_start(i2c_soft_dev_t *dev);
static int i2c_soft_stop(i2c_soft_dev_t *dev);
static int i2c_soft_send_byte(i2c_soft_dev_t *dev, uint8_t byte);
static int i2c_soft_recv_byte(i2c_soft_dev_t *dev, uint8_t *byte);
static int i2c_soft_send_ack(i2c_soft_dev_t *dev, uint8_t ack);
static int i2c_soft_recv_ack(i2c_soft_dev_t *dev, uint8_t *ack);
static int i2c_soft_read_reg(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
static int i2c_soft_read_regs(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
static int i2c_soft_write_reg(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
static int i2c_soft_write_regs(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
static int i2c_soft_drv_deinit(i2c_soft_dev_t *dev);

/**
 * @brief   初始化软件 I2C
 * @param[in,out] dev i2c_soft_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int i2c_soft_drv_init(i2c_soft_dev_t *dev)
{
	if (!dev)
		return -1;
	
	i2c_soft_io_clock_enable(dev->config.scl_port);
	i2c_soft_io_clock_enable(dev->config.sda_port);
	i2c_soft_config_io_out_od(dev->config.scl_port, dev->config.scl_pin);	// 开漏输出
	i2c_soft_config_io_out_od(dev->config.sda_port, dev->config.sda_pin);	// 开漏输出
	
	dev->start = i2c_soft_start;
	dev->stop = i2c_soft_stop;
	dev->send_byte = i2c_soft_send_byte;
	dev->recv_byte = i2c_soft_recv_byte;
	dev->send_ack = i2c_soft_send_ack;
	dev->recv_ack = i2c_soft_recv_ack;
	dev->read_reg = i2c_soft_read_reg;
	dev->read_regs = i2c_soft_read_regs;
	dev->write_reg = i2c_soft_write_reg;
	dev->write_regs = i2c_soft_write_regs;
	dev->deinit = i2c_soft_drv_deinit;

	dev->init_flag = true;
	
	i2c_soft_scl_write(dev, 1);
	i2c_soft_sda_write(dev, 1);
	
	return 0;
}

/**
 * @brief   软件 I2C SCL 线置高/低电平
 * @param[in] dev   i2c_soft_dev_t 结构体指针
 * @param[in] level 要写入的电平值，取 1/0
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_scl_write(i2c_soft_dev_t *dev, uint8_t level)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_io_write(dev->config.scl_port, dev->config.scl_pin, level);
	i2c_soft_delay_us(1);

	return 0;
}

/**
 * @brief   软件 I2C SDA 线置高/低电平
 * @param[in] dev   i2c_soft_dev_t 结构体指针
 * @param[in] level 要写入的电平值，取 1/0
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_sda_write(i2c_soft_dev_t *dev, uint8_t level)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_io_write(dev->config.sda_port, dev->config.sda_pin, level);
	i2c_soft_delay_us(1);

	return 0;
}

/**
 * @brief   软件 I2C SDA 线置高/低电平
 * @param[in]  dev   i2c_soft_dev_t 结构体指针
 * @param[out] level SDA 的电平值，为 1/0
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_sda_read(i2c_soft_dev_t *dev, uint8_t *level)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	*level = i2c_soft_io_read(dev->config.sda_port, dev->config.sda_pin);
	i2c_soft_delay_us(1);
	
	return 0;
}

/**
 * @brief   软件 I2C 起始
 * @param[in] dev i2c_soft_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_start(i2c_soft_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_sda_write(dev, 1);
	i2c_soft_scl_write(dev, 1);
	i2c_soft_sda_write(dev, 0);
	i2c_soft_scl_write(dev, 0);

	return 0;
}

/**
 * @brief   软件 I2C 停止
 * @param[in] dev i2c_soft_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_stop(i2c_soft_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_scl_write(dev, 0);
	i2c_soft_sda_write(dev, 0);
	i2c_soft_scl_write(dev, 1);
	i2c_soft_sda_write(dev, 1);

	return 0;
}

/**
 * @brief   软件 I2C 发送一个字节
 * @param[in] dev  i2c_soft_dev_t 结构体指针
 * @param[in] byte 要发送的字节
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_send_byte(i2c_soft_dev_t *dev, uint8_t byte)
{
	uint8_t i;

	if (!dev || !dev->init_flag)
		return -1;
	
	for (i = 0; i < 8; i++) {
		i2c_soft_sda_write(dev, (byte & (0x80 >> i)));	// 高位先行
		i2c_soft_scl_write(dev, 1);						// SCL原来是低电平，放完数据拉高SCL，等待从机读取
		i2c_soft_scl_write(dev, 0);						// 从机读取完毕，SCL置低电平，完成一位发送
	}

	return 0;
}

/**
 * @brief   软件 I2C 接收一个字节
 * @param[in]  dev  i2c_soft_dev_t 结构体指针
 * @param[out] byte 接收到的字节
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_recv_byte(i2c_soft_dev_t *dev, uint8_t *byte)
{
	uint8_t level;
	int8_t i;

	if (!dev || !dev->init_flag)
		return -1;
	
	*byte = 0;
	i2c_soft_sda_write(dev, 1);			// 主机释放SDA，等待从机放入数据
	for (i = 7; i >= 0; i--) {
		i2c_soft_scl_write(dev, 1);		// SCL原来是低电平，从机放完数据SCL置高电平，主机开始读取
		i2c_soft_sda_read(dev, &level);
		*byte |= (level << i);			// 高位先行
		i2c_soft_scl_write(dev, 0);		// 主机读取完毕，SCL置低电平，完成一位接收
	}
	
	return 0;
}

/**
 * @brief   软件 I2C 发送应答位
 * @param[in] dev i2c_soft_dev_t 结构体指针
 * @param[in] ack 要发送的应答位，0为应答，1为非应答
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_send_ack(i2c_soft_dev_t *dev, uint8_t ack)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_sda_write(dev, ack);
	i2c_soft_scl_write(dev, 1);		// SCL原来是低电平，放完数据拉高SCL，等待从机读取
	i2c_soft_scl_write(dev, 0);		// 从机读取完毕，SCL置低电平，完成发送

	return 0;
}

/**
 * @brief   软件 I2C 接收应答位
 * @param[in]  dev i2c_soft_dev_t 结构体指针
 * @param[out] ack 接收到的应答位，0为应答，1为非应答
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_recv_ack(i2c_soft_dev_t *dev, uint8_t *ack)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_sda_write(dev, 1);		// 主机释放SDA，等待从机放入数据
	i2c_soft_scl_write(dev, 1);		// SCL原来是低电平，从机放完数据SCL置高电平，主机开始读取
	i2c_soft_sda_read(dev, ack);
	i2c_soft_scl_write(dev, 0);		// 主机读取完毕，SCL置低电平，完成接收
	
	return 0;
}

/**
 * @brief   软件 I2C 读寄存器
 * @param[in]  dev 	 	i2c_soft_dev_t 结构体指针
 * @param[in]  dev_addr I2C设备从机地址
 * @param[in]  reg_addr 要读的寄存器地址
 * @param[out] data 	读到的寄存器数据
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_read_reg(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
	uint8_t ack;

	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_start(dev);								// I2C起始
	i2c_soft_send_byte(dev, dev_addr << 1);				// 发送从机地址，读写位为0，表示即将写入
	i2c_soft_recv_ack(dev, &ack);						// 接收应答
	i2c_soft_send_byte(dev, reg_addr);					// 发送寄存器地址
	i2c_soft_recv_ack(dev, &ack);						// 接收应答
	
	i2c_soft_start(dev);								// I2C重复起始
	i2c_soft_send_byte(dev, (dev_addr << 1) | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	i2c_soft_recv_ack(dev, &ack);						// 接收应答
	i2c_soft_recv_byte(dev, data);						// 接收指定寄存器的数据
	i2c_soft_send_ack(dev, 1);							// 发送应答，给从机非应答，终止从机的数据输出
	i2c_soft_stop(dev);									// I2C终止

	return 0;
}

/**
 * @brief   软件 I2C 读多个寄存器
 * @param[in]  dev 	 	i2c_soft_dev_t 结构体指针
 * @param[in]  dev_addr I2C设备从机地址
 * @param[in]  reg_addr 要读的寄存器首地址
 * @param[in]  num		要读的寄存器个数
 * @param[out] data 	读到的寄存器数据
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_read_regs(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[])
{
	uint8_t ack;
	uint16_t i;

	if (!dev || !dev->init_flag)
		return -1;
	
	i2c_soft_start(dev);								// I2C起始
	i2c_soft_send_byte(dev, dev_addr << 1);				// 发送从机地址，读写位为0，表示即将写
	i2c_soft_recv_ack(dev, &ack);						// 接收应答
	i2c_soft_send_byte(dev, reg_addr); 					// 发送寄存器地址
	i2c_soft_recv_ack(dev, &ack);

	i2c_soft_start(dev);								// 重复起始条件
    i2c_soft_send_byte(dev, (dev_addr << 1) | 0x01);	// 读模式
	i2c_soft_recv_ack(dev, &ack);
	for (i = 0; i < num; i++) {
        i2c_soft_recv_byte(dev, &data[i]);				// 接收数据
        i2c_soft_send_ack(dev, (i == num - 1) ? 1 : 0);	
    }
	i2c_soft_stop(dev);									// I2C终止

	return 0;
}

/**
 * @brief   软件 I2C 写寄存器
 * @param[in] dev 	   i2c_soft_dev_t 结构体指针
 * @param[in] dev_addr I2C设备从机地址
 * @param[in] reg_addr 要写入的寄存器地址
 * @param[in] data 	   要写入寄存器的数据，范围：0x00~0xFF
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_write_reg(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	uint8_t ack;

	if (!dev || !dev->init_flag)
		return -1;

	i2c_soft_start(dev);					// I2C起始							
	i2c_soft_send_byte(dev, dev_addr << 1);	// 发送从机地址，读写位为0，表示即将写入
	i2c_soft_recv_ack(dev, &ack);			// 接收应答
	i2c_soft_send_byte(dev, reg_addr);		// 发送寄存器地址
	i2c_soft_recv_ack(dev, &ack);			// 接收应答
	i2c_soft_send_byte(dev, data);			// 发送要写入寄存器的数据
	i2c_soft_recv_ack(dev, &ack);			// 接收应答
	i2c_soft_stop(dev);						// I2C终止

	return 0;
}

/**
 * @brief   软件 I2C 写多个寄存器
 * @param[in] dev 	   i2c_soft_dev_t 结构体指针
 * @param[in] dev_addr I2C设备从机地址
 * @param[in] reg_addr 要写入的寄存器首地址
 * @param[in] num	   要写入的寄存器个数
 * @param[in] data 	   要写入的寄存器数据，范围：0x00~0xFF
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_write_regs(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[])
{
	uint8_t ack;
	uint16_t i;

	if (!dev || !dev->init_flag)
		return -1;

    i2c_soft_start(dev);					// I2C起始
	i2c_soft_send_byte(dev, dev_addr << 1);	// 发送从机地址，读写位为0，表示即将写入
	i2c_soft_recv_ack(dev, &ack);			// 接收应答
	i2c_soft_send_byte(dev, reg_addr);	    // 发送寄存器地址
	i2c_soft_recv_ack(dev, &ack);			// 接收应答

	for (i = 0; i < num; i++) {
		i2c_soft_send_byte(dev, data[i]);	// 发送要写入寄存器的数据
		i2c_soft_recv_ack(dev, &ack);		// 接收应答
	}
	
	i2c_soft_stop(dev);						// I2C终止

	return 0;
}

/**
 * @brief   去初始化软件 I2C
 * @param[in,out] dev i2c_soft_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_drv_deinit(i2c_soft_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	dev->init_flag = false;

    return 0;
}
