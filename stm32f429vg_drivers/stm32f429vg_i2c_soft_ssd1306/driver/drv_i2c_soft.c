#include "drv_i2c_soft.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void i2c_soft_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_I2C_SOFT_PLATFORM_STM32F1
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); break;
	case (uint32_t)GPIOB: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); break;
	case (uint32_t)GPIOC: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); break;
	case (uint32_t)GPIOD: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); break;
	case (uint32_t)GPIOE: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); break;
	case (uint32_t)GPIOF: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); break;
	case (uint32_t)GPIOG: RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_I2C_SOFT_PLATFORM_STM32F4
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); break;
	case (uint32_t)GPIOB: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); break;
	case (uint32_t)GPIOC: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); break;
	case (uint32_t)GPIOD: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); break;
	case (uint32_t)GPIOE: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); break;
	case (uint32_t)GPIOF: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); break;
	case (uint32_t)GPIOG: RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); break;
	default: break;
    }
#elif DRV_I2C_SOFT_PLATFORM_GD32F1
	switch ((uint32_t)port) {
	case (uint32_t)GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
	case (uint32_t)GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
	case (uint32_t)GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
	case (uint32_t)GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
	case (uint32_t)GPIOE: rcu_periph_clock_enable(RCU_GPIOE); break;
	case (uint32_t)GPIOF: rcu_periph_clock_enable(RCU_GPIOF); break;
	case (uint32_t)GPIOG: rcu_periph_clock_enable(RCU_GPIOG); break;
	default: break;
    }
#endif
}

/**
 * @brief	初始化 GPIO
 * @param[in] cfg i2c_soft_cfg_t 结构体指针
 */
static void i2c_soft_hw_gpio_init(const i2c_soft_cfg_t *cfg)
{
#if DRV_I2C_SOFT_PLATFORM_STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->scl_pin;
	GPIO_Init(cfg->scl_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->sda_pin;
	GPIO_Init(cfg->sda_port, &GPIO_InitStructure);
#elif DRV_I2C_SOFT_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->scl_pin;
	GPIO_Init(cfg->scl_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->sda_pin;
	GPIO_Init(cfg->sda_port, &GPIO_InitStructure);
#elif DRV_I2C_SOFT_PLATFORM_GD32F1
	gpio_init(cfg->scl_port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, cfg->scl_pin);
	gpio_init(cfg->sda_port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, cfg->sda_pin);
#endif
}

/**
 * @brief	GPIO 读输入引脚电平
 * @param[in] port 端口
 * @param[in] pin  引脚
 * @return	电平
 */
static inline uint8_t i2c_soft_hw_gpio_read_in_bit(gpio_port_t port, gpio_pin_t pin)
{
#if DRV_I2C_SOFT_PLATFORM_STM32F1 || DRV_I2C_SOFT_PLATFORM_STM32F4
	return GPIO_ReadInputDataBit(port, pin);
#elif DRV_I2C_SOFT_PLATFORM_GD32F1
	return (uint8_t)gpio_input_bit_get(port, pin);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void i2c_soft_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_I2C_SOFT_PLATFORM_STM32F1 || DRV_I2C_SOFT_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#elif DRV_I2C_SOFT_PLATFORM_GD32F1
	gpio_bit_write(port, pin, (bit_status)(level));
#endif
}

/**
 * @brief   初始化软件 I2C 硬件
 * @param[in] cfg i2c_soft_cfg_t 结构体指针
 */
static void i2c_soft_hw_init(const i2c_soft_cfg_t *cfg)
{
	i2c_soft_hw_gpio_clock_enable(cfg->scl_port);
	i2c_soft_hw_gpio_clock_enable(cfg->sda_port);

	i2c_soft_hw_gpio_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int i2c_soft_scl_write(i2c_soft_dev_t *dev, uint8_t level);
static int i2c_soft_sda_write(i2c_soft_dev_t *dev, uint8_t level);
static int i2c_soft_sda_read(i2c_soft_dev_t *dev, uint8_t *level);
static int i2c_soft_start_impl(i2c_soft_dev_t *dev);
static int i2c_soft_stop_impl(i2c_soft_dev_t *dev);
static int i2c_soft_send_byte_impl(i2c_soft_dev_t *dev, uint8_t byte);
static int i2c_soft_recv_byte_impl(i2c_soft_dev_t *dev, uint8_t *byte);
static int i2c_soft_send_ack_impl(i2c_soft_dev_t *dev, uint8_t ack);
static int i2c_soft_recv_ack_impl(i2c_soft_dev_t *dev, uint8_t *ack);
static int i2c_soft_read_reg_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
static int i2c_soft_read_regs_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
static int i2c_soft_write_reg_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
static int i2c_soft_write_regs_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[]);
static int i2c_soft_deinit_impl(i2c_soft_dev_t *dev);

/* 操作接口表 */
static const i2c_soft_ops_t i2c_soft_ops = {
	.start      = i2c_soft_start_impl, 
	.stop       = i2c_soft_stop_impl, 
	.send_byte  = i2c_soft_send_byte_impl, 
	.recv_byte  = i2c_soft_recv_byte_impl, 
	.send_ack   = i2c_soft_send_ack_impl, 
	.recv_ack   = i2c_soft_recv_ack_impl, 
	.read_reg   = i2c_soft_read_reg_impl, 
	.read_regs  = i2c_soft_read_regs_impl, 
	.write_reg  = i2c_soft_write_reg_impl, 
	.write_regs = i2c_soft_write_regs_impl, 
	.deinit     = i2c_soft_deinit_impl,
};

/**
 * @brief   初始化软件 I2C 驱动
 * @param[out] dev i2c_soft_dev_t 结构体指针
 * @param[in]  cfg i2c_soft_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_i2c_soft_init(i2c_soft_dev_t *dev, const i2c_soft_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;
	
	dev->cfg = *cfg;
	dev->ops = &i2c_soft_ops;

	i2c_soft_hw_init(cfg);
	
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
	if (!dev)
		return -EINVAL;
	
	i2c_soft_hw_gpio_write_bit(dev->cfg.scl_port, dev->cfg.scl_pin, level);
	dev->cfg.delay_us(dev->cfg.bit_delay_us);
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
	if (!dev)
		return -EINVAL;
	
	i2c_soft_hw_gpio_write_bit(dev->cfg.sda_port, dev->cfg.sda_pin, level);
	dev->cfg.delay_us(dev->cfg.bit_delay_us);
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
	if (!dev)
		return -EINVAL;
	
	*level = i2c_soft_hw_gpio_read_in_bit(dev->cfg.sda_port, dev->cfg.sda_pin);
	dev->cfg.delay_us(dev->cfg.bit_delay_us);
	return 0;
}

/**
 * @brief   软件 I2C 起始
 * @param[in] dev i2c_soft_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_start_impl(i2c_soft_dev_t *dev)
{
	if (!dev)
		return -EINVAL;
	
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
static int i2c_soft_stop_impl(i2c_soft_dev_t *dev)
{
	if (!dev)
		return -EINVAL;
	
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
static int i2c_soft_send_byte_impl(i2c_soft_dev_t *dev, uint8_t byte)
{
	uint8_t i;

	if (!dev)
		return -EINVAL;
	
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
static int i2c_soft_recv_byte_impl(i2c_soft_dev_t *dev, uint8_t *byte)
{
	uint8_t level;
	int8_t i;

	if (!dev)
		return -EINVAL;
	
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
static int i2c_soft_send_ack_impl(i2c_soft_dev_t *dev, uint8_t ack)
{
	if (!dev)
		return -EINVAL;
	
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
static int i2c_soft_recv_ack_impl(i2c_soft_dev_t *dev, uint8_t *ack)
{
	if (!dev)
		return -EINVAL;
	
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
static int i2c_soft_read_reg_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
	uint8_t ack;

	if (!dev)
		return -EINVAL;
	
	i2c_soft_start_impl(dev);								// I2C起始
	i2c_soft_send_byte_impl(dev, dev_addr << 1);			// 发送从机地址，读写位为0，表示即将写入
	i2c_soft_recv_ack_impl(dev, &ack);						// 接收应答
	i2c_soft_send_byte_impl(dev, reg_addr);					// 发送寄存器地址
	i2c_soft_recv_ack_impl(dev, &ack);						// 接收应答
	
	i2c_soft_start_impl(dev);								// I2C重复起始
	i2c_soft_send_byte_impl(dev, (dev_addr << 1) | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	i2c_soft_recv_ack_impl(dev, &ack);						// 接收应答
	i2c_soft_recv_byte_impl(dev, data);						// 接收指定寄存器的数据
	i2c_soft_send_ack_impl(dev, 1);							// 发送应答，给从机非应答，终止从机的数据输出
	i2c_soft_stop_impl(dev);								// I2C终止

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
static int i2c_soft_read_regs_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[])
{
	uint8_t ack;
	uint16_t i;

	if (!dev)
		return -EINVAL;
	
	i2c_soft_start_impl(dev);								// I2C起始
	i2c_soft_send_byte_impl(dev, dev_addr << 1);			// 发送从机地址，读写位为0，表示即将写
	i2c_soft_recv_ack_impl(dev, &ack);						// 接收应答
	i2c_soft_send_byte_impl(dev, reg_addr); 				// 发送寄存器地址
	i2c_soft_recv_ack_impl(dev, &ack);

	i2c_soft_start_impl(dev);								// 重复起始条件
    i2c_soft_send_byte_impl(dev, (dev_addr << 1) | 0x01);	// 读模式
	i2c_soft_recv_ack_impl(dev, &ack);
	for (i = 0; i < num; i++) {
        i2c_soft_recv_byte_impl(dev, &data[i]);				// 接收数据
        i2c_soft_send_ack_impl(dev, (i == num - 1) ? 1 : 0);	
    }
	i2c_soft_stop_impl(dev);								// I2C终止

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
static int i2c_soft_write_reg_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	uint8_t ack;

	if (!dev)
		return -EINVAL;

	i2c_soft_start_impl(dev);						// I2C起始							
	i2c_soft_send_byte_impl(dev, dev_addr << 1);	// 发送从机地址，读写位为0，表示即将写入
	i2c_soft_recv_ack_impl(dev, &ack);				// 接收应答
	i2c_soft_send_byte_impl(dev, reg_addr);			// 发送寄存器地址
	i2c_soft_recv_ack_impl(dev, &ack);				// 接收应答
	i2c_soft_send_byte_impl(dev, data);				// 发送要写入寄存器的数据
	i2c_soft_recv_ack_impl(dev, &ack);				// 接收应答
	i2c_soft_stop_impl(dev);						// I2C终止
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
static int i2c_soft_write_regs_impl(i2c_soft_dev_t *dev, uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t data[])
{
	uint8_t ack;
	uint16_t i;

	if (!dev)
		return -EINVAL;

    i2c_soft_start_impl(dev);						// I2C起始
	i2c_soft_send_byte_impl(dev, dev_addr << 1);	// 发送从机地址，读写位为0，表示即将写入
	i2c_soft_recv_ack_impl(dev, &ack);				// 接收应答
	i2c_soft_send_byte_impl(dev, reg_addr);	    	// 发送寄存器地址
	i2c_soft_recv_ack_impl(dev, &ack);				// 接收应答

	for (i = 0; i < num; i++) {
		i2c_soft_send_byte_impl(dev, data[i]);		// 发送要写入寄存器的数据
		i2c_soft_recv_ack_impl(dev, &ack);			// 接收应答
	}
	
	i2c_soft_stop_impl(dev);						// I2C终止
	return 0;
}

/**
 * @brief   去初始化软件 I2C
 * @param[in] dev i2c_soft_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int i2c_soft_deinit_impl(i2c_soft_dev_t *dev)
{    
    if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
