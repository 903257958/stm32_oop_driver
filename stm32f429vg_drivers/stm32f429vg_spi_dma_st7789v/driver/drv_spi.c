#include "drv_spi.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

/**
 * @brief	使能 SPI 时钟
 * @param[in] spi_periph SPI 外设
 */
static void spi_hw_spi_clock_enable(spi_periph_t spi_periph)
{
#if DRV_SPI_PLATFORM_STM32F1 || DRV_SPI_PLATFORM_STM32F4
	switch ((uint32_t)spi_periph) {
	case (uint32_t)SPI1: RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); break;
	case (uint32_t)SPI2: RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); break;
	case (uint32_t)SPI3: RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE); break;
	default: break;
    }
#elif DRV_SPI_PLATFORM_GD32F1
	switch ((uint32_t)spi_periph) {
	case (uint32_t)SPI0: rcu_periph_clock_enable(RCU_SPI0); break;
	case (uint32_t)SPI1: rcu_periph_clock_enable(RCU_SPI1); break;
	case (uint32_t)SPI2: rcu_periph_clock_enable(RCU_SPI2); break;
	default: break;
    }
#endif
}

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void spi_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_SPI_PLATFORM_STM32F1
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
#elif DRV_SPI_PLATFORM_STM32F4
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
#elif DRV_SPI_PLATFORM_GD32F1
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

#if DRV_SPI_PLATFORM_STM32F4
/**
 * @brief	获取 GPIO 引脚源（STM32F4特有）
 * @param[in] pin GPIO 引脚
 * @return	引脚源编号（如9对应PinSource9）
 */
static inline uint8_t spi_hw_get_gpio_pin_source(gpio_pin_t pin)
{
    for (uint8_t i = 0; i < 16; i++)
        if (pin & (1 << i))
			return i;  // GPIO_Pin = 1<<i，对应 PinSource = i
    return 0xFF;
}

/**
 * @brief	获取 GPIO AF（STM32F4特有）
 * @param[in] spi_periph SPI 外设
 * @return	GPIO AF
 */
static inline uint8_t spi_hw_get_gpio_af(spi_periph_t spi_periph)
{
    switch ((uint32_t)spi_periph) {
	case (uint32_t)SPI1: return GPIO_AF_SPI1;
	case (uint32_t)SPI2: return GPIO_AF_SPI2;
	case (uint32_t)SPI3: return GPIO_AF_SPI3;
	default: return 0xFF;
	}
}

#endif	/* DRV_SPI_PLATFORM_STM32F4 */

/**
 * @brief	初始化 GPIO
 * @param[in] cfg spi_cfg_t 结构体指针
 */
static void spi_hw_gpio_init(const spi_cfg_t *cfg)
{
#if DRV_SPI_PLATFORM_STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin   = cfg->sck_pin;
	GPIO_Init(cfg->sck_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->mosi_pin;
	GPIO_Init(cfg->mosi_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin   = cfg->miso_pin;
	GPIO_Init(cfg->miso_port, &GPIO_InitStructure);

#elif DRV_SPI_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin   = cfg->sck_pin;
	GPIO_Init(cfg->sck_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->mosi_pin;
	GPIO_Init(cfg->mosi_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->miso_pin;
	GPIO_Init(cfg->miso_port, &GPIO_InitStructure);

	uint8_t gpio_af = spi_hw_get_gpio_af(cfg->spi_periph);
	GPIO_PinAFConfig(cfg->sck_port, spi_hw_get_gpio_pin_source(cfg->sck_pin), gpio_af);
	GPIO_PinAFConfig(cfg->mosi_port, spi_hw_get_gpio_pin_source(cfg->mosi_pin), gpio_af);
	GPIO_PinAFConfig(cfg->miso_port, spi_hw_get_gpio_pin_source(cfg->miso_pin), gpio_af);

#elif DRV_SPI_PLATFORM_GD32F1
	gpio_init(cfg->sck_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, cfg->sck_pin);
	gpio_init(cfg->mosi_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, cfg->mosi_pin);
	gpio_init(cfg->miso_port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, cfg->miso_pin);
#endif
}

/**
 * @brief	初始化 SPI
 * @param[in] cfg spi_cfg_t 结构体指针
 */
static void spi_hw_spi_init(const spi_cfg_t *cfg)
{
#if DRV_SPI_PLATFORM_STM32F1 || DRV_SPI_PLATFORM_STM32F4
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						// 主机
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	// 双线全双工
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					// 8位数据帧
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					// 高位先行
	SPI_InitStructure.SPI_BaudRatePrescaler = cfg->prescaler;			// 分频系数
	if (cfg->mode == SPI_MODE_0 || SPI_MODE_1)
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	else
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	if (cfg->mode == SPI_MODE_0 || SPI_MODE_2)
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	else
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	// 软件NSS
	SPI_InitStructure.SPI_CRCPolynomial = 7;	// CRC，不使用
	SPI_Init(cfg->spi_periph, &SPI_InitStructure);
	SPI_Cmd(cfg->spi_periph, ENABLE);

#elif DRV_SPI_PLATFORM_GD32F1
	spi_i2s_deinit(cfg->spi_periph);
	spi_parameter_struct spi_struct;
	spi_struct.device_mode = SPI_MASTER;				// 主机
	spi_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;	// 全双工
	spi_struct.frame_size = SPI_FRAMESIZE_8BIT;			// 8位数据帧
	spi_struct.nss = SPI_NSS_SOFT;						// 软件NSS
	spi_struct.endian = SPI_ENDIAN_MSB;					// 高位先行
	if (cfg->mode == SPI_MODE_0)	  spi_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	else if (cfg->mode == SPI_MODE_1) spi_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
	else if (cfg->mode == SPI_MODE_2) spi_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
	else if (cfg->mode == SPI_MODE_3) spi_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
	spi_struct.prescale = cfg->prescaler;				// 分频系数
	spi_init(cfg->spi_periph, &spi_struct);
	spi_enable(cfg->spi_periph);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void spi_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_SPI_PLATFORM_STM32F1 || DRV_SPI_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#elif DRV_SPI_PLATFORM_GD32F1
	gpio_bit_write(port, pin, (bit_status)(level));
#endif
}

/**
 * @brief	SPI 获取标志位状态
 * @param[in] spi_periph SPI 外设
 * @param[in] flag 		 标志位
 * @return	true 表示SET，false 表示RESET
 */
static inline bool spi_hw_get_flag_status(spi_periph_t spi_periph, uint16_t flag)
{
#if DRV_SPI_PLATFORM_STM32F1 || DRV_SPI_PLATFORM_STM32F4
	return SPI_I2S_GetFlagStatus(spi_periph, flag);
#elif DRV_SPI_PLATFORM_GD32F1
	return spi_i2s_flag_get(spi_periph, flag);
#endif
}

/**
 * @brief	SPI 发送数据
 * @param[in] spi_periph SPI 外设
 * @param[in] data 		 发送的数据
 */
static inline void spi_hw_send_data(spi_periph_t spi_periph, uint8_t data)
{
#if DRV_SPI_PLATFORM_STM32F1 || DRV_SPI_PLATFORM_STM32F4
	SPI_I2S_SendData(spi_periph, data);
#elif DRV_SPI_PLATFORM_GD32F1
	spi_i2s_data_transmit(spi_periph, data);
#endif
}

/**
 * @brief	SPI 接收数据
 * @param[in]  spi_periph SPI 外设
 * @return	接收的数据
 */
static inline uint16_t spi_hw_recv_data(spi_periph_t spi_periph)
{
#if DRV_SPI_PLATFORM_STM32F1 || DRV_SPI_PLATFORM_STM32F4
	return SPI_I2S_ReceiveData(spi_periph);
#elif DRV_SPI_PLATFORM_GD32F1
	return spi_i2s_data_receive(spi_periph);
#endif
}

/**
 * @brief   初始化 SPI 硬件
 * @param[in] cfg spi_cfg_t 结构体指针
 */
static void spi_hw_init(const spi_cfg_t *cfg)
{
	spi_hw_spi_clock_enable(cfg->spi_periph);
	spi_hw_gpio_clock_enable(cfg->sck_port);
	spi_hw_gpio_clock_enable(cfg->miso_port);
	spi_hw_gpio_clock_enable(cfg->mosi_port);

	spi_hw_gpio_init(cfg);
	spi_hw_spi_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */
								
static int spi_start_impl(spi_dev_t *dev, gpio_port_t cs_port, gpio_pin_t cs_pin);
static int spi_stop_impl(spi_dev_t *dev, gpio_port_t cs_port, gpio_pin_t cs_pin);
static int spi_swap_byte_impl(spi_dev_t *dev, uint8_t send, uint8_t *recv);
static int spi_deinit_impl(spi_dev_t *dev);

/* 操作接口表 */
static const spi_ops_t spi_ops = {
	.start     = spi_start_impl, 
	.stop      = spi_stop_impl, 
	.swap_byte = spi_swap_byte_impl,
	.deinit    = spi_deinit_impl,
};
							
/**
 * @brief   初始化 SPI 驱动
 * @param[out] dev spi_dev_t 结构体指针
 * @param[in]  cfg spi_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_spi_init(spi_dev_t *dev, const spi_cfg_t *cfg)
{
	if (!dev || !cfg)
        return -EINVAL;
	
	dev->cfg = *cfg;
	dev->ops = &spi_ops;
	
	spi_hw_init(cfg);
	return 0;
}

/**
 * @brief   SPI 起始
 * @param[in] dev     spi_dev_t 结构体指针
 * @param[in] cs_port 片选线端口
 * @param[in] cs_pin  片选线引脚
 * @return	0 表示成功，其他值表示失败
 */
static int spi_start_impl(spi_dev_t *dev, gpio_port_t cs_port, gpio_pin_t cs_pin)
{
	if (!dev)
		return -EINVAL;

	spi_hw_gpio_write_bit(cs_port, cs_pin, 0);
	return 0;
}

/**
 * @brief   SPI 停止
 * @param[in] dev   spi_dev_t 结构体指针
 * @param[in] cs_port 片选线端口
 * @param[in] cs_pin  片选线引脚
 * @return	0 表示成功，其他值表示失败
 */
static int spi_stop_impl(spi_dev_t *dev, gpio_port_t cs_port, gpio_pin_t cs_pin)
{
	if (!dev)
		return -EINVAL;

	spi_hw_gpio_write_bit(cs_port, cs_pin, 1);
	return 0;
}

/**
 * @brief   SPI 交换一个字节
 * @param[in]  dev  spi_dev_t 结构体指针
 * @param[in]  send 发送的字节
 * @param[out] recv 接收的字节
 * @return	0 表示成功，其他值表示失败
 */
static int spi_swap_byte_impl(spi_dev_t *dev, uint8_t send, uint8_t *recv)
{
	if (!dev)
		return -EINVAL;	

	while (!spi_hw_get_flag_status(dev->cfg.spi_periph, SPI_I2S_FLAG_TXE));	// 等待TXE置1，表示发送寄存器为空，发送一个字节	
	spi_hw_send_data(dev->cfg.spi_periph, send);							// 发送字节
	while(!spi_hw_get_flag_status(dev->cfg.spi_periph, SPI_I2S_FLAG_RXNE));	// 等待RXNE置1，表示接收寄存器非空，收到一个字节
	uint16_t recv_raw = spi_hw_recv_data(dev->cfg.spi_periph);				// 读取接收到的字节

	if (recv != NULL) 
        *recv = (uint8_t)(recv_raw & 0xFF);
	
	return 0;
}

/**
 * @brief   去初始化 SPI
 * @param[in] dev spi_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int spi_deinit_impl(spi_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/* ------------------------------- 核心驱动层结束 ------------------------------- */
