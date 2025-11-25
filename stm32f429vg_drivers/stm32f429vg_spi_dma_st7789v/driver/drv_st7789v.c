#include "drv_st7789v.h"
#include "drv_lcd_data.h"
#include <stddef.h>
#include <errno.h>

/* --------------------------------- 硬件抽象层 --------------------------------- */

#if DRV_ST7789V_PLATFORM_STM32F1
#define TIMER_FREQ	72000000									

#elif DRV_ST7789V_PLATFORM_STM32F4
#if defined(STM32F40_41xxx)
#define TIMER_FREQ	84000000

#elif defined(STM32F429_439xx)
#define TIMER_FREQ	90000000

#elif defined(STM32F411xE)
#define TIMER_FREQ	100000000
#endif

#endif

/**
 * @brief	使能 GPIO 端口时钟
 * @param[in] port GPIO 端口
 */
static void st7789v_hw_gpio_clock_enable(gpio_port_t port)
{
#if DRV_ST7789V_PLATFORM_STM32F1
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
#elif DRV_ST7789V_PLATFORM_STM32F4
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
#endif
}

/**
 * @brief	使能 DMA 时钟
 * @param[in] spi_periph SPI 外设
 */
static void st7789v_hw_dma_clock_enable(spi_periph_t spi_periph)
{
#if DRV_ST7789V_PLATFORM_STM32F4
	if (spi_periph == SPI1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	else if (spi_periph == SPI2)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	else if (spi_periph == SPI3)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
#endif
}

/**
 * @brief	初始化 GPIO
 * @param[in] cfg st7789v_cfg_t 结构体指针
 */
static void st7789v_hw_gpio_init(const st7789v_cfg_t *cfg)
{
#if DRV_ST7789V_PLATFORM_STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->cs_pin;
	GPIO_Init(cfg->cs_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->res_pin;
	GPIO_Init(cfg->res_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->dc_pin;
	GPIO_Init(cfg->dc_port, &GPIO_InitStructure);
#elif DRV_ST7789V_PLATFORM_STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = cfg->cs_pin;
	GPIO_Init(cfg->cs_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->res_pin;
	GPIO_Init(cfg->res_port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = cfg->dc_pin;
	GPIO_Init(cfg->dc_port, &GPIO_InitStructure);
#endif
}

/**
 * @brief	写 GPIO 引脚电平
 * @param[in] port  端口
 * @param[in] pin   引脚
 * @param[in] level 电平
 */
static inline void st7789v_hw_gpio_write_bit(gpio_port_t port, gpio_pin_t pin, uint8_t level)
{
#if DRV_ST7789V_PLATFORM_STM32F1 || DRV_ST7789V_PLATFORM_STM32F4
	GPIO_WriteBit(port, pin, (BitAction)level);
#endif
}

#if DRV_ST7789V_PLATFORM_STM32F4
/**
 * @brief	获取 DMA Stream
 * @param[in] spi_periph SPI 外设
 * @return	DMA Stream
 */
static inline dma_stream_t st7789v_hw_get_dma_stream(spi_periph_t spi_periph)
{
	if (spi_periph == SPI1)
		return DMA2_Stream3;
	else if (spi_periph == SPI2)
		return DMA1_Stream4;
	else if (spi_periph == SPI3)
		return DMA1_Stream5;
	else
		return (dma_stream_t)-1;
}

/**
 * @brief	获取 DMA Channel
 * @param[in] spi_periph SPI 外设
 * @return	DMA Channel
 */
static inline dma_channel_t st7789v_hw_get_dma_channel(spi_periph_t spi_periph)
{
	if (spi_periph == SPI1)
		return DMA_Channel_3;
	else if (spi_periph == SPI2)
		return DMA_Channel_0;
	else if (spi_periph == SPI3)
		return DMA_Channel_0;
	else
		return (dma_channel_t)0;
}

/**
 * @brief	获取 DMA Flag
 * @param[in] spi_periph SPI 外设
 * @return	DMA Flag
 */
static inline uint32_t st7789v_hw_get_dma_flag(spi_periph_t spi_periph)
{
	if (spi_periph == SPI1)
		return DMA_FLAG_TCIF3;
	else if (spi_periph == SPI2)
		return DMA_FLAG_TCIF4;
	else if (spi_periph == SPI3)
		return DMA_FLAG_TCIF5;
	else
		return 0;
}
#endif

/**
 * @brief   初始化 ST7789V DMA 传输
 * @details 用于 LVGL 等图形库
 * @param[in] cfg 			st7789v_cfg_t 结构体指针
 * @param[in] disp_buf_addr 显示缓冲区地址
 */
void st7789v_hw_dma_init(const st7789v_cfg_t *cfg, uint32_t disp_buf_addr)
{
	#if DRV_ST7789V_PLATFORM_STM32F4
    st7789v_hw_dma_clock_enable(cfg->spi_periph);	// 开启DMA时钟
	DMA_DeInit(st7789v_hw_get_dma_stream(cfg->spi_periph));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = st7789v_hw_get_dma_channel(cfg->spi_periph);// 选择DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&cfg->spi_periph->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)disp_buf_addr;			// 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;						// 方向：从内存到外设
    DMA_InitStructure.DMA_BufferSize = 0;										// 传输大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 外设地址不增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// 内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		// 外设数据单位8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				// 内存数据单位8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								// 工作在正常模式，一次传输后自动结束
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;						// 优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;						// 禁用FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;				// FIFO阈值为满
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					// 内存突发传输为单次
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			// 外设突发传输为单次
    DMA_Init(st7789v_hw_get_dma_stream(cfg->spi_periph), &DMA_InitStructure);
    DMA_ClearFlag(st7789v_hw_get_dma_stream(cfg->spi_periph), st7789v_hw_get_dma_flag(cfg->spi_periph));		
    DMA_Cmd(st7789v_hw_get_dma_stream(cfg->spi_periph), DISABLE);
	#endif
}

/**
 * @brief   ST7789V SPI 进行一次 DMA 传输
 * @param[in] spi_periph SPI 外设
 * @param[in] size 		 传输大小
 */
static void st7789v_hw_spi_dma_transfer(spi_periph_t spi_periph, uint16_t size)
{
#if DRV_ST7789V_PLATFORM_STM32F4
	/* 开启SPI的DMA接收 */
	SPI_I2S_DMACmd(spi_periph, SPI_I2S_DMAReq_Tx, ENABLE);

	/* 使能DMA */
	DMA_Cmd(st7789v_hw_get_dma_stream(spi_periph), DISABLE);
	while (DMA_GetCmdStatus(st7789v_hw_get_dma_stream(spi_periph)) != DISABLE);
	DMA_SetCurrDataCounter(st7789v_hw_get_dma_stream(spi_periph), size);
	DMA_Cmd(st7789v_hw_get_dma_stream(spi_periph), ENABLE);
	
	/* 等待传输完成 */
	while (DMA_GetFlagStatus(st7789v_hw_get_dma_stream(spi_periph),
							 st7789v_hw_get_dma_flag(spi_periph)) == RESET);

	/* 清除标志位 */
	DMA_ClearFlag(st7789v_hw_get_dma_stream(spi_periph),
				  st7789v_hw_get_dma_flag(spi_periph));
#endif
}

/**
 * @brief   初始化 ST7789V 硬件
 * @param[in] cfg st7789v_cfg_t 结构体指针
 */
static void st7789v_hw_init(const st7789v_cfg_t *cfg)
{
	st7789v_hw_gpio_clock_enable(cfg->cs_port);
	st7789v_hw_gpio_clock_enable(cfg->res_port);
	st7789v_hw_gpio_clock_enable(cfg->dc_port);
	
	st7789v_hw_gpio_init(cfg);
}
/* ------------------------------- 硬件抽象层结束 ------------------------------- */

/* --------------------------------- 核心驱动层 --------------------------------- */

static int st7789v_cs_write(st7789v_dev_t *dev, uint8_t level);
static int st7789v_res_write(st7789v_dev_t *dev, uint8_t level);
static int st7789v_dc_write(st7789v_dev_t *dev, uint8_t level);
static int st7789v_reset(st7789v_dev_t *dev);
static int st7789v_write_cmd(st7789v_dev_t *dev, uint8_t cmd);
static int st7789v_write_byte(st7789v_dev_t *dev, uint8_t data);
static int st7789v_write_halfword(st7789v_dev_t *dev, uint16_t halfword);
static int st7789v_set_direction(st7789v_dev_t *dev);
static int st7789v_set_windows(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static int st7789v_set_backlight_impl(st7789v_dev_t *dev, uint16_t val);
static int st7789v_clear_impl(st7789v_dev_t *dev, st7789v_color_t color);
static int st7789v_fill_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, st7789v_color_t color);
static int st7789v_flush_area_impl(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color);
static int st7789v_flush_area_dma_impl(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size);
static int st7789v_show_char_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint8_t chr, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
static int st7789v_show_str_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *str, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
static int st7789v_show_num_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
static int st7789v_show_hex_num_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
static int st7789v_show_float_num_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
static int st7789v_show_chinese_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *chinese, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay);
static int st7789v_show_image_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
static int st7789v_draw_point_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, st7789v_color_t color);
static int st7789v_draw_line_impl(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, st7789v_color_t color);
static int st7789v_draw_rectangle_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, st7789v_color_t color);
static int st7789v_draw_circle(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint8_t radius, st7789v_color_t color);
static int st7789v_deinit_impl(st7789v_dev_t *dev);

/* 操作接口表 */
static const st7789v_ops_t st7789v_ops = {
	.set_backlight  = st7789v_set_backlight_impl,
	.clear 		 	= st7789v_clear_impl,
	.fill 			= st7789v_fill_impl,
	.flush_area 	= st7789v_flush_area_impl,
	.flush_area_dma = st7789v_flush_area_dma_impl,
	.show_char 		= st7789v_show_char_impl,
	.show_str 		= st7789v_show_str_impl,
	.show_num 		= st7789v_show_num_impl,
	.show_hex_num 	= st7789v_show_hex_num_impl,
	.show_float_num = st7789v_show_float_num_impl,
	.show_chinese 	= st7789v_show_chinese_impl,
	.show_image 	= st7789v_show_image_impl,
	.draw_point 	= st7789v_draw_point_impl,
	.draw_line 		= st7789v_draw_line_impl,
	.draw_rectangle = st7789v_draw_rectangle_impl,
	.draw_circle 	= st7789v_draw_circle,
	.deinit 		= st7789v_deinit_impl,
};

/**
 * @brief   初始化 ST7789V 驱动
 * @param[out] dev 			 st7789v_dev_t 结构体指针
 * @param[in]  cfg 			 st7789v_cfg_t 结构体指针
 * @param[in]  disp_buf_addr 显示缓冲区地址，不使用 LVGL 等图形库时传入 NULL
 * @return	0 表示成功，其他值表示失败
 */
int drv_st7789v_init(st7789v_dev_t *dev, const st7789v_cfg_t *cfg, uint32_t disp_buf_addr)
{
	if (!dev || !cfg)
        return -EINVAL;

    dev->cfg = *cfg;
	dev->ops = &st7789v_ops;

	st7789v_hw_init(cfg);

	st7789v_cs_write(dev, 1);
	st7789v_dc_write(dev, 1);
	st7789v_res_write(dev, 1);

	/* 初始化背光 */
	cfg->pwm_ops->set_psc(TIMER_FREQ / 1000000 - 1);
	cfg->pwm_ops->set_arr(1000 - 1);
	st7789v_set_backlight_impl(dev, 100);
	
	/* 配置LCD */
	st7789v_reset(dev);			// 复位
	st7789v_set_direction(dev);	// 设置屏幕方向

	st7789v_write_cmd(dev, 0x36);
	if (cfg->is_vertical && cfg->is_forward)
		st7789v_write_byte(dev, 0x00);
    else if (cfg->is_vertical && !cfg->is_forward)
		st7789v_write_byte(dev, 0xC0);
	else if (!cfg->is_vertical && cfg->is_forward)
		st7789v_write_byte(dev, 0x70);
	else if (!cfg->is_vertical && !cfg->is_forward)
		st7789v_write_byte(dev, 0xA0);

    st7789v_write_cmd(dev, 0x3A);
    st7789v_write_byte(dev, 0x05);

    st7789v_write_cmd(dev, 0xB2);
    st7789v_write_byte(dev, 0x0B);
    st7789v_write_byte(dev, 0x0B);
    st7789v_write_byte(dev, 0x00);
    st7789v_write_byte(dev, 0x33);
    st7789v_write_byte(dev, 0x35);

    st7789v_write_cmd(dev, 0xB7);
    st7789v_write_byte(dev, 0x11);

    st7789v_write_cmd(dev, 0xBB);
    st7789v_write_byte(dev, 0x35);

    st7789v_write_cmd(dev, 0xC0);
    st7789v_write_byte(dev, 0x2C);

    st7789v_write_cmd(dev, 0xC2);
    st7789v_write_byte(dev, 0x01);

    st7789v_write_cmd(dev, 0xC3);
    st7789v_write_byte(dev, 0x0D);

    st7789v_write_cmd(dev, 0xC4);
    st7789v_write_byte(dev, 0x20);

    st7789v_write_cmd(dev, 0xC6);
    st7789v_write_byte(dev, 0x13);

    st7789v_write_cmd(dev, 0xD0);
    st7789v_write_byte(dev, 0xA4);
    st7789v_write_byte(dev, 0xA1);

    st7789v_write_cmd(dev, 0xD6);
    st7789v_write_byte(dev, 0xA1);

    st7789v_write_cmd(dev, 0xE0);
    st7789v_write_byte(dev, 0xF0);
    st7789v_write_byte(dev, 0x06);
    st7789v_write_byte(dev, 0x0B);
    st7789v_write_byte(dev, 0x0A);
    st7789v_write_byte(dev, 0x09);
    st7789v_write_byte(dev, 0x26);
    st7789v_write_byte(dev, 0x29);
    st7789v_write_byte(dev, 0x33);
    st7789v_write_byte(dev, 0x41);
    st7789v_write_byte(dev, 0x18);
    st7789v_write_byte(dev, 0x16);
    st7789v_write_byte(dev, 0x15);
    st7789v_write_byte(dev, 0x29);
    st7789v_write_byte(dev, 0x2D);

    st7789v_write_cmd(dev, 0xE1);
    st7789v_write_byte(dev, 0xF0);
    st7789v_write_byte(dev, 0x04);
    st7789v_write_byte(dev, 0x08);
    st7789v_write_byte(dev, 0x08);
    st7789v_write_byte(dev, 0x07);
    st7789v_write_byte(dev, 0x03);
    st7789v_write_byte(dev, 0x28);
    st7789v_write_byte(dev, 0x32);
    st7789v_write_byte(dev, 0x40);
    st7789v_write_byte(dev, 0x3B);
    st7789v_write_byte(dev, 0x19);
    st7789v_write_byte(dev, 0x18);
    st7789v_write_byte(dev, 0x2A);
    st7789v_write_byte(dev, 0x2E);

    st7789v_write_cmd(dev, 0xE4);
    st7789v_write_byte(dev, 0x25);
    st7789v_write_byte(dev, 0x00);
    st7789v_write_byte(dev, 0x00);

    st7789v_write_cmd(dev, 0x21);

    st7789v_write_cmd(dev, 0x11);
	cfg->delay_ms(120);
    st7789v_write_cmd(dev, 0x29);
	
	st7789v_clear_impl(dev, BLACK);

	/* DMA 设置 */
	if (disp_buf_addr == NULL)
		return 0;

	st7789v_hw_dma_init(cfg, disp_buf_addr);

	return 0;
}

/*通信协议*********************************************************************/

/**
 * @brief   ST7789V CS 线置高/低电平
 * @param[in] dev   st7789v_dev_t 结构体指针
 * @param[in] level 要写入的电平值，取 1/0
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_cs_write(st7789v_dev_t *dev, uint8_t level)
{
	if (!dev)
        return -EINVAL;

	st7789v_hw_gpio_write_bit(dev->cfg.cs_port, dev->cfg.cs_pin, level);
	return 0;
}

/**
 * @brief   ST7789V RES 线置高/低电平
 * @param[in] dev   st7789v_dev_t 结构体指针
 * @param[in] level 要写入的电平值，取 1/0
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_res_write(st7789v_dev_t *dev, uint8_t level)
{	
	if (!dev)
        return -EINVAL;

	st7789v_hw_gpio_write_bit(dev->cfg.res_port, dev->cfg.res_pin, level);
	return 0;
}

/**
 * @brief   ST7789V DC 线置高/低电平
 * @param[in] dev   st7789v_dev_t 结构体指针
 * @param[in] level 要写入的电平值，取 1/0
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_dc_write(st7789v_dev_t *dev, uint8_t level)
{	
	if (!dev)
        return -EINVAL;

	st7789v_hw_gpio_write_bit(dev->cfg.dc_port, dev->cfg.dc_pin, level);
	return 0;
}

/**
 * @brief   ST7789V 复位
 * @param[in] dev st7789v_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_reset(st7789v_dev_t *dev)
{	
	if (!dev)
        return -EINVAL;

	st7789v_res_write(dev, 1);
	dev->cfg.delay_ms(100);
	st7789v_res_write(dev, 0);
	dev->cfg.delay_ms(100);
	st7789v_res_write(dev, 1);
	dev->cfg.delay_ms(100);

	return 0;
}

/**
 * @brief   ST7789V 写命令
 * @param[in] dev st7789v_dev_t 结构体指针
 * @param[in] cmd 要写入的命令值，范围：0x00~0xFF
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_write_cmd(st7789v_dev_t *dev, uint8_t cmd)
{	
	if (!dev)
        return -EINVAL;

	st7789v_dc_write(dev, 0);									// 写命令
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉低CS，开始通信
	dev->cfg.spi_ops->swap_byte(cmd, NULL);
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉高CS，结束通信
	return 0;
}

/**
 * @brief   ST7789V 写一个字节数据
 * @param[in] dev  st7789v_dev_t 结构体指针
 * @param[in] byte 要写入的数据，范围：0x00~0xFF
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_write_byte(st7789v_dev_t *dev, uint8_t byte)
{	
	if (!dev)
        return -EINVAL;

	st7789v_dc_write(dev, 1);									// 写数据
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉低CS，开始通信
	dev->cfg.spi_ops->swap_byte(byte, NULL);					// 写入指定命令
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉高CS，结束通信
	return 0;
}

/**
 * @brief   ST7789V 写一个半字数据
 * @param[in] dev  	   st7789v_dev_t 结构体指针
 * @param[in] halfword 要写入的数据，范围：0x0000~0xFFFF
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_write_halfword(st7789v_dev_t *dev, uint16_t halfword)
{	
	if (!dev)
        return -EINVAL;

	st7789v_dc_write(dev, 1);									// 写数据
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉低CS，开始通信
	dev->cfg.spi_ops->swap_byte(halfword >> 8, NULL);			// 写入数据
	dev->cfg.spi_ops->swap_byte(halfword, NULL);				// 写入数据
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉高CS，结束通信
	return 0;
}

/**********************************************************************通信协议*/

/*硬件配置**********************************************************************/

/**
 * @brief   ST7789V 设置屏幕方向
 * @param[in] dev st7789v_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_set_direction(st7789v_dev_t *dev)
{	
	if (!dev)
        return -EINVAL;

    uint8_t memoryAccessReg = 0x00;

    if (!dev->cfg.is_vertical)
        memoryAccessReg = 0X70;
    else if (dev->cfg.is_vertical)
        memoryAccessReg = 0X00;

    st7789v_write_cmd(dev, 0x36);
    st7789v_write_byte(dev, memoryAccessReg);

	return 0;
}

/**
 * @brief   ST7789V 设置起始和结束地址
 * @param[in] dev st7789v_dev_t 结构体指针
 * @param[in] x1  设置列的起始地址
 * @param[in] y1  设置行的起始地址
 * @param[in] x2  设置列的结束地址
 * @param[in] y2  设置行的结束地址
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_set_windows(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{	
	if (!dev)
        return -EINVAL;

	if (dev->cfg.is_vertical) {
		st7789v_write_cmd(dev, 0x2a);			// 列地址设置
		st7789v_write_halfword(dev, x1);
		st7789v_write_halfword(dev, x2);

		st7789v_write_cmd(dev, 0x2b);			// 行地址设置
		st7789v_write_halfword(dev, y1 + 20);
		st7789v_write_halfword(dev, y2 + 20);

	} else if (!dev->cfg.is_vertical) {
		st7789v_write_cmd(dev, 0x2a);			// 列地址设置
		st7789v_write_byte(dev, (x1 + 20) >> 8);
        st7789v_write_byte(dev, x1 + 20);
        st7789v_write_byte(dev, (x2 + 20) >> 8);
        st7789v_write_byte(dev, x2 + 20);

		st7789v_write_cmd(dev, 0x2b);			// 行地址设置
        st7789v_write_byte(dev, y1 >> 8);
        st7789v_write_byte(dev, y1);
        st7789v_write_byte(dev, (y2) >> 8);
        st7789v_write_byte(dev, y2);
	}

	st7789v_write_cmd(dev, 0x2c);				// 储存器写 

	return 0;
}

/**********************************************************************硬件配置*/

/*工具函数**********************************************************************/

/**
 * @brief   次方函数
 * @param[in] x 底数
 * @param[in] y 指数
 * @return	x^y
 */
static uint32_t st7789v_pow(uint32_t x, uint32_t y)
{
	uint32_t result = 1;

	while (y--)			// 累乘y次
		result *= x;		// 每次把x累乘到结果上
	return result;
}

/**********************************************************************工具函数*/

/*功能函数**********************************************************************/

/**
 * @brief   ST7789V 设置背光
 * @param[in] dev st7789v_dev_t 结构体指针
 * @param[in] val 背光调节系数，范围：0~100
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_set_backlight_impl(st7789v_dev_t *dev, uint16_t val)
{
	if (!dev)
        return -EINVAL;

	uint16_t compare = val * 10;

	if (compare > 1000)
		compare = 1000;
	compare = 1000 - compare;
	dev->cfg.pwm_ops->set_compare(compare);

	return 0;
}

/**
 * @brief   ST7789V 填充颜色
 * @param[in] dev   st7789v_dev_t 结构体指针
 * @param[in] color 要填充的颜色
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_clear_impl(st7789v_dev_t *dev, st7789v_color_t color)
{
	if (!dev)
        return -EINVAL;
 
	uint16_t i, j;
    
    st7789v_set_windows(dev, 0, 0, dev->cfg.x_max, dev->cfg.y_max);

	st7789v_dc_write(dev, 1);
    for(i = 0; i < dev->cfg.x_max; i++)
        for(j = 0; j < dev->cfg.y_max; j++)
			st7789v_write_halfword(dev, color);

	return 0;
}

/**
 * @brief   ST7789V 在指定区域填充颜色
 * @param[in] dev 	 st7789v_dev_t 结构体指针
 * @param[in] x  	 填充起点横坐标
 * @param[in] y  	 填充起点纵坐标
 * @param[in] width  填充的宽度
 * @param[in] height 填充的高度
 * @param[in] color  要填充的颜色
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_fill_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, st7789v_color_t color)
{
	if (!dev)
        return -EINVAL;

	uint16_t i, j;

	st7789v_set_windows(dev, x, y, x + width - 1, y + height - 1);	// 设置显示范围
	for (i = y; i < y + height; i++)													   	 	
		for (j = x; j < x + width; j++)
			st7789v_write_halfword(dev, color);
	
	return 0;
}

/**
 * @brief   ST7789V 向显示区域刷新像素数据，用于 LVGL 等图形库
 * @param[in] dev 	st7789v_dev_t 结构体指针
 * @param[in] x1  	填充起点横坐标
 * @param[in] y1  	填充起点纵坐标
 * @param[in] x2  	填充终点横坐标
 * @param[in] y2 	填充终点纵坐标
 * @param[in] color 要填充的颜色
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_flush_area_impl(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color)
{	
	if (!dev)
        return -EINVAL;

    uint16_t height, width;
    uint16_t i, j;
    width = x2 - x1 + 1;            // 填充的宽度
    height = y2 - y1 + 1;           // 填充的高度

	st7789v_set_windows(dev, x1, y1, x2, y2);					// 设置光标位置
	
    for (i = 0; i < height; i++)
        for (j = 0; j < width; j++)
            st7789v_write_halfword(dev, color[i * width + j]);	// 写入数据
	
	return 0;
}

/**
 * @brief   ST7789V 向显示区域刷新像素数据，并使用 DMA 传输，用于 LVGL 等图形库
 * @param[in] dev 	st7789v_dev_t 结构体指针
 * @param[in] x1  	填充起点横坐标
 * @param[in] y1  	填充起点纵坐标
 * @param[in] x2  	填充终点横坐标
 * @param[in] y2 	填充终点纵坐标
 * @param[in] size  传输大小
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_flush_area_dma_impl(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size)
{	
	if (!dev)
        return -EINVAL;

	st7789v_set_windows(dev, x1, y1, x2, y2);

	st7789v_dc_write(dev, 1);									// 写数据
	dev->cfg.spi_ops->start(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉低CS，开始通信
	st7789v_hw_spi_dma_transfer(dev->cfg.spi_periph, size);
	dev->cfg.spi_ops->stop(dev->cfg.cs_port, dev->cfg.cs_pin);	// 拉高CS，结束通信
	return 0;
}

/**
 * @brief   ST7789V 显示单个字符
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] chr  	  要显示的字符
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] size    指定字体大小
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_char_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint8_t chr, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay)
{	
	if (!dev)
        return -EINVAL;

	uint8_t temp, sizex, t, m = 0;
	uint16_t i, typeface_num;							// 一个字符所占字节大小
	uint16_t x0 = x;
	sizex = size / 2;
	typeface_num = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * size;
	chr = chr - ' ';    								// 得到偏移后的值

	for (i = 0; i < typeface_num; i++) { 
		if (size==12)temp=lcd_f6x12[chr][i];			// 调用6x12字体
		else if (size==16)temp=lcd_f8x6[chr][i];		// 调用8x16字体
		else if (size==24)temp=lcd_f12x24[chr][i];		// 调用12x24字体
		else if (size==32)temp=lcd_f16x32[chr][i];		// 调用16x32字体
		else return -EINVAL;

		for (t = 0;t < 8;t++) {
			if (!overlay) {								// 非叠加模式
				if (temp & (0x01 << t)) st7789v_draw_point_impl(dev, x, y, fc);
                else st7789v_draw_point_impl(dev, x, y, bc);
                m++;
                x++;
                if (m % sizex == 0)
                {
                    m = 0;
                    x = x0;
                    y++;
                    break;
                }
			} else {									// 叠加模式
				if (temp & (0x01 << t)) st7789v_draw_point_impl(dev, x, y, fc);
                x++;
                if ((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
			}
		}
	}
	return 0;
}

/**
 * @brief   ST7789V 显示字符串
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] str  	  要显示的字符串
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] size    指定字体大小
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_str_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *str, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay)
{
	if (!dev)
        return -EINVAL;
   
	while (*str != '\0') {       
		st7789v_show_char_impl(dev, x, y, *str, fc, bc, size, overlay);
		x += size / 2;
		str++;
	}
	return 0;
}

/**
 * @brief   ST7789V 显示整数
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] num  	  要显示的整数
 * @param[in] len  	  指定数字的位数
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] size    指定字体大小
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_num_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay)
{	
	if (!dev)
		return -EINVAL;
	
	uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++) {
        temp = (num / st7789v_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                st7789v_show_char_impl(dev, x + (size / 2)*t, y, ' ', fc, bc, size, overlay);
                continue;
            }
            else enshow = 1;
        }
        st7789v_show_char_impl(dev, x + (size / 2)*t, y, temp + '0', fc, bc, size, overlay);
    }
	return 0;
}

/**
 * @brief   ST7789V 显示十六进制数
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] num  	  要显示的数字，范围：0x00000000~0xFFFFFFFF
 * @param[in] len  	  指定数字的长度，范围：0~8
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] size    指定字体大小
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_hex_num_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay)
{	
	if (!dev)
        return -EINVAL;

	uint8_t i, single_num;

	/* 遍历数字的每一位 */
	for (i = 0; i < len; i++) {
		/* 以十六进制提取数字的每一位 */
		single_num = num / st7789v_pow(16, len - i - 1) % 16;
		
		if (single_num < 10) {			// 单个数字小于10
			/* 调用__oled_show_char函数，显示此数字 */
			/* + '0' 可将数字转换为字符格式 */
			st7789v_show_char_impl(dev, x + i * size / 2, y, single_num + '0', fc, bc, size, overlay);
		} else {						// 单个数字大于10
			/* 调用__oled_show_char函数，显示此数字 */
			/* + 'A' 可将数字转换为从A开始的十六进制字符 */
			st7789v_show_char_impl(dev, x + i * size / 2, y, single_num - 10 + 'A', fc, bc, size, overlay);
		}
	}
	return 0;
}

/**
 * @brief   ST7789V 显示两位小数变量
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] num  	  要显示的小数
 * @param[in] int_len 要显示的整数位数
 * @param[in] fra_len 要显示的小数位数
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] size    指定字体大小
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_float_num_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay)
{	
	if (!dev)
        return -EINVAL;

    uint32_t pow_num, int_num, fra_num;
	
	if (num < 0)
		st7789v_show_char_impl(dev, x, y, '-', fc, bc, size, overlay);	// 显示-号
	
	/* 提取整数部分和小数部分 */
	int_num = num;						// 直接赋值给整型变量，提取整数
	num -= int_num;						// 将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	pow_num = st7789v_pow(10, fra_len);	// 根据指定小数的位数，确定乘数
	fra_num = round(num * pow_num);		// 将小数乘到整数，同时四舍五入，避免显示误差
	int_num += fra_num / pow_num;		// 若四舍五入造成了进位，则需要再加给整数
	
	if (num >= 0) {
		st7789v_show_num_impl(dev, x, y, int_num, int_len, fc, bc, size, overlay);								// 显示整数部分
		st7789v_show_char_impl(dev, x + (int_len) * size / 2, y, '.', fc, bc, size, overlay);					// 显示小数点
		st7789v_show_num_impl(dev, x + (int_len + 1) * size / 2, y, fra_num, fra_len, fc, bc, size, overlay);	// 显示小数部分
	} else {
		num = -num;
		st7789v_show_num_impl(dev, x + size / 2, y, int_num, int_len, fc, bc, size, overlay);					// 显示整数部分
		st7789v_show_char_impl(dev, x + (int_len + 1) * size / 2, y, '.', fc, bc, size, overlay);				// 显示小数点
		st7789v_show_num_impl(dev, x + (int_len + 2) * size / 2, y, fra_num, fra_len, fc, bc, size, overlay);	// 显示小数部分
	}
	return 0;
}

/**
 * @brief   ST7789V 显示单个 12x12 汉字
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] chinese 要显示的汉字
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_chinese_impl12x12(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *chinese, st7789v_color_t fc, st7789v_color_t bc, bool overlay)
{	
	if (!dev)
        return -EINVAL;

	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(12); 					// 汉字数目
	uint16_t typeface_num = (12 / 8 + ((12 % 8) ? 1 : 0)) * 12;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++) {
		if ((lcd_f12x12[k].index[0] == *(chinese)) && (lcd_f12x12[k].index[1] == *(chinese + 1))) {
			st7789v_set_windows(dev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < typeface_num; i++) {
				for (j = 0; j < 8; j++) {
					if (lcd_f12x12[k].msk[i] & (0x01 << j))
					st7789v_draw_point_impl(dev, x, y, fc); // 画一个点
					else if (!overlay)
						st7789v_draw_point_impl(dev, x, y, bc);
					x++;
					if ((x - x0) == 12) {
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
	return 0;
}

/**
 * @brief   ST7789V 显示单个 16x16 汉字
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] chinese 要显示的汉字
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_chinese_impl16x16(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *chinese, st7789v_color_t fc, st7789v_color_t bc, bool overlay)
{	
	if (!dev)
        return -EINVAL;

	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(16); 					// 汉字数目
	uint16_t typeface_num = (16 / 8 + ((16 % 8) ? 1 : 0)) * 16;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++) {
		if ((lcd_f16x16[k].index[0] == *(chinese)) && (lcd_f16x16[k].index[1] == *(chinese + 1))) {
			st7789v_set_windows(dev, x, y, x + 16 - 1, y + 16 - 1);
			for (i = 0; i < typeface_num; i++) {
				for (j = 0; j < 8; j++) {
					if (lcd_f16x16[k].msk[i] & (0x01 << j))
						st7789v_draw_point_impl(dev, x, y, fc); // 画一个点
					else if (!overlay)
						st7789v_draw_point_impl(dev, x, y, bc);
					x++;
					if ((x - x0) == 16) {
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
	return 0;
}

/**
 * @brief   ST7789V 显示单个 24x24 汉字
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] chinese 要显示的汉字
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_chinese_impl24x24(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *chinese, st7789v_color_t fc, st7789v_color_t bc, bool overlay)
{	
	if (!dev)
        return -EINVAL;

	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(24); 					// 汉字数目
	uint16_t typeface_num = (24 / 8 + ((24 % 8) ? 1 : 0)) * 24;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++) {
		if ((lcd_f24x24[k].index[0] == *(chinese)) && (lcd_f24x24[k].index[1] == *(chinese + 1))) {
			st7789v_set_windows(dev, x, y, x + 24 - 1, y + 24 - 1);
			for (i = 0; i < typeface_num; i++) {
				for (j = 0; j < 8; j++) {
					if (lcd_f24x24[k].msk[i] & (0x01 << j))
						st7789v_draw_point_impl(dev, x, y, fc); // 画一个点
					else if (!overlay)
						st7789v_draw_point_impl(dev, x, y, bc);
					x++;
					if ((x - x0) == 24) {
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
	return 0;
}

/**
 * @brief   ST7789V 显示单个 32x32 汉字
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] chinese 要显示的汉字
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_chinese_impl32x32(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *chinese, st7789v_color_t fc, st7789v_color_t bc, bool overlay)
{	
	if (!dev)
        return -EINVAL;

	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(32); 					// 汉字数目
	uint16_t typeface_num = (32 / 8 + ((32 % 8) ? 1 : 0)) * 32;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++) {
		if ((lcd_f32x32[k].index[0] == *(chinese)) && (lcd_f32x32[k].index[1] == *(chinese + 1))) {
			st7789v_set_windows(dev, x, y, x + 32 - 1, y + 32 - 1);
			for (i = 0; i < typeface_num; i++) {
				for (j = 0; j < 8; j++) {
					if (lcd_f32x32[k].msk[i] & (0x01 << j))
						st7789v_draw_point_impl(dev, x, y, fc); //画一个点
					else if (!overlay)
						st7789v_draw_point_impl(dev, x, y, bc);
					x++;
					if ((x - x0) == 32) {
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
	return 0;
}

/**
 * @brief   ST7789V 显示汉字串
 * @param[in] dev 	  st7789v_dev_t 结构体指针
 * @param[in] x  	  起点横坐标
 * @param[in] y  	  起点纵坐标
 * @param[in] chinese 要显示的汉字串
 * @param[in] fc 	  字的颜色
 * @param[in] bc  	  字的背景色
 * @param[in] overlay 是否叠加
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_chinese_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, char *chinese, st7789v_color_t fc, st7789v_color_t bc, st7789v_font_size_t size, bool overlay)
{	
	if (!dev)
        return -EINVAL;

	while (*chinese != 0) {
		if (size == LCD_12X12)		st7789v_show_chinese_impl12x12(dev, x, y, chinese, fc, bc, overlay);
		else if (size == LCD_16X16)	st7789v_show_chinese_impl16x16(dev, x, y, chinese, fc, bc, overlay);
		else if (size == LCD_24X24)	st7789v_show_chinese_impl24x24(dev, x, y, chinese, fc, bc, overlay);
		else if (size == LCD_32X32)	st7789v_show_chinese_impl32x32(dev, x, y, chinese, fc, bc, overlay);
		else return -EINVAL;
		
		chinese += LCD_CHN_CHAR_WIDTH;
		x += size;
	}
	return 0;
}

/**
 * @brief   ST7789V 显示图片
 * @param[in] dev 	 st7789v_dev_t 结构体指针
 * @param[in] x  	 图片左上角横坐标
 * @param[in] y  	 图片左上角纵坐标
 * @param[in] width  图片宽度
 * @param[in] height 图片高度
 * @param[in] pic  	 图片数组
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_show_image_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{	
	if (!dev)
        return -EINVAL;

	uint16_t i, j; 
	uint32_t k = 0;

	st7789v_set_windows(dev, x, y, x + width - 1, y + height - 1);	// 设置显示范围
	for (i = 0; i < height; i++) {													   	 	
		for (j = 0; j < width; j++) {
			st7789v_write_byte(dev, pic[k * 2]);
			st7789v_write_byte(dev, pic[k * 2 + 1]);
			k++;
		}
	}
	return 0;
}

/**
 * @brief   ST7789V 画点
 * @param[in] dev 	st7789v_dev_t 结构体指针
 * @param[in] x  	横坐标
 * @param[in] y  	纵坐标
 * @param[in] color 颜色
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_draw_point_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, st7789v_color_t color)
{	
	if (!dev)
        return -EINVAL;

	st7789v_set_windows(dev, x, y, x, y);		// 设置光标位置 
	st7789v_write_halfword(dev, (uint16_t)color);
	return 0;
}

/**
 * @brief   ST7789V 画线
 * @param[in] dev 	st7789v_dev_t 结构体指针
 * @param[in] x1  	起点横坐标
 * @param[in] y1  	起点纵坐标
 * @param[in] x2  	终点横坐标
 * @param[in] y3  	终点纵坐标
 * @param[in] color 颜色
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_draw_line_impl(st7789v_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, st7789v_color_t color)
{	
	if (!dev)
        return -EINVAL;

    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1;              // 计算坐标增量
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

	/* 设置单步方向 */
    if (delta_x > 0) {
		incx = 1;       
	} else if (delta_x == 0) {
		incx = 0; // 垂直线
	} else{
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0) {
		incy = 1;
	} else if (delta_y == 0) {
		incy = 0; // 水平线
    } else {
        incy = -1;
        delta_y = -delta_y;
    }

    if ( delta_x > delta_y)distance = delta_x; // 选取基本增量坐标轴
    else distance = delta_y;

	/* 画线输出 */
    for (t = 0; t <= distance + 1; t++ ) {
        st7789v_draw_point_impl(dev, uRow, uCol, color); // 画点
        xerr += delta_x ;
        yerr += delta_y ;

        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }

        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
	return 0;
}

/**
 * @brief   ST7789V 画矩形
 * @param[in] dev 	 st7789v_dev_t 结构体指针
 * @param[in] x  	 矩形左上角横坐标
 * @param[in] y  	 矩形左上角纵坐标
 * @param[in] width  矩形宽度
 * @param[in] height 矩形高度
 * @param[in] color  颜色
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_draw_rectangle_impl(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, st7789v_color_t color)
{	
	if (!dev)
        return -EINVAL;

	uint16_t x_end = x + width - 1;
    uint16_t y_end = y + height - 1;

	st7789v_draw_line_impl(dev, x, y, x_end, y, color);
	st7789v_draw_line_impl(dev, x, y, x, y_end, color);
	st7789v_draw_line_impl(dev, x, y_end, x_end, y_end, color);
	st7789v_draw_line_impl(dev, x_end, y, x_end, y_end, color);
	return 0;
}

/**
 * @brief   ST7789V 画圆
 * @param[in] dev 	 st7789v_dev_t 结构体指针
 * @param[in] x  	 圆心横坐标
 * @param[in] y  	 圆心纵坐标
 * @param[in] radius 半径
 * @param[in] color  颜色
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_draw_circle(st7789v_dev_t *dev, uint16_t x, uint16_t y, uint8_t radius, st7789v_color_t color)
{	
	if (!dev)
        return -EINVAL;

	int a, b;
	a = 0;
	b = radius;
	
	while (a <= b) {
		st7789v_draw_point_impl(dev, x - b, y - a, color);	// 3           
		st7789v_draw_point_impl(dev, x + b, y - a, color);	// 0           
		st7789v_draw_point_impl(dev, x - a, y + b, color);	// 1                
		st7789v_draw_point_impl(dev, x - a, y - b, color);	// 2             
		st7789v_draw_point_impl(dev, x + b, y + a, color);	// 4               
		st7789v_draw_point_impl(dev, x + a, y - b, color);	// 5
		st7789v_draw_point_impl(dev, x + a, y + b, color);	// 6 
		st7789v_draw_point_impl(dev, x - b, y + a, color);	// 7
		a++;
		if ((a * a + b * b) > (radius * radius))	// 判断要画的点是否过远
			b--;
	}
	return 0;
}

/**
 * @brief   去初始化 ST7789V
 * @param[in] dev st7789v_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int st7789v_deinit_impl(st7789v_dev_t *dev)
{
	if (!dev)
		return -EINVAL;

	dev->ops = NULL;
	return 0;
}

/**********************************************************************功能函数*/

/* ------------------------------- 核心驱动层结束 ------------------------------- */
