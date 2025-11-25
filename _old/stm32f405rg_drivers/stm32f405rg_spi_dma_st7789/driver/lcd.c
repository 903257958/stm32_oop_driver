#include "lcd.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)

#define TIMER_FREQ	72000000
	
#define	__lcd_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
									}
													
#define	__lcd_dma_clock_enable(spix)	{	if (spix == SPI1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
											else if (spix == SPI2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
											else if (spix == SPI3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);} \
										}
													
#define	__lcd_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __lcd_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#define	__lcd_get_dma_channel(spix)		(	spix == SPI1 ? DMA1_Channel3 : \
											spix == SPI2 ? DMA1_Channel5 : \
											spix == SPI3 ? DMA2_Channel2 : \
											(int)0)
												
#define	__lcd_get_dma_flag(spix)		(	spix == SPI1 ? DMA1_FLAG_TC3 : \
											spix == SPI2 ? DMA1_FLAG_TC5 : \
											spix == SPI3 ? DMA2_FLAG_TC2 : \
											(int)0)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

	#if defined(STM32F40_41xxx)
	#define TIMER_FREQ	84000000

	#elif defined(STM32F411xE)
	#define TIMER_FREQ	100000000

	#elif defined(STM32F429_439xx)
	#define TIMER_FREQ	90000000

	#endif

#define	__lcd_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
										else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
										else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
										else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
										else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
										else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
										else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
									}

#define	__lcd_dma_clock_enable(spix)	{	if (spix == SPI1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
											else if (spix == SPI2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
											else if (spix == SPI3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
										}
													
#define	__lcd_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __lcd_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)
											
#define __lcd_get_dma_stream(spix)	(	spix == SPI1 ? DMA2_Stream3 : \
										spix == SPI2 ? DMA1_Stream4 : \
										spix == SPI3 ? DMA1_Stream5 : \
										(int)0)

#define	__lcd_get_dma_channel(spix)		(	spix == SPI1 ? DMA_Channel_3 : \
											spix == SPI2 ? DMA_Channel_0 : \
											spix == SPI3 ? DMA_Channel_0 : \
											(int)0)
												
#define	__lcd_get_dma_flag(spix)		(	spix == SPI1 ? DMA_FLAG_TCIF3 : \
											spix == SPI2 ? DMA_FLAG_TCIF4 : \
											spix == SPI3 ? DMA_FLAG_TCIF5 : \
											(int)0)

#endif

#endif
											
/* LCD私有数据结构体 */
typedef struct {
	spi_hard_dev_t spi;	// 硬件SPI设备
	pwm_dev_t pwm;		// PWM背光引脚
} lcd_priv_data_t;

static void __lcd_backlight_ctrl(struct lcd_dev *dev, uint16_t val);

/* 通信协议 */
static void __lcd_res_write(lcd_dev_t *dev, uint8_t bit_val);
static void __lcd_dc_write(lcd_dev_t *dev, uint8_t bit_val);
static void __lcd_reset(lcd_dev_t *dev);
static void __lcd_write_command(lcd_dev_t *dev, uint8_t command);
static void __lcd_write_byte(lcd_dev_t *dev, uint8_t data);
static void __lcd_write_halfword(lcd_dev_t *dev, uint16_t halfword);

/* 硬件配置 */
static void __lcd_set_direction(lcd_dev_t *dev);
static void __lcd_set_windows(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/* 功能函数 */
static void __lcd_clear(lcd_dev_t *dev, uint16_t color);
static void __lcd_fill(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
static void __lcd_color_fill(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color);
static void __lcd_color_fill_dma(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size);
static void __lcd_show_char(lcd_dev_t *dev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_string(lcd_dev_t *dev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_num(lcd_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_hex_num(lcd_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_float_num(lcd_dev_t *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_chinese(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_image(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
static void __lcd_draw_point(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t color);
static void __lcd_draw_line(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
static void __lcd_draw_rectangle(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
static void __lcd_draw_circle(lcd_dev_t *dev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color);
static int8_t __lcd_deinit(lcd_dev_t *dev);

/******************************************************************************
 * @brief	初始化LCD
 * @param	dev	:	lcd_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t lcd_init(lcd_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 函数指针赋值 */
	dev->backlight_ctrl = __lcd_backlight_ctrl;
	dev->clear = __lcd_clear;
	dev->fill = __lcd_fill;
	dev->color_fill = __lcd_color_fill;
	dev->color_fill_dma = __lcd_color_fill_dma;
	dev->show_char = __lcd_show_char;
	dev->show_string = __lcd_show_string;
	dev->show_num = __lcd_show_num;
	dev->show_hex_num = __lcd_show_hex_num;
	dev->show_float_num = __lcd_show_float_num;
	dev->show_chinese = __lcd_show_chinese;
	dev->show_image = __lcd_show_image;
	dev->draw_point = __lcd_draw_point;
	dev->draw_line = __lcd_draw_line;
	dev->draw_rectangle = __lcd_draw_rectangle;
	dev->draw_circle = __lcd_draw_circle;
	dev->deinit = __lcd_deinit;
	
	dev->init_flag = true;

	/* 保存私有数据 */
	dev->priv_data = (lcd_priv_data_t *)malloc(sizeof(lcd_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
	
	priv_data->spi.config.spix = dev->config.spix;
	priv_data->spi.config.sck_port = dev->config.sck_port;
	priv_data->spi.config.sck_pin = dev->config.sck_pin;
	priv_data->spi.config.mosi_port = dev->config.mosi_port;
	priv_data->spi.config.mosi_pin = dev->config.mosi_pin;
	priv_data->spi.config.miso_port = NULL;
	priv_data->spi.config.miso_pin = NULL;
	priv_data->spi.config.cs_port = dev->config.cs_port;
	priv_data->spi.config.cs_pin = dev->config.cs_pin;
	priv_data->spi.config.prescaler = 2;
	priv_data->spi.config.mode = SPI_MODE_3;

	priv_data->pwm.config.timx = dev->config.timx;
	priv_data->pwm.config.oc_channel = dev->config.oc_channel;
	priv_data->pwm.config.psc = TIMER_FREQ / 1000000 - 1;
	priv_data->pwm.config.arr = 999;
	priv_data->pwm.config.port = dev->config.bl_port;
	priv_data->pwm.config.pin = dev->config.bl_pin;
	
	/* 配置硬件SPI */
	spi_hard_init(&priv_data->spi);
	
	/* 配置时钟与GPIO */
	__lcd_io_clock_enable(dev->config.res_port);
	__lcd_io_clock_enable(dev->config.dc_port);
	
	__lcd_config_io_out_pp(dev->config.res_port, dev->config.res_pin);
	__lcd_config_io_out_pp(dev->config.dc_port, dev->config.dc_pin);
	
	__lcd_dc_write(dev, 1);
	__lcd_res_write(dev, 1);
	
	/* 配置LCD */
	__lcd_reset(dev);			// 复位

	__lcd_set_direction(dev);	// 设置屏幕方向

	__lcd_write_command(dev, 0x36);
	if (dev->config.dir == VERTICAL_FORWARD)
		__lcd_write_byte(dev, 0x00);
    else if (dev->config.dir == VERTICAL_REVERSE)
		__lcd_write_byte(dev, 0xC0);
	else if (dev->config.dir == HORIZONTAL_FORWARD)
		__lcd_write_byte(dev, 0x70);
	else if (dev->config.dir == HORIZONTAL_REVERSE)
		__lcd_write_byte(dev, 0xA0);

    __lcd_write_command(dev, 0x3A);
    __lcd_write_byte(dev, 0x05);

    __lcd_write_command(dev, 0xB2);
    __lcd_write_byte(dev, 0x0B);
    __lcd_write_byte(dev, 0x0B);
    __lcd_write_byte(dev, 0x00);
    __lcd_write_byte(dev, 0x33);
    __lcd_write_byte(dev, 0x35);

    __lcd_write_command(dev, 0xB7);
    __lcd_write_byte(dev, 0x11);

    __lcd_write_command(dev, 0xBB);
    __lcd_write_byte(dev, 0x35);

    __lcd_write_command(dev, 0xC0);
    __lcd_write_byte(dev, 0x2C);

    __lcd_write_command(dev, 0xC2);
    __lcd_write_byte(dev, 0x01);

    __lcd_write_command(dev, 0xC3);
    __lcd_write_byte(dev, 0x0D);

    __lcd_write_command(dev, 0xC4);
    __lcd_write_byte(dev, 0x20);

    __lcd_write_command(dev, 0xC6);
    __lcd_write_byte(dev, 0x13);

    __lcd_write_command(dev, 0xD0);
    __lcd_write_byte(dev, 0xA4);
    __lcd_write_byte(dev, 0xA1);

    __lcd_write_command(dev, 0xD6);
    __lcd_write_byte(dev, 0xA1);

    __lcd_write_command(dev, 0xE0);
    __lcd_write_byte(dev, 0xF0);
    __lcd_write_byte(dev, 0x06);
    __lcd_write_byte(dev, 0x0B);
    __lcd_write_byte(dev, 0x0A);
    __lcd_write_byte(dev, 0x09);
    __lcd_write_byte(dev, 0x26);
    __lcd_write_byte(dev, 0x29);
    __lcd_write_byte(dev, 0x33);
    __lcd_write_byte(dev, 0x41);
    __lcd_write_byte(dev, 0x18);
    __lcd_write_byte(dev, 0x16);
    __lcd_write_byte(dev, 0x15);
    __lcd_write_byte(dev, 0x29);
    __lcd_write_byte(dev, 0x2D);

    __lcd_write_command(dev, 0xE1);
    __lcd_write_byte(dev, 0xF0);
    __lcd_write_byte(dev, 0x04);
    __lcd_write_byte(dev, 0x08);
    __lcd_write_byte(dev, 0x08);
    __lcd_write_byte(dev, 0x07);
    __lcd_write_byte(dev, 0x03);
    __lcd_write_byte(dev, 0x28);
    __lcd_write_byte(dev, 0x32);
    __lcd_write_byte(dev, 0x40);
    __lcd_write_byte(dev, 0x3B);
    __lcd_write_byte(dev, 0x19);
    __lcd_write_byte(dev, 0x18);
    __lcd_write_byte(dev, 0x2A);
    __lcd_write_byte(dev, 0x2E);

    __lcd_write_command(dev, 0xE4);
    __lcd_write_byte(dev, 0x25);
    __lcd_write_byte(dev, 0x00);
    __lcd_write_byte(dev, 0x00);

    __lcd_write_command(dev, 0x21);

    __lcd_write_command(dev, 0x11);
    LCD_DELAY_MS(120);
    __lcd_write_command(dev, 0x29);
	
	/* 屏幕初始化为黑色 */
	__lcd_clear(dev, BLACK);
    
    /* 配置背光 */
	pwm_init(&priv_data->pwm);
    priv_data->pwm.set_compare(&priv_data->pwm, 100);
	
	return 0;
}

/******************************************************************************
 * @brief	LCD配置DMA传输，用于LVGL
 * @param	dev			:	lcd_dev_t 结构体指针
 * @param	mem_base_addr	:	DMA内存地址
 * @return	无
 ******************************************************************************/
void lcd_dma_init(lcd_dev_t *dev, uint32_t mem_base_addr)
{
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	__lcd_dma_clock_enable(dev->config.spix);	// 开启DMA时钟
	DMA_DeInit(__lcd_get_dma_channel(dev->config.spix));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.spix->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)mem_base_addr;				// 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;							// 方向：从内存到外设
    DMA_InitStructure.DMA_BufferSize = 0;										// 传输大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 外设地址不增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// 内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		// 外设数据单位8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				// 内存数据单位8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								// 工作在正常模式，一次传输后自动结束
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;						// 优先级：中
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								// 没有设置为内存到内存传输
    DMA_Init(__lcd_get_dma_channel(dev->config.spix), &DMA_InitStructure);
	
    DMA_ClearFlag(__lcd_get_dma_flag(dev->config.spix));
	
    DMA_Cmd(__lcd_get_dma_channel(dev->config.spix), DISABLE);
	
	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	
    __lcd_dma_clock_enable(dev->config.spix);	// 开启DMA时钟
	
	DMA_DeInit(__lcd_get_dma_stream(dev->config.spix));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = __lcd_get_dma_channel(dev->config.spix);	// 选择DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.spix->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)mem_base_addr;			// 内存地址
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
    DMA_Init(__lcd_get_dma_stream(dev->config.spix), &DMA_InitStructure);
	
    DMA_ClearFlag(	__lcd_get_dma_stream(dev->config.spix), 
					__lcd_get_dma_flag(dev->config.spix)	);
					
    DMA_Cmd(__lcd_get_dma_stream(dev->config.spix), DISABLE);
	
	#endif
}

/******************************************************************************
 * @brief	LCD背光控制
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	value	:	背光调节系数，范围：0~100
 * @return	无
 ******************************************************************************/
static void __lcd_backlight_ctrl(struct lcd_dev *dev, uint16_t val)
{
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;

	uint16_t compare = val * 10;
	if (compare > 1000)
	{
		compare = 1000;
	}
	compare = 1000 - compare;

	priv_data->pwm.set_compare(&priv_data->pwm, compare);
}

/*通信协议*********************************************************************/

/******************************************************************************
 * @brief	LCD写RES高低电平
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	bit_val	:	要写入RES的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_res_write(lcd_dev_t *dev, uint8_t bit_val)
{
	__lcd_io_write(dev->config.res_port, dev->config.res_pin, bit_val);
}

/******************************************************************************
 * @brief	LCD写DC高低电平
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	bit_val	:	要写入DC的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_dc_write(lcd_dev_t *dev, uint8_t bit_val)
{
	__lcd_io_write(dev->config.dc_port, dev->config.dc_pin, bit_val);
}

/******************************************************************************
 * @brief	LCD复位
 * @param	dev	:	lcd_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_reset(lcd_dev_t *dev)
{
	__lcd_res_write(dev, 1);
	LCD_DELAY_MS(100);
	__lcd_res_write(dev, 0);
	LCD_DELAY_MS(100);
	__lcd_res_write(dev, 1);
	LCD_DELAY_MS(100);
}

/******************************************************************************
 * @brief	LCD写命令
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	byte	:	要写入的命令值，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_command(lcd_dev_t *dev, uint8_t command)
{
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
	
	__lcd_dc_write(dev, 0);								// 写命令
	priv_data->spi.start(&priv_data->spi);				// 拉低CS，开始通信
	priv_data->spi.swap_byte(&priv_data->spi, command);	// 写入指定命令
	priv_data->spi.stop(&priv_data->spi);				// 拉高CS，结束通信
}

/******************************************************************************
 * @brief	LCD写一个字节数据
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	byte	:	要写入的数据，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_byte(lcd_dev_t *dev, uint8_t byte)
{	
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
	
	__lcd_dc_write(dev, 1);								// 写数据
	priv_data->spi.start(&priv_data->spi);				// 拉低CS，开始通信
	priv_data->spi.swap_byte(&priv_data->spi, byte);	// 写入指定命令
	priv_data->spi.stop(&priv_data->spi);				// 拉高CS，结束通信
}

/******************************************************************************
 * @brief	LCD写一个半字数据
 * @param	dev			:	lcd_dev_t 结构体指针
 * @param	halfword	:	要写入的数据，范围：0x0000~0xFFFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_halfword(lcd_dev_t *dev, uint16_t halfword)
{
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
	
	__lcd_dc_write(dev, 1);										// 写数据
	priv_data->spi.start(&priv_data->spi);						// 拉低CS，开始通信
	priv_data->spi.swap_byte(&priv_data->spi, halfword >> 8);	// 写入数据
	priv_data->spi.swap_byte(&priv_data->spi, halfword);		// 写入数据
	priv_data->spi.stop(&priv_data->spi);						// 拉高CS，结束通信
}

/**********************************************************************通信协议*/

/*硬件配置**********************************************************************/

/******************************************************************************
 * @brief	LCD设置屏幕方向
 * @param	dev	:	lcd_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_set_direction(lcd_dev_t *dev)
{
    uint8_t memoryAccessReg = 0x00;

    if (dev->config.dir == HORIZONTAL_FORWARD || dev->config.dir == HORIZONTAL_REVERSE)
	{
		dev->width = LCD_H;
		dev->height = LCD_W;
        memoryAccessReg = 0X70;
    }
    else if (dev->config.dir == VERTICAL_FORWARD || dev->config.dir == VERTICAL_REVERSE)
	{    
		dev->width = LCD_W;
		dev->height = LCD_H;
        memoryAccessReg = 0X00;
    }

    __lcd_write_command(dev, 0x36);
    __lcd_write_byte(dev, memoryAccessReg);
}

/******************************************************************************
 * @brief	LCD设置起始和结束地址
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x1,x2	:	设置列的起始和结束地址
 * @param	y1,y2	:	设置行的起始和结束地址
 * @return	无
 ******************************************************************************/
static void __lcd_set_windows(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if (dev->config.dir == VERTICAL_FORWARD || dev->config.dir == VERTICAL_REVERSE)	// 竖屏
	{
		__lcd_write_command(dev, 0x2a);			// 列地址设置
		__lcd_write_halfword(dev, x1);
		__lcd_write_halfword(dev, x2);

		__lcd_write_command(dev, 0x2b);			// 行地址设置
		__lcd_write_halfword(dev, y1 + 20);
		__lcd_write_halfword(dev, y2 + 20);

	}
	else if (dev->config.dir == HORIZONTAL_FORWARD || dev->config.dir == HORIZONTAL_REVERSE)	// 横屏
	{
		__lcd_write_command(dev, 0x2a);			// 列地址设置
		__lcd_write_byte(dev, (x1 + 20) >> 8);
        __lcd_write_byte(dev, x1 + 20);
        __lcd_write_byte(dev, (x2 + 20) >> 8);
        __lcd_write_byte(dev, x2 + 20);

		__lcd_write_command(dev, 0x2b);			// 行地址设置
        __lcd_write_byte(dev, y1 >> 8);
        __lcd_write_byte(dev, y1);
        __lcd_write_byte(dev, (y2) >> 8);
        __lcd_write_byte(dev, y2);
	}

	__lcd_write_command(dev, 0x2c);				// 储存器写 
}

/**********************************************************************硬件配置*/

/*工具函数**********************************************************************/

/******************************************************************************
 * @brief	次方函数
 * @param	x	:	底数
 * @param	y	:	指数
 * @return	x^y
 ******************************************************************************/
static uint32_t __lcd_pow(uint32_t x, uint32_t y)
{
	uint32_t result = 1;	// 结果默认为1
	while (y --)			// 累乘y次
	{
		result *= x;		// 每次把x累乘到结果上
	}
	return result;
}

/**********************************************************************工具函数*/

/*功能函数**********************************************************************/

/******************************************************************************
 * @brief	LCD填充颜色
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	color	:	要填充的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_clear(lcd_dev_t *dev, uint16_t color)
{          
	uint16_t i, j;
    
    __lcd_set_windows(dev, 0, 0, dev->width, dev->height);

	__lcd_dc_write(dev, 1);

    for(i = 0; i < LCD_W; i++)
	{
        for(j = 0; j < LCD_H; j++)
		{
			__lcd_write_halfword(dev, color);
        }
     }
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	填充起点
 * @param	width	:	填充的宽度
 * @param	height	:	填充的高度
 * @param	color	:	要填充的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_fill(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
	uint16_t i, j;

	__lcd_set_windows(dev, x, y, x + width - 1, y + height - 1);	// 设置显示范围
	for (i = y; i < y + height; i++)
	{													   	 	
		for (j = x; j < x + width; j++)
		{
			__lcd_write_halfword(dev, color);
		}
	}
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色，用于LVGL
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x1		:	填充起点x坐标
 * @param	y1		:	填充起点y坐标
 * @param	x2		:	填充终点x坐标
 * @param	y2		:	填充终点y坐标
 * @param	color	:	要填充的颜色数组
 * @return	无
 ******************************************************************************/
static void __lcd_color_fill(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color)
{
    uint16_t height, width;
    uint16_t i, j;
    width = x2 - x1 + 1;            // 填充的宽度
    height = y2 - y1 + 1;           // 填充的高度

	__lcd_set_windows(dev, x1, y1, x2, y2);						// 设置光标位置
	
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            __lcd_write_halfword(dev, color[i * width + j]);	// 写入数据
        }
    }
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色，并使用DMA传输，用于LVGL
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x1		:	填充起点x坐标
 * @param	y1		:	填充起点y坐标
 * @param	x2		:	填充终点x坐标
 * @param	y2		:	填充终点y坐标
 * @param	size	:	传输大小
 * @return	无
 ******************************************************************************/
static void __lcd_color_fill_dma(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size)
{
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
	
	__lcd_set_windows(dev, x1, y1, x2, y2);

	__lcd_dc_write(dev, 1);										// 写数据

	priv_data->spi.start(&priv_data->spi);						// 拉低CS，开始通信
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	/* STM32F10X代码待编写 */
	#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)
	/* 开启SPI的DMA接收 */
	SPI_I2S_DMACmd(dev->config.spix, SPI_I2S_DMAReq_Tx, ENABLE);

	/* 使能DMA */
	DMA_Cmd(__lcd_get_dma_stream(dev->config.spix), DISABLE);
	while (DMA_GetCmdStatus(__lcd_get_dma_stream(dev->config.spix)) != DISABLE);
	DMA_SetCurrDataCounter(__lcd_get_dma_stream(dev->config.spix), size);
	DMA_Cmd(__lcd_get_dma_stream(dev->config.spix), ENABLE);
	
	/* 等待传输完成 */
	while (DMA_GetFlagStatus(__lcd_get_dma_stream(dev->config.spix),
							 __lcd_get_dma_flag(dev->config.spix)) == RESET);

	/* 清除标志位 */
	DMA_ClearFlag(__lcd_get_dma_stream(dev->config.spix),
				  __lcd_get_dma_flag(dev->config.spix));
	
	priv_data->spi.stop(&priv_data->spi);						// 拉高CS，结束通信
	#endif
}

/******************************************************************************
 * @brief	LCD显示单个字符
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chr		:	要显示的字符
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_char(lcd_dev_t *dev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	uint8_t temp, sizex, t, m = 0;
	uint16_t i, typeface_num;							// 一个字符所占字节大小
	uint16_t x0 = x;
	sizex = size / 2;
	typeface_num = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * size;
	chr = chr - ' ';    								// 得到偏移后的值

	for (i = 0; i < typeface_num; i++)
	{ 
		if (size==12)temp=lcd_f6x12[chr][i];			// 调用6x12字体
		else if (size==16)temp=lcd_f8x6[chr][i];		// 调用8x16字体
		else if (size==24)temp=lcd_f12x24[chr][i];		// 调用12x24字体
		else if (size==32)temp=lcd_f16x32[chr][i];		// 调用16x32字体
		else return;

		for (t = 0;t < 8;t++)
		{
			if (!mode)									// 非叠加模式
			{
				if (temp & (0x01 << t)) __lcd_draw_point(dev, x, y, fc);
                else __lcd_draw_point(dev, x, y, bc);
                m++;
                x++;
                if (m % sizex == 0)
                {
                    m = 0;
                    x = x0;
                    y++;
                    break;
                }
			}
			else										// 叠加模式
			{
				if (temp & (0x01 << t)) __lcd_draw_point(dev, x, y, fc);
                x++;
                if ((x - x0) == sizex)
                {
                    x = x0;
                    y++;
                    break;
                }
			}
		}
	}
}

/******************************************************************************
 * @brief	LCD显示字符串
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	str		:	要显示的字符
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_string(lcd_dev_t *dev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{         
	while (*str != '\0')
	{       
		__lcd_show_char(dev, x, y, *str, fc, bc, size, mode);
		x += size / 2;
		str++;
	}
}

/******************************************************************************
 * @brief	LCD显示整数变量
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	num		:	要显示整数变量
 * @param	len		:	要显示的位数
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_num(lcd_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{         	
	u8 t, temp;
    u8 enshow = 0;

    for (t = 0; t < len; t++)
    {
        temp = (num / __lcd_pow(10, len - t - 1)) % 10;

        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                __lcd_show_char(dev, x + (size / 2)*t, y, ' ', fc, bc, size, mode);
                continue;
            }
            else enshow = 1;
        }

        __lcd_show_char(dev, x + (size / 2)*t, y, temp + '0', fc, bc, size, mode);
    }
}

/******************************************************************************
 * @brief	LCD显示十六进制数
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	num		:	指定要显示的数字，范围：0x00000000~0xFFFFFFFF
 * @param	len		:	指定数字的长度，范围：0~8
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_hex_num(lcd_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	uint8_t i, single_num;

	for (i = 0; i < len; i++)			// 遍历数字的每一位
	{
		/* 以十六进制提取数字的每一位 */
		single_num = num / __lcd_pow(16, len - i - 1) % 16;
		
		if (single_num < 10)			// 单个数字小于10
		{
			/* 调用__oled_show_char函数，显示此数字 */
			/* + '0' 可将数字转换为字符格式 */
			__lcd_show_char(dev, x + i * size / 2, y, single_num + '0', fc, bc, size, mode);
		}
		else							// 单个数字大于10
		{
			/* 调用__oled_show_char函数，显示此数字 */
			/* + 'A' 可将数字转换为从A开始的十六进制字符 */
			__lcd_show_char(dev, x + i * size / 2, y, single_num - 10 + 'A', fc, bc, size, mode);
		}
	}
}

/******************************************************************************
 * @brief	LCD显示两位小数变量
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	num		:	要显示整数变量
 * @param	int_len	:	要显示的整数位数
 * @param	fra_len	:	要显示的小数位数
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_float_num(lcd_dev_t *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
    uint32_t pow_num, int_num, fra_num;
	
	if (num < 0)
	{
		__lcd_show_char(dev, x, y, '-', fc, bc, size, mode);	// 显示-号
	}
	
	/* 提取整数部分和小数部分 */
	int_num = num;						// 直接赋值给整型变量，提取整数
	num -= int_num;						// 将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	pow_num = __lcd_pow(10, fra_len);	// 根据指定小数的位数，确定乘数
	fra_num = round(num * pow_num);		// 将小数乘到整数，同时四舍五入，避免显示误差
	int_num += fra_num / pow_num;		// 若四舍五入造成了进位，则需要再加给整数
	
	if (num >= 0)
	{
		__lcd_show_num(dev, x, y, int_num, int_len, fc, bc, size, mode);							// 显示整数部分
		__lcd_show_char(dev, x + (int_len) * size / 2, y, '.', fc, bc, size, mode);					// 显示小数点
		__lcd_show_num(dev, x + (int_len + 1) * size / 2, y, fra_num, fra_len, fc, bc, size, mode);	// 显示小数部分
	}
	else
	{
		num = -num;
		__lcd_show_num(dev, x + size / 2, y, int_num, int_len, fc, bc, size, mode);					// 显示整数部分
		__lcd_show_char(dev, x + (int_len + 1) * size / 2, y, '.', fc, bc, size, mode);				// 显示小数点
		__lcd_show_num(dev, x + (int_len + 2) * size / 2, y, fra_num, fra_len, fc, bc, size, mode);	// 显示小数部分
	}
}

/******************************************************************************
 * @brief	LCD显示单个12x12汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese12x12(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(12); 					// 汉字数目
	uint16_t typeface_num = (12 / 8 + ((12 % 8) ? 1 : 0)) * 12;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++)
	{
		if ((lcd_f12x12[k].index[0] == *(chinese)) && (lcd_f12x12[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_windows(dev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (lcd_f12x12[k].msk[i] & (0x01 << j))
					__lcd_draw_point(dev, x, y, fc); // 画一个点
					else if (mode == 0)
						__lcd_draw_point(dev, x, y, bc);
					x++;
					if ((x - x0) == 12)
					{
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示单个16x16汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese16x16(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(16); 					// 汉字数目
	uint16_t typeface_num = (16 / 8 + ((16 % 8) ? 1 : 0)) * 16;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++)
	{
		if ((lcd_f16x16[k].index[0] == *(chinese)) && (lcd_f16x16[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_windows(dev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (lcd_f16x16[k].msk[i] & (0x01 << j))
						__lcd_draw_point(dev, x, y, fc); // 画一个点
					else if (mode == 0)
						__lcd_draw_point(dev, x, y, bc);
					x++;
					if ((x - x0) == 16)
					{
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示单个24x24汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese24x24(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(24); 					// 汉字数目
	uint16_t typeface_num = (24 / 8 + ((24 % 8) ? 1 : 0)) * 24;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++)
	{
		if ((lcd_f24x24[k].index[0] == *(chinese)) && (lcd_f24x24[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_windows(dev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (lcd_f24x24[k].msk[i] & (0x01 << j))
						__lcd_draw_point(dev, x, y, fc); // 画一个点
					else if (mode == 0)
						__lcd_draw_point(dev, x, y, bc);
					x++;
					if ((x - x0) == 24)
					{
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示单个32x32汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese32x32(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t chinese_num = get_chinese_num(32); 					// 汉字数目
	uint16_t typeface_num = (32 / 8 + ((32 % 8) ? 1 : 0)) * 32;		// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chinese_num; k++)
	{
		if ((lcd_f32x32[k].index[0] == *(chinese)) && (lcd_f32x32[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_windows(dev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (lcd_f32x32[k].msk[i] & (0x01 << j))
						__lcd_draw_point(dev, x, y, fc); //画一个点
					else if (mode == 0)
						__lcd_draw_point(dev, x, y, bc);
					x++;
					if ((x - x0) == 32)
					{
						x = x0;
						y++;
						break;
					}
				}
			}
		}
		continue; // 查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示汉字串
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chinese	:	要显示的汉字串
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	while (*chinese != 0)
	{
		if (size == LCD_12X12)		__lcd_show_chinese12x12(dev, x, y, chinese, fc, bc, mode);
		else if (size == LCD_16X16)	__lcd_show_chinese16x16(dev, x, y, chinese, fc, bc, mode);
		else if (size == LCD_24X24)	__lcd_show_chinese24x24(dev, x, y, chinese, fc, bc, mode);
		else if (size == LCD_32X32)	__lcd_show_chinese32x32(dev, x, y, chinese, fc, bc, mode);
		else return;
		
		chinese += LCD_CHN_CHAR_WIDTH;
		x += size;
	}
}

/******************************************************************************
 * @brief	LCD显示图片
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x		:	图片左上角x坐标
 * @param	y		:	图片左上角y坐标
 * @param	width	:	图片宽度
 * @param	height	:	图片高度
 * @param	pic[]	:	图片数组
 * @return	无
 ******************************************************************************/
static void __lcd_show_image(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{
	uint16_t i, j; 
	uint32_t k = 0;

	__lcd_set_windows(dev, x, y, x + width - 1, y + height - 1);	// 设置显示范围
	for (i = 0; i < height; i++)
	{													   	 	
		for (j = 0; j < width; j++)
		{
			__lcd_write_byte(dev, pic[k * 2]);
			__lcd_write_byte(dev, pic[k * 2 + 1]);
			k++;
		}
	}
}

/******************************************************************************
 * @brief	LCD画点
 * @param	dev		:  lcd_dev_t 结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_point(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t color)
{
	__lcd_set_windows(dev, x, y, x, y);		// 设置光标位置 
	__lcd_write_halfword(dev, color);
}

/******************************************************************************
 * @brief	LCD画线
 * @param	dev		:  lcd_dev_t 结构体指针
 * @param	x1		:  起始x坐标
 * @param	y1		:  起始y坐标
 * @param	x2		:  终点x坐标
 * @param	y2		:  终点y坐标
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_line(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1;              // 计算坐标增量
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0)incx = 1;       // 设置单步方向
    else if (delta_x == 0)incx = 0; // 垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)incy = 1;
    else if (delta_y == 0)incy = 0; // 水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if ( delta_x > delta_y)distance = delta_x; // 选取基本增量坐标轴
    else distance = delta_y;

    for (t = 0; t <= distance + 1; t++ )    // 画线输出
    {
        __lcd_draw_point(dev, uRow, uCol, color); // 画点
        xerr += delta_x ;
        yerr += delta_y ;

        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

/******************************************************************************
 * @brief	LCD画矩形
 * @param	dev		:  lcd_dev_t 结构体指针
 * @param	x		:  矩形左上角x坐标
 * @param	y		:  矩形左上角y坐标
 * @param	width	:  矩形宽度
 * @param	height	:  矩形高度
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_rectangle(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
	uint16_t x_end = x + width - 1;
    uint16_t y_end = y + height - 1;

	__lcd_draw_line(dev, x, y, x_end, y, color);
	__lcd_draw_line(dev, x, y, x, y_end, color);
	__lcd_draw_line(dev, x, y_end, x_end, y_end, color);
	__lcd_draw_line(dev, x_end, y, x_end, y_end, color);
}

/******************************************************************************
 * @brief	LCD画圆
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	圆心坐标
 * @param	radius	:	半径
 * @param	color	:	圆的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_circle(lcd_dev_t *dev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color)
{
	int a, b;
	a = 0;
	b = radius;
	
	while (a <= b)
	{
		__lcd_draw_point(dev, x - b, y - a, color);	// 3           
		__lcd_draw_point(dev, x + b, y - a, color);	// 0           
		__lcd_draw_point(dev, x - a, y + b, color);	// 1                
		__lcd_draw_point(dev, x - a, y - b, color);	// 2             
		__lcd_draw_point(dev, x + b, y + a, color);	// 4               
		__lcd_draw_point(dev, x + a, y - b, color);	// 5
		__lcd_draw_point(dev, x + a, y + b, color);	// 6 
		__lcd_draw_point(dev, x - b, y + a, color);	// 7
		a++;
		if ((a * a + b * b) > (radius * radius))	// 判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
 * @brief	去初始化LCD
 * @param	dev   :  OLEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __lcd_deinit(lcd_dev_t *dev)
{
	if (!dev || !dev->init_flag)
	return -1;
	
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
	
	/* 去初始化硬件SPI */
	priv_data->spi.deinit(&priv_data->spi);

	/* 去初始化PWM */
	priv_data->pwm.deinit(&priv_data->pwm);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	return 0;
}

/**********************************************************************功能函数*/
