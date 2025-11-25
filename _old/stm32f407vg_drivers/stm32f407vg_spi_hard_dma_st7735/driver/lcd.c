#include "lcd.h"
#include "lcd_data.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
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

#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)

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

/**
  * LCD显存数组
  * 在使用DMA传输时，所有的显示函数，都只是对此显存数组进行读写
  * 随后调用__lcd_update函数或__lcd_updateArea函数启动一次DMA传输
  * 才会将显存数组的数据发送到LCD硬件，进行显示
  * LCD屏幕的像素为128*160，每个像素存放16位的颜色数据，故定义类型为uint8_t时，LCD_W需要*2
  */
uint8_t g_lcd_display_buf[LCD_H][LCD_W * 2] = {0};
											
/* LCD私有数据结构体 */
typedef struct {
	spi_hard_dev_t spi;	// 硬件SPI设备
} lcd_priv_data_t;

static void __lcd_dma_init(lcd_dev_t *dev);

/* 通信协议 */
static void __lcd_res_write(lcd_dev_t *dev, uint8_t bit_value);
static void __lcd_dc_write(lcd_dev_t *dev, uint8_t bit_value);
static void __lcd_bl_write(lcd_dev_t *dev, uint8_t bit_value);
static void __lcd_write_command(lcd_dev_t *dev, uint8_t command);
static void __lcd_write_byte(lcd_dev_t *dev, uint8_t data);
static void __lcd_write_halfword(lcd_dev_t *dev, uint16_t dat);

/* 功能函数 */
static void __lcd_update(lcd_dev_t *dev);
static void __lcd_fill(lcd_dev_t *dev, uint16_t color);
static void __lcd_fill_area(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
static void __lcd_show_char(lcd_dev_t *dev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size);
static void __lcd_show_string(lcd_dev_t *dev, uint16_t x, uint16_t y, char *string,uint16_t fc, uint16_t bc, uint8_t size);
static void __lcd_show_num(lcd_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc , uint8_t size);
static void __lcd_show_float_num(lcd_dev_t *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, uint16_t fc, uint16_t bc, uint8_t size);
static void __lcd_show_chinese(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t size);
static void __lcd_show_image(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
static void __lcd_draw_point(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t color);
static void __lcd_draw_line(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
static void __lcd_draw_rectangle(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t is_filled);
static void __lcd_draw_circle(lcd_dev_t *dev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color);
static int __lcd_deinit(lcd_dev_t *dev);

/******************************************************************************
 * @brief	初始化LCD
 * @param	dev	:	lcd_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int lcd_init(lcd_dev_t *dev)
{
	if (!dev)
		return -1;
	
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
	priv_data->spi.config.prescaler = dev->config.prescaler;
	priv_data->spi.config.mode = dev->config.mode;
	
	/* 配置硬件SPI */
	spi_hard_init(&priv_data->spi);
	
	/* 配置时钟与GPIO */
	__lcd_io_clock_enable(dev->config.res_port);
	__lcd_io_clock_enable(dev->config.dc_port);
	__lcd_io_clock_enable(dev->config.bl_port);
	
	__lcd_config_io_out_pp(dev->config.res_port, dev->config.res_pin);
	__lcd_config_io_out_pp(dev->config.dc_port, dev->config.dc_pin);
	__lcd_config_io_out_pp(dev->config.bl_port, dev->config.bl_pin);
	
	/* 配置DMA */
	if(dev->config.use_dma == LCD_USE_DMA)
	{
		__lcd_dma_init(dev);
	}
	
	/* 函数指针赋值 */
	dev->update = __lcd_update;
	dev->fill = __lcd_fill;
	dev->fill_area = __lcd_fill_area;
	dev->show_char = __lcd_show_char;
	dev->show_string = __lcd_show_string;
	dev->show_num = __lcd_show_num;
	dev->show_float_num = __lcd_show_float_num;
	dev->show_chinese = __lcd_show_chinese;
	dev->show_image = __lcd_show_image;
	dev->draw_point = __lcd_draw_point;
	dev->draw_line = __lcd_draw_line;
	dev->draw_rectangle = __lcd_draw_rectangle;
	dev->draw_circle = __lcd_draw_circle;
	dev->deinit = __lcd_deinit;

	dev->init_flag = true;
	
	/* 配置LCD */
	__lcd_res_write(dev, 0);	// 复位
	LCD_DELAY_MS(100);
	__lcd_res_write(dev, 1);
	LCD_DELAY_MS(100);
	__lcd_bl_write(dev, 1);		// 打开背光
	LCD_DELAY_MS(100);
	/* 启动初始序列 */
	__lcd_write_command(dev, 0x11); // Sleep out 
	LCD_DELAY_MS(120);
	/* ST7735S Frame Rate */
	__lcd_write_command(dev, 0xB1);
	__lcd_write_byte(dev, 0x05);
	__lcd_write_byte(dev, 0x3C);
	__lcd_write_byte(dev, 0x3C);
	__lcd_write_command(dev, 0xB2);
	__lcd_write_byte(dev, 0x05);
	__lcd_write_byte(dev, 0x3C);
	__lcd_write_byte(dev, 0x3C);
	__lcd_write_command(dev, 0xB3);
	__lcd_write_byte(dev, 0x05);
	__lcd_write_byte(dev, 0x3C); 
	__lcd_write_byte(dev, 0x3C);
	__lcd_write_byte(dev, 0x05);
	__lcd_write_byte(dev, 0x3C);
	__lcd_write_byte(dev, 0x3C);
	/* Dot inversion */
	__lcd_write_command(dev, 0xB4);
	__lcd_write_byte(dev, 0x03);
	/* ST7735S Power Sequence */
	__lcd_write_command(dev, 0xC0);
	__lcd_write_byte(dev, 0x28);
	__lcd_write_byte(dev, 0x08);
	__lcd_write_byte(dev, 0x04);
	__lcd_write_command(dev, 0xC1);
	__lcd_write_byte(dev, 0XC0);
	__lcd_write_command(dev, 0xC2);
	__lcd_write_byte(dev, 0x0D);
	__lcd_write_byte(dev, 0x00);
	__lcd_write_command(dev, 0xC3);
	__lcd_write_byte(dev, 0x8D);
	__lcd_write_byte(dev, 0x2A);
	__lcd_write_command(dev, 0xC4);
	__lcd_write_byte(dev, 0x8D);
	__lcd_write_byte(dev, 0xEE);
	/* VCOM */
	__lcd_write_command(dev, 0xC5);
	__lcd_write_byte(dev, 0x1A);
	/* MX, MY, RGB mode */
	__lcd_write_command(dev, 0x36);
	if(LCD_DIRECTION==0)			// 竖屏正向
		__lcd_write_byte(dev, 0xC0);
	else if(LCD_DIRECTION==1)		// 竖屏反向
		__lcd_write_byte(dev, 0x00);
	else if(LCD_DIRECTION==2)		// 横屏正向
		__lcd_write_byte(dev, 0x70);
	else							// 横屏反向
		__lcd_write_byte(dev, 0xA0);
	/* ST7735S Gamma Sequence */ 
	__lcd_write_command(dev, 0xE0);
	__lcd_write_byte(dev, 0x04);
	__lcd_write_byte(dev, 0x22);
	__lcd_write_byte(dev, 0x07);
	__lcd_write_byte(dev, 0x0A);
	__lcd_write_byte(dev, 0x2E);
	__lcd_write_byte(dev, 0x30);
	__lcd_write_byte(dev, 0x25);
	__lcd_write_byte(dev, 0x2A);
	__lcd_write_byte(dev, 0x28);
	__lcd_write_byte(dev, 0x26);
	__lcd_write_byte(dev, 0x2E);
	__lcd_write_byte(dev, 0x3A);
	__lcd_write_byte(dev, 0x00);
	__lcd_write_byte(dev, 0x01);
	__lcd_write_byte(dev, 0x03);
	__lcd_write_byte(dev, 0x13);
	__lcd_write_command(dev, 0xE1);
	__lcd_write_byte(dev, 0x04);
	__lcd_write_byte(dev, 0x16);
	__lcd_write_byte(dev, 0x06);
	__lcd_write_byte(dev, 0x0D);
	__lcd_write_byte(dev, 0x2D);
	__lcd_write_byte(dev, 0x26);
	__lcd_write_byte(dev, 0x23);
	__lcd_write_byte(dev, 0x27);
	__lcd_write_byte(dev, 0x27);
	__lcd_write_byte(dev, 0x25);
	__lcd_write_byte(dev, 0x2D);
	__lcd_write_byte(dev, 0x3B);
	__lcd_write_byte(dev, 0x00);
	__lcd_write_byte(dev, 0x01);
	__lcd_write_byte(dev, 0x04);
	__lcd_write_byte(dev, 0x13);
	/* 65k mode */
	__lcd_write_command(dev, 0x3A);
	__lcd_write_byte(dev, 0x05);
	/* Display on */
	__lcd_write_command(dev, 0x29);
	
	/* 屏幕初始化为黑色 */
	dev->fill(dev, BLACK);
	
	return 0;
}

/******************************************************************************
 * @brief	LCD配置DMA传输
 * @param	dev	:	lcd_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_dma_init(lcd_dev_t *dev)
{
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	__lcd_dma_clock_enable(dev->config.spix);	// 开启DMA时钟
	DMA_DeInit(__lcd_get_dma_channel(dev->config.spix));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.spix->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_lcd_display_buf;			// 内存地址
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
	
	#elif defined(STM32F40_41xxx) || defined(STM32F429_439xx)
	
    __lcd_dma_clock_enable(dev->config.spix);	// 开启DMA时钟
	
	DMA_DeInit(__lcd_get_dma_stream(dev->config.spix));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = __lcd_get_dma_channel(dev->config.spix);		// 选择DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&dev->config.spix->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)g_lcd_display_buf;			// 内存地址
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

/*通信协议*********************************************************************/

/******************************************************************************
 * @brief	LCD写RES高低电平
 * @param	dev			:	lcd_dev_t 结构体指针
 * @param	bit_value	:	要写入RES的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_res_write(lcd_dev_t *dev, uint8_t bit_value)
{
	__lcd_io_write(dev->config.res_port, dev->config.res_pin, bit_value);
}

/******************************************************************************
 * @brief	LCD写DC高低电平
 * @param	dev			:	lcd_dev_t 结构体指针
 * @param	bit_value	:	要写入DC的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_dc_write(lcd_dev_t *dev, uint8_t bit_value)
{
	__lcd_io_write(dev->config.dc_port, dev->config.dc_pin, bit_value);
}

/******************************************************************************
 * @brief	LCD写BL高低电平
 * @param	dev			:	lcd_dev_t 结构体指针
 * @param	bit_value	:	要写入DC的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_bl_write(lcd_dev_t *dev, uint8_t bit_value)
{
	__lcd_io_write(dev->config.bl_port, dev->config.bl_pin, bit_value);
}

/******************************************************************************
 * @brief	LCD写命令
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	byte	:	要写入的命令值，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_command(lcd_dev_t *dev, uint8_t command)
{
	__lcd_dc_write(dev, 0);			// 写命令
	__lcd_write_byte(dev, command);
	__lcd_dc_write(dev, 1);			// 写数据
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
	
	priv_data->spi.start(&priv_data->spi);					// 拉低CS，开始通信
	priv_data->spi.swap_byte(&priv_data->spi, byte);		// 写入指定命令
	priv_data->spi.stop(&priv_data->spi);					// 拉高CS，结束通信
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
	
	priv_data->spi.start(&priv_data->spi);						// 拉低CS，开始通信
	priv_data->spi.swap_byte(&priv_data->spi, halfword >> 8);	// 写入数据
	priv_data->spi.swap_byte(&priv_data->spi, halfword);		// 写入数据
	priv_data->spi.stop(&priv_data->spi);						// 拉高CS，结束通信
}

/**********************************************************************通信协议*/

/*硬件配置**********************************************************************/

/******************************************************************************
 * @brief	LCD设置起始和结束地址
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x1,x2	:	设置列的起始和结束地址
 * @param	y1,y2	:	设置行的起始和结束地址
 * @return	无
 ******************************************************************************/
static void __lcd_set_address(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if(LCD_DIRECTION == 0 || LCD_DIRECTION == 1)			// 竖屏
	{
		__lcd_write_command(dev, 0x2a);					// 列地址设置
		__lcd_write_halfword(dev, x1 + 2);
		__lcd_write_halfword(dev, x2 + 2);
		__lcd_write_command(dev, 0x2b);					// 行地址设置
		__lcd_write_halfword(dev, y1 + 1);
		__lcd_write_halfword(dev, y2 + 1);
		__lcd_write_command(dev, 0x2c);					// 储存器写
	}
	else if(LCD_DIRECTION == 2 || LCD_DIRECTION == 3)		// 横屏
	{
		__lcd_write_command(dev, 0x2a);					// 列地址设置
		__lcd_write_halfword(dev, x1 + 1);
		__lcd_write_halfword(dev, x2 + 1);
		__lcd_write_command(dev, 0x2b);					// 行地址设置
		__lcd_write_halfword(dev, y1 + 2);
		__lcd_write_halfword(dev, y2 + 2);
		__lcd_write_command(dev, 0x2c);					// 储存器写
	}
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
 * @brief	LCD DMA传输更新显存
 * @param	dev	:	lcd_dev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_update(lcd_dev_t *dev)
{
	if(dev->config.use_dma == LCD_USE_DMA)
	{
		lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
		
		/* 将整个数据搬运一次到DMA */
		__lcd_set_address(dev, 0, 0, LCD_W - 1, LCD_H - 1);
		
		/* 重新设置缓存大小，开启一次DMA传输 */
		#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
		
		DMA_Cmd(__lcd_get_dma_channel(dev->config.spix), DISABLE);							// 关闭DMA
		DMA_SetCurrDataCounter(__lcd_get_dma_channel(dev->config.spix), LCD_W * LCD_H * 2);	// 数据传输量
		DMA_ClearFlag(__lcd_get_dma_flag(dev->config.spix));
		
		priv_data->spi.cs_write(&priv_data->spi, 0);
		__lcd_res_write(dev, 1);
		
		SPI_I2S_DMACmd(dev->config.spix, SPI_I2S_DMAReq_Tx, ENABLE);							// 开启SPI的DMA接收
		DMA_Cmd(__lcd_get_dma_channel(dev->config.spix), ENABLE);							// 使能DMA
		
		while(!DMA_GetFlagStatus(__lcd_get_dma_flag(dev->config.spix)));						// 等待传输完成
		DMA_ClearFlag(__lcd_get_dma_flag(dev->config.spix));
		
		#elif defined(STM32F40_41xxx)
		
		DMA_Cmd(__lcd_get_dma_stream(dev->config.spix), DISABLE);							// 关闭DMA
		DMA_SetCurrDataCounter(__lcd_get_dma_stream(dev->config.spix), LCD_W * LCD_H * 2);	// 数据传输量
		
		DMA_ClearFlag(	__lcd_get_dma_stream(dev->config.spix), 
						__lcd_get_dma_flag(dev->config.spix)	);
		
		priv_data->spi.cs_write(&priv_data->spi, 0);
		__lcd_res_write(dev, 1);
		
		SPI_I2S_DMACmd(dev->config.spix, SPI_I2S_DMAReq_Tx, ENABLE);							// 开启SPI的DMA接收
		DMA_Cmd(__lcd_get_dma_stream(dev->config.spix), ENABLE);								// 使能DMA
		
		while(!DMA_GetFlagStatus(	__lcd_get_dma_stream(dev->config.spix), 					// 等待传输完成
									__lcd_get_dma_flag(dev->config.spix)		));
		
		DMA_ClearFlag(	__lcd_get_dma_stream(dev->config.spix),
						__lcd_get_dma_flag(dev->config.spix)	);
		
		#endif
		
		priv_data->spi.cs_write(&priv_data->spi, 1);
	}
}

/******************************************************************************
 * @brief	LCD填充颜色
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	color	:	要填充的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_fill(lcd_dev_t *dev, uint16_t color)
{          
	uint16_t i, j;
	
	if(dev->config.use_dma == LCD_USE_DMA)
	{
		for(i = 0 ; i < LCD_H; i++)
		{
			for(j = 0;j < LCD_W * 2; j += 2)
			{
				g_lcd_display_buf[i][j] = color >> 8;
				g_lcd_display_buf[i][j + 1] = color;
			}
		}
	}
	else if(dev->config.use_dma == LCD_NONUSE_DMA)
	{
		__lcd_set_address(dev, 0, 0, LCD_W - 1, LCD_H - 1);// 设置显示范围
		for(i = 0; i < LCD_H; i++)
		{													   	 	
			for(j = 0; j < LCD_W; j++)
			{
				__lcd_write_halfword(dev, color);
			}
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
static void __lcd_fill_area(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{          
	uint16_t i, j;
	
	if(dev->config.use_dma == LCD_USE_DMA)
	{
		for(i = y ; i < y + height; i++)
		{
			for(j = x * 2; j < (x + width) * 2; j += 2)
			{
				g_lcd_display_buf[i][j] = color >> 8;
				g_lcd_display_buf[i][j + 1] = color;
			}
		}
	}
	else if(dev->config.use_dma == LCD_NONUSE_DMA)
	{
		__lcd_set_address(dev, x, y, x + width - 1, y + height - 1);// 设置显示范围
		for(i = y; i < y + height; i++)
		{													   	 	
			for(j = x; j < x + width; j++)
			{
				__lcd_write_halfword(dev, color);
			}
		}
	}
}

/******************************************************************************
 * @brief	LCD显示单个字符
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	chr		:	要显示的字符
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_char(lcd_dev_t *dev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size)
{
	uint8_t temp, sizex, t, m = 0;
	uint16_t i, TypefaceNum;										//一个字符所占字节大小
	uint16_t x0 = x;
	sizex = size / 2;
	TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * size;
	chr = chr - ' ';    								//得到偏移后的值
	__lcd_set_address(dev, x, y, x+sizex - 1, y+size - 1);	//设置光标位置 
	for(i = 0; i < TypefaceNum; i++)
	{ 
		if(size == 12)		temp=lcd_f6x12[chr][i];		//调用6x12字体
		else if(size == 16)	temp=lcd_f8x16[chr][i];		//调用8x16字体
		else if(size == 24)	temp=lcd_f12x24[chr][i];		//调用12x24字体
		else if(size == 32)	temp=lcd_f16x32[chr][i];		//调用16x32字体
		else return;
		for(t = 0; t < 8; t++)
		{
			if(dev->config.use_dma == LCD_USE_DMA)
			{
				if(temp & (0x01 << t))__lcd_draw_point(dev, x, y, fc);//画一个点
				else __lcd_draw_point(dev, x, y, bc);
				x++;
				if((x - x0) == sizex)
				{
					x = x0;
					y++;
					break;
				}
			}
			else if(dev->config.use_dma == LCD_NONUSE_DMA)
			{
				if(temp & (0x01 << t))__lcd_write_halfword(dev, fc);
				else __lcd_write_halfword(dev, bc);
				m++;
				if(m % sizex == 0)
				{
					m = 0;
					break;
				}
			}
		}
	}   	 	  
}

/******************************************************************************
 * @brief	LCD显示字符串
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	string	:	要显示的字符串
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_string(lcd_dev_t *dev, uint16_t x, uint16_t y, char *string, uint16_t fc, uint16_t bc, uint8_t size)
{         
	while(*string!='\0')
	{       
		__lcd_show_char(dev, x, y, *string, fc, bc, size);
		x += size / 2;
		string++;
	}
}

/******************************************************************************
 * @brief	LCD显示整数变量
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	num		:	要显示整数变量
 * @param	len		:	要显示的位数
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_num(lcd_dev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size)
{         	
	uint8_t t, temp;
	uint8_t enshow = 0;
	uint8_t sizex = size / 2;
	for(t = 0; t < len; t++)
	{
		temp = (num / __lcd_pow(10, len - t - 1)) % 10;
		if(enshow == 0 && t < (len - 1))
		{
			if(temp == 0)
			{
				__lcd_show_char(dev, x + t * sizex, y, ' ', fc, bc, size);
				continue;
			}else enshow = 1; 
		 	 
		}
	 	__lcd_show_char(dev, x + t * sizex, y, temp + 48, fc, bc, size);
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
static void __lcd_show_float_num(lcd_dev_t *dev, uint16_t x, uint16_t y, float num, uint8_t int_len, uint8_t fra_len, uint16_t fc, uint16_t bc, uint8_t size)
{
    uint32_t powNum, intNum, fraNum;
	
	if (num < 0)
	{
		__lcd_show_char(dev, x, y, '-', fc, bc, size);	// 显示-号
	}
	
	/* 提取整数部分和小数部分 */
	intNum = num;						// 直接赋值给整型变量，提取整数
	num -= intNum;						// 将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	powNum = __lcd_pow(10, fra_len);		// 根据指定小数的位数，确定乘数
	fraNum = round(num * powNum);		// 将小数乘到整数，同时四舍五入，避免显示误差
	intNum += fraNum / powNum;			// 若四舍五入造成了进位，则需要再加给整数
	
	if (num >= 0)
	{
		__lcd_show_num(dev, x, y, intNum, int_len, fc, bc, size);								// 显示整数部分
		__lcd_show_char(dev, x + (int_len) * size / 2, y, '.', fc, bc, size);					// 显示小数点
		__lcd_show_num(dev, x + (int_len + 1) * size / 2, y, fraNum, fra_len, fc, bc, size);		// 显示小数部分
	}
	else
	{
		num = -num;
		__lcd_show_num(dev, x + size / 2, y, intNum, int_len, fc, bc, size);					// 显示整数部分
		__lcd_show_char(dev, x + (int_len + 1) * size / 2, y, '.', fc, bc, size);				// 显示小数点
		__lcd_show_num(dev, x + (int_len + 2) * size / 2, y, fraNum, fra_len, fc, bc, size);		// 显示小数部分
	}
}

/******************************************************************************
 * @brief	LCD显示单个12x12汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese12x12(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc)
{
	uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t chineseNum = get_chinese_num(12); 					//汉字数目
	uint16_t TypefaceNum = (12 / 8 + ((12 % 8) ? 1 : 0)) * 12;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chineseNum; k++)
	{
		if ((lcd_cf12x12[k].index[0] == *(chinese)) && (lcd_cf12x12[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_address(dev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(dev->config.use_dma == LCD_USE_DMA)
					{
						if (lcd_cf12x12[k].msk[i] & (0x01 << j))
							__lcd_draw_point(dev, x, y, fc); //画一个点
						else
							__lcd_draw_point(dev, x, y, bc);
						x++;
						if ((x - x0) == 12)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(dev->config.use_dma == LCD_NONUSE_DMA)
					{
						if (lcd_cf12x12[k].msk[i] & (0x01 << j))
							__lcd_write_halfword(dev, fc);
						else
							__lcd_write_halfword(dev, bc);
						m++;
						if (m % 12 == 0)
						{
							m = 0;
							break;
						}
					}
				}
			}
		}
		continue; //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示单个16x16汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese16x16(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc)
{
	uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t chineseNum = get_chinese_num(16); 					//汉字数目
	uint16_t TypefaceNum = (16 / 8 + ((16 % 8) ? 1 : 0)) * 16;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chineseNum; k++)
	{
		if ((lcd_cf16x16[k].index[0] == *(chinese)) && (lcd_cf16x16[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_address(dev, x, y, x + 16 - 1, y + 16 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(dev->config.use_dma == LCD_USE_DMA)
					{
						if (lcd_cf16x16[k].msk[i] & (0x01 << j))
							__lcd_draw_point(dev, x, y, fc); //画一个点
						else
							__lcd_draw_point(dev, x, y, bc);
						x++;
						if ((x - x0) == 16)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(dev->config.use_dma == LCD_NONUSE_DMA)
					{
						if (lcd_cf16x16[k].msk[i] & (0x01 << j))
							__lcd_write_halfword(dev, fc);
						else
							__lcd_write_halfword(dev, bc);
						m++;
						if (m % 16 == 0)
						{
							m = 0;
							break;
						}
					}
				}
			}
		}
		continue; //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示单个24x24汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese24x24(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc)
{
	uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t chineseNum = get_chinese_num(24); 					//汉字数目
	uint16_t TypefaceNum = (24 / 8 + ((24 % 8) ? 1 : 0)) * 24;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chineseNum; k++)
	{
		if ((lcd_cf24x24[k].index[0] == *(chinese)) && (lcd_cf24x24[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_address(dev, x, y, x + 24 - 1, y + 24 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(dev->config.use_dma == LCD_USE_DMA)
					{
						if (lcd_cf24x24[k].msk[i] & (0x01 << j))
							__lcd_draw_point(dev, x, y, fc); //画一个点
						else
							__lcd_draw_point(dev, x, y, bc);
						x++;
						if ((x - x0) == 24)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(dev->config.use_dma == LCD_NONUSE_DMA)
					{
						if (lcd_cf24x24[k].msk[i] & (0x01 << j))
							__lcd_write_halfword(dev, fc);
						else
							__lcd_write_halfword(dev, bc);
						m++;
						if (m % 24 == 0)
						{
							m = 0;
							break;
						}
					}
				}
			}
		}
		continue; //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示单个32x32汉字
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese32x32(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc)
{
		uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t chineseNum = get_chinese_num(32); 					//汉字数目
	uint16_t TypefaceNum = (32 / 8 + ((32 % 8) ? 1 : 0)) * 32;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < chineseNum; k++)
	{
		if ((lcd_cf32x32[k].index[0] == *(chinese)) && (lcd_cf32x32[k].index[1] == *(chinese + 1)))
		{
			__lcd_set_address(dev, x, y, x + 32 - 1, y + 32 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(dev->config.use_dma == LCD_USE_DMA)
					{
						if (lcd_cf32x32[k].msk[i] & (0x01 << j))
							__lcd_draw_point(dev, x, y, fc); //画一个点
						else
							__lcd_draw_point(dev, x, y, bc);
						x++;
						if ((x - x0) == 32)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(dev->config.use_dma == LCD_NONUSE_DMA)
					{
						if (lcd_cf32x32[k].msk[i] & (0x01 << j))
							__lcd_write_halfword(dev, fc);
						else
							__lcd_write_halfword(dev, bc);
						m++;
						if (m % 32 == 0)
						{
							m = 0;
							break;
						}
					}
				}
			}
		}
		continue; //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/******************************************************************************
 * @brief	LCD显示汉字串
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	显示坐标
 * @param	chinese	:	要显示的汉字串
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese(lcd_dev_t *dev, uint16_t x, uint16_t y, char *chinese, uint16_t fc, uint16_t bc, uint8_t size)
{
	while(*chinese != 0)
	{
		if(size==LCD_12X12)			__lcd_show_chinese12x12(dev, x, y, chinese, fc, bc);
		else if(size==LCD_16X16)	__lcd_show_chinese16x16(dev, x, y, chinese, fc, bc);
		else if(size==LCD_24X24)	__lcd_show_chinese24x24(dev, x, y, chinese, fc, bc);
		else if(size==LCD_32X32)	__lcd_show_chinese32x32(dev, x, y, chinese, fc, bc);
		else return;
		
		chinese += LCD_CHN_CHAR_WIDTH;
		x += size;
	}
}

/******************************************************************************
 * @brief	LCD显示图片
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	起点坐标
 * @param	width	:	图片宽度
 * @param	height	:	图片高度
 * @param	pic		:	图片数组
 * @return	无
 ******************************************************************************/
static void __lcd_show_image(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{
	uint16_t i, j; 
	uint32_t k = 0;
	
	if(dev->config.use_dma == LCD_USE_DMA)
	{
		for(i = y; i < y + height; i++)
		{
			for(j = x; j < x + width; j++)
			{
				g_lcd_display_buf[i][j * 2] = pic[k * 2];
                g_lcd_display_buf[i][j * 2 + 1] = pic[k * 2 + 1];
				k++;
			}
		}
	}
	else if(dev->config.use_dma == LCD_NONUSE_DMA)
	{
		__lcd_set_address(dev, x, y, x + width - 1, y + height - 1);	//设置显示范围
		for(i = 0; i < height; i++)
		{													   	 	
			for(j = 0; j < width; j++)
			{
				__lcd_write_byte(dev, pic[k * 2]);
				__lcd_write_byte(dev, pic[k * 2 + 1]);
				k++;
			}
		}
	}
}

/******************************************************************************
 * @brief	LCD在指定位置画点
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x,y		:	画点坐标
 * @param	color	:	点的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_point(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t color)
{
	if(dev->config.use_dma == LCD_USE_DMA)
	{
		/*使用DMA的话，从对点刷屏到对显存数组写入数据，DMA传输数据的时候再统一进行传输*/
		g_lcd_display_buf[y][x*2] = color >> 8;
		g_lcd_display_buf[y][x*2+1] = color;
	}
	else if(dev->config.use_dma == LCD_NONUSE_DMA)
	{
		__lcd_set_address(dev, x, y, x, y);		//设置光标位置 
		__lcd_write_halfword(dev, color);
	}
}

/******************************************************************************
 * @brief	LCD画线
 * @param	dev		:	lcd_dev_t 结构体指针
 * @param	x1,y1	:	起始坐标
 * @param	x2,y2	:	终止坐标
 * @param	color	:	线的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_line(lcd_dev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t; 
	int xerr = 0, yerr = 0, delta_x, delta_y, distance;
	int incx, incy, uRow, uCol;
	
	/*计算坐标增量*/
	delta_x = x2 - x1; 
	delta_y = y2 - y1;
	
	/*画线起点坐标*/
	uRow = x1;
	uCol = y1;
	
	/*设置单步方向 */
	if(delta_x > 0)
		incx = 1; 
	else if (delta_x == 0)	//垂直线 
		incx = 0;
	else
	{
		incx = -1;
		delta_x = -delta_x;
	}
	
	if(delta_y > 0)
		incy = 1;
	else if (delta_y == 0)	//水平线
		incy = 0; 
	else
	{
		incy = -1;
		delta_y = -delta_y;
	}
	
	/*选取基本增量坐标轴*/
	if(delta_x > delta_y)
		distance = delta_x;
	else
		distance = delta_y;
	
	/*画点*/
	for(t = 0; t < distance + 1; t++)
	{
		__lcd_draw_point(dev, uRow, uCol, color);
		xerr += delta_x;
		yerr += delta_y;
		if(xerr > distance)
		{
			xerr -= distance;
			uRow += incx;
		}
		if(yerr > distance)
		{
			yerr -= distance;
			uCol += incy;
		}
	}
}

/******************************************************************************
 * @brief	LCD画矩形
 * @param	dev			:	lcd_dev_t 结构体指针
 * @param	x			:	指定矩形左上角的横坐标
 * @param	y			:	指定矩形左上角的纵坐标
 * @param	width		:	指定矩形的宽度
 * @param	height		:	指定矩形的高度
 * @param	color		:	矩形的颜色
 * @param	is_filled	:	指定是否填充
 * @return	无
 ******************************************************************************/
static void __lcd_draw_rectangle(lcd_dev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t is_filled)
{
	uint8_t i, j;
	if (!is_filled)		//指定矩形不填充
	{
		/*遍历上下x坐标，画矩形上下两条线*/
		for (i = x; i < x + width; i ++)
		{
			__lcd_draw_point(dev, i, y, color);
			__lcd_draw_point(dev, i, y + height - 1, color);
		}
		/*遍历左右y坐标，画矩形左右两条线*/
		for (i = y; i < y + height; i ++)
		{
			__lcd_draw_point(dev, x, i, color);
			__lcd_draw_point(dev, x + width - 1, i, color);
		}
	}
	else				//指定矩形填充
	{
		/*遍历x坐标*/
		for (i = x; i < x + width; i ++)
		{
			/*遍历y坐标*/
			for (j = y; j < y + height; j ++)
			{
				/*在指定区域画点，填充满矩形*/
				__lcd_draw_point(dev, i, j, color);
			}
		}
	}
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
	
	while(a <= b)
	{
		__lcd_draw_point(dev, x - b, y - a, color);	//3           
		__lcd_draw_point(dev, x + b, y - a, color);	//0           
		__lcd_draw_point(dev, x - a, y + b, color);	//1                
		__lcd_draw_point(dev, x - a, y - b, color);	//2             
		__lcd_draw_point(dev, x + b, y + a, color);	//4               
		__lcd_draw_point(dev, x + a, y - b, color);	//5
		__lcd_draw_point(dev, x + a, y + b, color);	//6 
		__lcd_draw_point(dev, x - b, y + a, color);	//7
		a++;
		if((a * a + b * b) > (radius * radius))		//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
 * @brief	去初始化LCD
 * @param	dev	:	lcd_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __lcd_deinit(lcd_dev_t *dev)
{
	if (!dev || !dev->init_flag)
	return -1;
	
	lcd_priv_data_t *priv_data = (lcd_priv_data_t *)dev->priv_data;
	
	/* 去初始化硬件SPI */
	priv_data->spi.deinit(&priv_data->spi);
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
	dev->priv_data = NULL;
	
	dev->init_flag = false;	//修改初始化标志
	return 0;
}

/**********************************************************************功能函数*/
