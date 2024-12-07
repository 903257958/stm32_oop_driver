#include "lcd.h"
#include "lcd_data.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__lcd_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{lcd_log("lcd gpio clock no enable\r\n");} \
												}
													
#define	__lcd_config_dma_clock_enable(spix)	{	if(spix == SPI1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if(spix == SPI2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
												else if(spix == SPI3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);} \
												else					{lcd_log("lcd dma clock no enable\r\n");} \
											}
													
#define	__lcd_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
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
											
	#if !FREERTOS
	static void __lcd_delay_ms(uint32_t ms)
	{
		while(ms--)
		{
			SysTick->LOAD = 72 * 1000;				// 设置定时器重装值
			SysTick->VAL = 0x00;					// 清空当前计数值
			SysTick->CTRL = 0x00000005;				// 设置时钟源为HCLK，启动定时器
			while(!(SysTick->CTRL & 0x00010000));	// 等待计数到0
			SysTick->CTRL = 0x00000004;				// 关闭定时器
		}
	}
	#else
	static void __lcd_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#elif defined(STM32F40_41xxx)

#define	__lcd_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{lcd_log("lcd gpio clock no enable\r\n");} \
												}

#define	__lcd_config_dma_clock_enable(spix)	{	if(spix == SPI1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
												else if(spix == SPI2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
												else if(spix == SPI3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
												else					{lcd_log("lcd dma clock no enable\r\n");} \
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
											
	#if !FREERTOS
	static void __lcd_delay_ms(uint32_t ms)
	{
		while(ms--)
		{
			uint32_t temp;	    	 
			SysTick->LOAD = 1000 * 21; 					// 时间加载	  		 
			SysTick->VAL=0x00;        					// 清空计数器
			SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ; 	// 开始倒数 	 
			do
			{
				temp = SysTick->CTRL;
			}while((temp&0x01) && !(temp&(1<<16)));		// 等待时间到达   
			SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	// 关闭计数器
			SysTick->VAL = 0X00;       					// 清空计数器 
		}
	}
	#else
	static void __lcd_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#endif		

/**
  * LCD显存数组
  * 在使用DMA传输时，所有的显示函数，都只是对此显存数组进行读写
  * 随后调用__lcd_update函数或__lcd_updateArea函数启动一次DMA传输
  * 才会将显存数组的数据发送到LCD硬件，进行显示
  * LCD屏幕的像素为128*160，每个像素存放16位的颜色数据，故定义类型为uint8_t时，LCD_W需要*2
  */
uint8_t gLCDDisplayBuf[LCD_H][LCD_W * 2] = {0};
											
/* LCD私有数据结构体 */
typedef struct {
	SPIDev_t lcdSPI;					// 硬件SPI设备
}LCDPrivData_t;

static void __lcd_dma_init(LCDDev_t *pDev);

/* 通信协议 */
static void __lcd_res_write(LCDDev_t *pDev, uint8_t bitValue);
static void __lcd_dc_write(LCDDev_t *pDev, uint8_t bitValue);
static void __lcd_bl_write(LCDDev_t *pDev, uint8_t bitValue);
static void __lcd_write_command(LCDDev_t *pDev, uint8_t command);
static void __lcd_write_byte(LCDDev_t *pDev, uint8_t data);
static void __lcd_write_halfword(LCDDev_t *pDev, uint16_t dat);

/* 功能函数 */
static void __lcd_update(LCDDev_t *pDev);
static void __lcd_fill(LCDDev_t *pDev, uint16_t color);
static void __lcd_fill_area(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
static void __lcd_show_char(LCDDev_t *pDev, uint16_t x, uint16_t y, uint8_t character, uint16_t fc, uint16_t bc, uint8_t fontSize);
static void __lcd_show_string(LCDDev_t *pDev, uint16_t x, uint16_t y, char *string,uint16_t fc, uint16_t bc, uint8_t fontSize);
static void __lcd_show_num(LCDDev_t *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc , uint8_t fontSize);
static void __lcd_show_float_num(LCDDev_t *pDev, uint16_t x, uint16_t y, float num, uint8_t intLen, uint8_t fraLen, uint16_t fc, uint16_t bc, uint8_t size);
static void __lcd_show_chinese(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t fontSize);
static void __lcd_show_image(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
static void __lcd_draw_point(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t color);
static void __lcd_draw_line(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
static void __lcd_draw_rectangle(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t isFilled);
static void __lcd_draw_circle(LCDDev_t *pDev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color);
static int __lcd_deinit(LCDDev_t *pDev);

/******************************************************************************
 * @brief	初始化LCD
 * @param	pDev	:	LCDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int lcd_init(LCDDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 保存私有数据 */
	pDev->pPrivData = (LCDPrivData_t *)malloc(sizeof(LCDPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	LCDPrivData_t *pPrivData = (LCDPrivData_t *)pDev->pPrivData;
	
	pPrivData->lcdSPI.info.spix = pDev->info.spix;
	pPrivData->lcdSPI.info.SCKPort = pDev->info.SCKPort;
	pPrivData->lcdSPI.info.SCKPin = pDev->info.SCKPin;
	pPrivData->lcdSPI.info.MOSIPort = pDev->info.MOSIPort;
	pPrivData->lcdSPI.info.MOSIPin = pDev->info.MOSIPin;
	pPrivData->lcdSPI.info.MISOPort = NULL;
	pPrivData->lcdSPI.info.MISOPin = NULL;
	pPrivData->lcdSPI.info.CSPort = pDev->info.CSPort;
	pPrivData->lcdSPI.info.CSPin = pDev->info.CSPin;
	pPrivData->lcdSPI.info.prescaler = pDev->info.prescaler;
	pPrivData->lcdSPI.info.mode = pDev->info.mode;
	
	/* 配置硬件SPI */
	spi_init(&pPrivData->lcdSPI);
	
	/* 配置时钟与GPIO */
	__lcd_config_gpio_clock_enable(pDev->info.RESPort);
	__lcd_config_gpio_clock_enable(pDev->info.DCPort);
	__lcd_config_gpio_clock_enable(pDev->info.BLPort);
	
	__lcd_config_io_out_pp(pDev->info.RESPort, pDev->info.RESPin);
	__lcd_config_io_out_pp(pDev->info.DCPort, pDev->info.DCPin);
	__lcd_config_io_out_pp(pDev->info.BLPort, pDev->info.BLPin);
	
	/* 配置DMA */
	if(pDev->info.useDMA == LCD_USE_DMA)
	{
		__lcd_dma_init(pDev);
	}
	
	/* 函数指针赋值 */
	pDev->update = __lcd_update;
	pDev->fill = __lcd_fill;
	pDev->fill_area = __lcd_fill_area;
	pDev->show_char = __lcd_show_char;
	pDev->show_string = __lcd_show_string;
	pDev->show_num = __lcd_show_num;
	pDev->show_float_num = __lcd_show_float_num;
	pDev->show_chinese = __lcd_show_chinese;
	pDev->show_image = __lcd_show_image;
	pDev->draw_point = __lcd_draw_point;
	pDev->draw_line = __lcd_draw_line;
	pDev->draw_rectangle = __lcd_draw_rectangle;
	pDev->draw_circle = __lcd_draw_circle;
	pDev->deinit = __lcd_deinit;
	
	/* 配置LCD */
	__lcd_res_write(pDev, 0);	// 复位
	__lcd_delay_ms(100);
	__lcd_res_write(pDev, 1);
	__lcd_delay_ms(100);
	__lcd_bl_write(pDev, 1);	// 打开背光
	__lcd_delay_ms(100);
	/* 启动初始序列 */
	__lcd_write_command(pDev, 0x11); // Sleep out 
	__lcd_delay_ms(120);
	/* ST7735S Frame Rate */
	__lcd_write_command(pDev, 0xB1);
	__lcd_write_byte(pDev, 0x05);
	__lcd_write_byte(pDev, 0x3C);
	__lcd_write_byte(pDev, 0x3C);
	__lcd_write_command(pDev, 0xB2);
	__lcd_write_byte(pDev, 0x05);
	__lcd_write_byte(pDev, 0x3C);
	__lcd_write_byte(pDev, 0x3C);
	__lcd_write_command(pDev, 0xB3);
	__lcd_write_byte(pDev, 0x05);
	__lcd_write_byte(pDev, 0x3C); 
	__lcd_write_byte(pDev, 0x3C);
	__lcd_write_byte(pDev, 0x05);
	__lcd_write_byte(pDev, 0x3C);
	__lcd_write_byte(pDev, 0x3C);
	/* Dot inversion */
	__lcd_write_command(pDev, 0xB4);
	__lcd_write_byte(pDev, 0x03);
	/* ST7735S Power Sequence */
	__lcd_write_command(pDev, 0xC0);
	__lcd_write_byte(pDev, 0x28);
	__lcd_write_byte(pDev, 0x08);
	__lcd_write_byte(pDev, 0x04);
	__lcd_write_command(pDev, 0xC1);
	__lcd_write_byte(pDev, 0XC0);
	__lcd_write_command(pDev, 0xC2);
	__lcd_write_byte(pDev, 0x0D);
	__lcd_write_byte(pDev, 0x00);
	__lcd_write_command(pDev, 0xC3);
	__lcd_write_byte(pDev, 0x8D);
	__lcd_write_byte(pDev, 0x2A);
	__lcd_write_command(pDev, 0xC4);
	__lcd_write_byte(pDev, 0x8D);
	__lcd_write_byte(pDev, 0xEE);
	/* VCOM */
	__lcd_write_command(pDev, 0xC5);
	__lcd_write_byte(pDev, 0x1A);
	/* MX, MY, RGB mode */
	__lcd_write_command(pDev, 0x36);
	if(LCD_DIRECTION==0)			// 竖屏正向
		__lcd_write_byte(pDev, 0xC0);
	else if(LCD_DIRECTION==1)		// 竖屏反向
		__lcd_write_byte(pDev, 0x00);
	else if(LCD_DIRECTION==2)		// 横屏正向
		__lcd_write_byte(pDev, 0x70);
	else							// 横屏反向
		__lcd_write_byte(pDev, 0xA0);
	/* ST7735S Gamma Sequence */ 
	__lcd_write_command(pDev, 0xE0);
	__lcd_write_byte(pDev, 0x04);
	__lcd_write_byte(pDev, 0x22);
	__lcd_write_byte(pDev, 0x07);
	__lcd_write_byte(pDev, 0x0A);
	__lcd_write_byte(pDev, 0x2E);
	__lcd_write_byte(pDev, 0x30);
	__lcd_write_byte(pDev, 0x25);
	__lcd_write_byte(pDev, 0x2A);
	__lcd_write_byte(pDev, 0x28);
	__lcd_write_byte(pDev, 0x26);
	__lcd_write_byte(pDev, 0x2E);
	__lcd_write_byte(pDev, 0x3A);
	__lcd_write_byte(pDev, 0x00);
	__lcd_write_byte(pDev, 0x01);
	__lcd_write_byte(pDev, 0x03);
	__lcd_write_byte(pDev, 0x13);
	__lcd_write_command(pDev, 0xE1);
	__lcd_write_byte(pDev, 0x04);
	__lcd_write_byte(pDev, 0x16);
	__lcd_write_byte(pDev, 0x06);
	__lcd_write_byte(pDev, 0x0D);
	__lcd_write_byte(pDev, 0x2D);
	__lcd_write_byte(pDev, 0x26);
	__lcd_write_byte(pDev, 0x23);
	__lcd_write_byte(pDev, 0x27);
	__lcd_write_byte(pDev, 0x27);
	__lcd_write_byte(pDev, 0x25);
	__lcd_write_byte(pDev, 0x2D);
	__lcd_write_byte(pDev, 0x3B);
	__lcd_write_byte(pDev, 0x00);
	__lcd_write_byte(pDev, 0x01);
	__lcd_write_byte(pDev, 0x04);
	__lcd_write_byte(pDev, 0x13);
	/* 65k mode */
	__lcd_write_command(pDev, 0x3A);
	__lcd_write_byte(pDev, 0x05);
	/* Display on */
	__lcd_write_command(pDev, 0x29);
	
	/* 屏幕初始化为黑色 */
	pDev->fill(pDev, BLACK);
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	LCD配置DMA传输
 * @param	pDev	:	LCDDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_dma_init(LCDDev_t *pDev)
{
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	__lcd_config_dma_clock_enable(pDev->info.spix);	// 开启DMA时钟
	DMA_DeInit(__lcd_get_dma_channel(pDev->info.spix));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&pDev->info.spix->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)gLCDDisplayBuf;			// 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;							// 方向：从内存到外设
    DMA_InitStructure.DMA_BufferSize = 0;										// 传输大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 外设地址不增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						// 内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		// 外设数据单位8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;				// 内存数据单位8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;								// 工作在正常模式，一次传输后自动结束
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;						// 优先级：中
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								// 没有设置为内存到内存传输
    DMA_Init(__lcd_get_dma_channel(pDev->info.spix), &DMA_InitStructure);
	
    DMA_ClearFlag(__lcd_get_dma_flag(pDev->info.spix));
	
    DMA_Cmd(__lcd_get_dma_channel(pDev->info.spix), DISABLE);
	
	#elif defined(STM32F40_41xxx)
	
    __lcd_config_dma_clock_enable(pDev->info.spix);	// 开启DMA时钟
	
	DMA_DeInit(__lcd_get_dma_stream(pDev->info.spix));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_Channel = __lcd_get_dma_channel(pDev->info.spix);		// 选择DMA通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&pDev->info.spix->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)gLCDDisplayBuf;			// 内存地址
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
    DMA_Init(__lcd_get_dma_stream(pDev->info.spix), &DMA_InitStructure);
	
    DMA_ClearFlag(	__lcd_get_dma_stream(pDev->info.spix), 
					__lcd_get_dma_flag(pDev->info.spix)	);
					
    DMA_Cmd(__lcd_get_dma_stream(pDev->info.spix), DISABLE);
	
	#endif
}

/*通信协议*********************************************************************/

/******************************************************************************
 * @brief	LCD写RES高低电平
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	bitValue	:	要写入RES的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_res_write(LCDDev_t *pDev, uint8_t bitValue)
{
	__lcd_io_write(pDev->info.RESPort, pDev->info.RESPin, bitValue);
}

/******************************************************************************
 * @brief	LCD写DC高低电平
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	bitValue	:	要写入DC的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_dc_write(LCDDev_t *pDev, uint8_t bitValue)
{
	__lcd_io_write(pDev->info.DCPort, pDev->info.DCPin, bitValue);
}

/******************************************************************************
 * @brief	LCD写BL高低电平
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	bitValue	:	要写入DC的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __lcd_bl_write(LCDDev_t *pDev, uint8_t bitValue)
{
	__lcd_io_write(pDev->info.BLPort, pDev->info.BLPin, bitValue);
}

/******************************************************************************
 * @brief	LCD写命令
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	byte	:	要写入的命令值，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_command(LCDDev_t *pDev, uint8_t command)
{
	__lcd_dc_write(pDev, 0);			// 写命令
	__lcd_write_byte(pDev, command);
	__lcd_dc_write(pDev, 1);			// 写数据
}

/******************************************************************************
 * @brief	LCD写一个字节数据
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	byte	:	要写入的数据，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_byte(LCDDev_t *pDev, uint8_t byte)
{	
	LCDPrivData_t *pPrivData = (LCDPrivData_t *)pDev->pPrivData;
	
	pPrivData->lcdSPI.start(&pPrivData->lcdSPI);					// 拉低CS，开始通信
	pPrivData->lcdSPI.swap_byte(&pPrivData->lcdSPI, byte);		// 写入指定命令
	pPrivData->lcdSPI.stop(&pPrivData->lcdSPI);					// 拉高CS，结束通信
}

/******************************************************************************
 * @brief	LCD写一个半字数据
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	halfword	:	要写入的数据，范围：0x0000~0xFFFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_halfword(LCDDev_t *pDev, uint16_t halfword)
{
	LCDPrivData_t *pPrivData = (LCDPrivData_t *)pDev->pPrivData;
	
	pPrivData->lcdSPI.start(&pPrivData->lcdSPI);						// 拉低CS，开始通信
	pPrivData->lcdSPI.swap_byte(&pPrivData->lcdSPI, halfword >> 8);	// 写入数据
	pPrivData->lcdSPI.swap_byte(&pPrivData->lcdSPI, halfword);		// 写入数据
	pPrivData->lcdSPI.stop(&pPrivData->lcdSPI);						// 拉高CS，结束通信
}

/**********************************************************************通信协议*/

/*硬件配置**********************************************************************/

/******************************************************************************
 * @brief	LCD设置起始和结束地址
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x1,x2	:	设置列的起始和结束地址
 * @param	y1,y2	:	设置行的起始和结束地址
 * @return	无
 ******************************************************************************/
static void __lcd_set_address(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if(LCD_DIRECTION == 0 || LCD_DIRECTION == 1)			// 竖屏
	{
		__lcd_write_command(pDev, 0x2a);					// 列地址设置
		__lcd_write_halfword(pDev, x1 + 2);
		__lcd_write_halfword(pDev, x2 + 2);
		__lcd_write_command(pDev, 0x2b);					// 行地址设置
		__lcd_write_halfword(pDev, y1 + 1);
		__lcd_write_halfword(pDev, y2 + 1);
		__lcd_write_command(pDev, 0x2c);					// 储存器写
	}
	else if(LCD_DIRECTION == 2 || LCD_DIRECTION == 3)		// 横屏
	{
		__lcd_write_command(pDev, 0x2a);					// 列地址设置
		__lcd_write_halfword(pDev, x1 + 1);
		__lcd_write_halfword(pDev, x2 + 1);
		__lcd_write_command(pDev, 0x2b);					// 行地址设置
		__lcd_write_halfword(pDev, y1 + 2);
		__lcd_write_halfword(pDev, y2 + 2);
		__lcd_write_command(pDev, 0x2c);					// 储存器写
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
 * @param	pDev	:	LCDDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_update(LCDDev_t *pDev)
{
	if(pDev->info.useDMA == LCD_USE_DMA)
	{
		LCDPrivData_t *pPrivData = (LCDPrivData_t *)pDev->pPrivData;
		
		/* 将整个数据搬运一次到DMA */
		__lcd_set_address(pDev, 0, 0, LCD_W - 1, LCD_H - 1);
		
		/* 重新设置缓存大小，开启一次DMA传输 */
		#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
		
		DMA_Cmd(__lcd_get_dma_channel(pDev->info.spix), DISABLE);							// 关闭DMA
		DMA_SetCurrDataCounter(__lcd_get_dma_channel(pDev->info.spix), LCD_W * LCD_H * 2);	// 数据传输量
		DMA_ClearFlag(__lcd_get_dma_flag(pDev->info.spix));
		
		pPrivData->lcdSPI.cs_write(&pPrivData->lcdSPI, 0);
		__lcd_res_write(pDev, 1);
		
		SPI_I2S_DMACmd(pDev->info.spix, SPI_I2S_DMAReq_Tx, ENABLE);							// 开启SPI的DMA接收
		DMA_Cmd(__lcd_get_dma_channel(pDev->info.spix), ENABLE);							// 使能DMA
		
		while(!DMA_GetFlagStatus(__lcd_get_dma_flag(pDev->info.spix)));						// 等待传输完成
		DMA_ClearFlag(__lcd_get_dma_flag(pDev->info.spix));
		
		#elif defined(STM32F40_41xxx)
		
		DMA_Cmd(__lcd_get_dma_stream(pDev->info.spix), DISABLE);							// 关闭DMA
		DMA_SetCurrDataCounter(__lcd_get_dma_stream(pDev->info.spix), LCD_W * LCD_H * 2);	// 数据传输量
		
		DMA_ClearFlag(	__lcd_get_dma_stream(pDev->info.spix), 
						__lcd_get_dma_flag(pDev->info.spix)	);
		
		pPrivData->lcdSPI.cs_write(&pPrivData->lcdSPI, 0);
		__lcd_res_write(pDev, 1);
		
		SPI_I2S_DMACmd(pDev->info.spix, SPI_I2S_DMAReq_Tx, ENABLE);							// 开启SPI的DMA接收
		DMA_Cmd(__lcd_get_dma_stream(pDev->info.spix), ENABLE);								// 使能DMA
		
		while(!DMA_GetFlagStatus(	__lcd_get_dma_stream(pDev->info.spix), 					// 等待传输完成
									__lcd_get_dma_flag(pDev->info.spix)		));
		
		DMA_ClearFlag(	__lcd_get_dma_stream(pDev->info.spix),
						__lcd_get_dma_flag(pDev->info.spix)	);
		
		#endif
		
		pPrivData->lcdSPI.cs_write(&pPrivData->lcdSPI, 1);
	}
}

/******************************************************************************
 * @brief	LCD填充颜色
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	color	:	要填充的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_fill(LCDDev_t *pDev, uint16_t color)
{          
	uint16_t i, j;
	
	if(pDev->info.useDMA == LCD_USE_DMA)
	{
		for(i = 0 ; i < LCD_H; i++)
		{
			for(j = 0;j < LCD_W * 2; j += 2)
			{
				gLCDDisplayBuf[i][j] = color >> 8;
				gLCDDisplayBuf[i][j + 1] = color;
			}
		}
	}
	else if(pDev->info.useDMA == LCD_NONUSE_DMA)
	{
		__lcd_set_address(pDev, 0, 0, LCD_W - 1, LCD_H - 1);// 设置显示范围
		for(i = 0; i < LCD_H; i++)
		{													   	 	
			for(j = 0; j < LCD_W; j++)
			{
				__lcd_write_halfword(pDev, color);
			}
		}
	}
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x,y		:	填充起点
 * @param	width	:	填充的宽度
 * @param	height	:	填充的高度
 * @param	color	:	要填充的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_fill_area(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{          
	uint16_t i, j;
	
	if(pDev->info.useDMA == LCD_USE_DMA)
	{
		for(i = y ; i < y + height; i++)
		{
			for(j = x * 2; j < (x + width) * 2; j += 2)
			{
				gLCDDisplayBuf[i][j] = color >> 8;
				gLCDDisplayBuf[i][j + 1] = color;
			}
		}
	}
	else if(pDev->info.useDMA == LCD_NONUSE_DMA)
	{
		__lcd_set_address(pDev, x, y, x + width - 1, y + height - 1);// 设置显示范围
		for(i = y; i < y + height; i++)
		{													   	 	
			for(j = x; j < x + width; j++)
			{
				__lcd_write_halfword(pDev, color);
			}
		}
	}
}

/******************************************************************************
 * @brief	LCD显示单个字符
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	character	:	要显示的字符
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_16X32	宽16像素，高32像素
							LCD_12X24	宽12像素，高24像素
							LCD_8X16	宽8像素，高16像素
							LCD_6X12	宽6像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_char(LCDDev_t *pDev, uint16_t x, uint16_t y, uint8_t character, uint16_t fc, uint16_t bc, uint8_t fontSize)
{
	uint8_t temp, fontSizex, t, m = 0;
	uint16_t i, TypefaceNum;										//一个字符所占字节大小
	uint16_t x0 = x;
	fontSizex = fontSize / 2;
	TypefaceNum = (fontSizex / 8 + ((fontSizex % 8) ? 1 : 0)) * fontSize;
	character = character - ' ';    								//得到偏移后的值
	__lcd_set_address(pDev, x, y, x+fontSizex - 1, y+fontSize - 1);	//设置光标位置 
	for(i = 0; i < TypefaceNum; i++)
	{ 
		if(fontSize == 12)		temp=LCD_F6x12[character][i];		//调用6x12字体
		else if(fontSize == 16)	temp=LCD_F8x16[character][i];		//调用8x16字体
		else if(fontSize == 24)	temp=LCD_F12x24[character][i];		//调用12x24字体
		else if(fontSize == 32)	temp=LCD_F16x32[character][i];		//调用16x32字体
		else return;
		for(t = 0; t < 8; t++)
		{
			if(pDev->info.useDMA == LCD_USE_DMA)
			{
				if(temp & (0x01 << t))__lcd_draw_point(pDev, x, y, fc);//画一个点
				else __lcd_draw_point(pDev, x, y, bc);
				x++;
				if((x - x0) == fontSizex)
				{
					x = x0;
					y++;
					break;
				}
			}
			else if(pDev->info.useDMA == LCD_NONUSE_DMA)
			{
				if(temp & (0x01 << t))__lcd_write_halfword(pDev, fc);
				else __lcd_write_halfword(pDev, bc);
				m++;
				if(m % fontSizex == 0)
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
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	string		:	要显示的字符串
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_16X32	宽16像素，高32像素
							LCD_12X24	宽12像素，高24像素
							LCD_8X16	宽8像素，高16像素
							LCD_6X12	宽6像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_string(LCDDev_t *pDev, uint16_t x, uint16_t y, char *string, uint16_t fc, uint16_t bc, uint8_t fontSize)
{         
	while(*string!='\0')
	{       
		__lcd_show_char(pDev, x, y, *string, fc, bc, fontSize);
		x += fontSize / 2;
		string++;
	}
}

/******************************************************************************
 * @brief	LCD显示整数变量
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	num			:	要显示整数变量
 * @param	len			:	要显示的位数
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_16X32	宽16像素，高32像素
							LCD_12X24	宽12像素，高24像素
							LCD_8X16	宽8像素，高16像素
							LCD_6X12	宽6像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_num(LCDDev_t *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t fontSize)
{         	
	uint8_t t, temp;
	uint8_t enshow = 0;
	uint8_t fontSizex = fontSize / 2;
	for(t = 0; t < len; t++)
	{
		temp = (num / __lcd_pow(10, len - t - 1)) % 10;
		if(enshow == 0 && t < (len - 1))
		{
			if(temp == 0)
			{
				__lcd_show_char(pDev, x + t * fontSizex, y, ' ', fc, bc, fontSize);
				continue;
			}else enshow = 1; 
		 	 
		}
	 	__lcd_show_char(pDev, x + t * fontSizex, y, temp + 48, fc, bc, fontSize);
	}
}

/******************************************************************************
 * @brief	LCD显示两位小数变量
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	num		:	要显示整数变量
 * @param	intLen	:	要显示的整数位数
 * @param	fraLen	:	要显示的小数位数
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_float_num(LCDDev_t *pDev, uint16_t x, uint16_t y, float num, uint8_t intLen, uint8_t fraLen, uint16_t fc, uint16_t bc, uint8_t size)
{
    uint32_t powNum, intNum, fraNum;
	
	if (num < 0)
	{
		__lcd_show_char(pDev, x, y, '-', fc, bc, size);	// 显示-号
	}
	
	/* 提取整数部分和小数部分 */
	intNum = num;						// 直接赋值给整型变量，提取整数
	num -= intNum;						// 将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	powNum = __lcd_pow(10, fraLen);		// 根据指定小数的位数，确定乘数
	fraNum = round(num * powNum);		// 将小数乘到整数，同时四舍五入，避免显示误差
	intNum += fraNum / powNum;			// 若四舍五入造成了进位，则需要再加给整数
	
	if (num >= 0)
	{
		__lcd_show_num(pDev, x, y, intNum, intLen, fc, bc, size);								// 显示整数部分
		__lcd_show_char(pDev, x + (intLen) * size / 2, y, '.', fc, bc, size);					// 显示小数点
		__lcd_show_num(pDev, x + (intLen + 1) * size / 2, y, fraNum, fraLen, fc, bc, size);		// 显示小数部分
	}
	else
	{
		num = -num;
		__lcd_show_num(pDev, x + size / 2, y, intNum, intLen, fc, bc, size);					// 显示整数部分
		__lcd_show_char(pDev, x + (intLen + 1) * size / 2, y, '.', fc, bc, size);				// 显示小数点
		__lcd_show_num(pDev, x + (intLen + 2) * size / 2, y, fraNum, fraLen, fc, bc, size);		// 显示小数部分
	}
}

/******************************************************************************
 * @brief	LCD显示单个12x12汉字
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	Chinese		:	要显示的汉字
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_32X32	宽32像素，高32像素
							LCD_24X24	宽24像素，高24像素
							LCD_16X16	宽16像素，高16像素
							LCD_12X12	宽12像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese12x12(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc)
{
	uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(12); 					//汉字数目
	uint16_t TypefaceNum = (12 / 8 + ((12 % 8) ? 1 : 0)) * 12;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF12x12[k].Index[0] == *(Chinese)) && (LCD_CF12x12[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_address(pDev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(pDev->info.useDMA == LCD_USE_DMA)
					{
						if (LCD_CF12x12[k].Msk[i] & (0x01 << j))
							__lcd_draw_point(pDev, x, y, fc); //画一个点
						else
							__lcd_draw_point(pDev, x, y, bc);
						x++;
						if ((x - x0) == 12)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(pDev->info.useDMA == LCD_NONUSE_DMA)
					{
						if (LCD_CF12x12[k].Msk[i] & (0x01 << j))
							__lcd_write_halfword(pDev, fc);
						else
							__lcd_write_halfword(pDev, bc);
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
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	Chinese		:	要显示的汉字
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_32X32	宽32像素，高32像素
							LCD_24X24	宽24像素，高24像素
							LCD_16X16	宽16像素，高16像素
							LCD_12X12	宽12像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese16x16(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc)
{
	uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(16); 					//汉字数目
	uint16_t TypefaceNum = (16 / 8 + ((16 % 8) ? 1 : 0)) * 16;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF16x16[k].Index[0] == *(Chinese)) && (LCD_CF16x16[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_address(pDev, x, y, x + 16 - 1, y + 16 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(pDev->info.useDMA == LCD_USE_DMA)
					{
						if (LCD_CF16x16[k].Msk[i] & (0x01 << j))
							__lcd_draw_point(pDev, x, y, fc); //画一个点
						else
							__lcd_draw_point(pDev, x, y, bc);
						x++;
						if ((x - x0) == 16)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(pDev->info.useDMA == LCD_NONUSE_DMA)
					{
						if (LCD_CF16x16[k].Msk[i] & (0x01 << j))
							__lcd_write_halfword(pDev, fc);
						else
							__lcd_write_halfword(pDev, bc);
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
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	Chinese		:	要显示的汉字
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_32X32	宽32像素，高32像素
							LCD_24X24	宽24像素，高24像素
							LCD_16X16	宽16像素，高16像素
							LCD_12X12	宽12像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese24x24(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc)
{
	uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(24); 					//汉字数目
	uint16_t TypefaceNum = (24 / 8 + ((24 % 8) ? 1 : 0)) * 24;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF24x24[k].Index[0] == *(Chinese)) && (LCD_CF24x24[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_address(pDev, x, y, x + 24 - 1, y + 24 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(pDev->info.useDMA == LCD_USE_DMA)
					{
						if (LCD_CF24x24[k].Msk[i] & (0x01 << j))
							__lcd_draw_point(pDev, x, y, fc); //画一个点
						else
							__lcd_draw_point(pDev, x, y, bc);
						x++;
						if ((x - x0) == 24)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(pDev->info.useDMA == LCD_NONUSE_DMA)
					{
						if (LCD_CF24x24[k].Msk[i] & (0x01 << j))
							__lcd_write_halfword(pDev, fc);
						else
							__lcd_write_halfword(pDev, bc);
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
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	Chinese		:	要显示的汉字
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_32X32	宽32像素，高32像素
							LCD_24X24	宽24像素，高24像素
							LCD_16X16	宽16像素，高16像素
							LCD_12X12	宽12像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese32x32(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc)
{
		uint8_t i, j, m = 0;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(32); 					//汉字数目
	uint16_t TypefaceNum = (32 / 8 + ((32 % 8) ? 1 : 0)) * 32;	//一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF32x32[k].Index[0] == *(Chinese)) && (LCD_CF32x32[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_address(pDev, x, y, x + 32 - 1, y + 32 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if(pDev->info.useDMA == LCD_USE_DMA)
					{
						if (LCD_CF32x32[k].Msk[i] & (0x01 << j))
							__lcd_draw_point(pDev, x, y, fc); //画一个点
						else
							__lcd_draw_point(pDev, x, y, bc);
						x++;
						if ((x - x0) == 32)
						{
							x = x0;
							y++;
							break;
						}
					}
					else if(pDev->info.useDMA == LCD_NONUSE_DMA)
					{
						if (LCD_CF32x32[k].Msk[i] & (0x01 << j))
							__lcd_write_halfword(pDev, fc);
						else
							__lcd_write_halfword(pDev, bc);
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
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	显示坐标
 * @param	Chinese		:	要显示的汉字串
 * @param	fc			:	字的颜色
 * @param	bc			:	字的背景色
 * @param	fontSize	:	指定字体大小
			范围		:	LCD_32X32	宽32像素，高32像素
							LCD_24X24	宽24像素，高24像素
							LCD_16X16	宽16像素，高16像素
							LCD_12X12	宽12像素，高12像素
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t fontSize)
{
	while(*Chinese != 0)
	{
		if(fontSize==LCD_12X12)			__lcd_show_chinese12x12(pDev, x, y, Chinese, fc, bc);
		else if(fontSize==LCD_16X16)	__lcd_show_chinese16x16(pDev, x, y, Chinese, fc, bc);
		else if(fontSize==LCD_24X24)	__lcd_show_chinese24x24(pDev, x, y, Chinese, fc, bc);
		else if(fontSize==LCD_32X32)	__lcd_show_chinese32x32(pDev, x, y, Chinese, fc, bc);
		else return;
		
		Chinese += LCD_CHN_CHAR_WIDTH;
		x += fontSize;
	}
}

/******************************************************************************
 * @brief	LCD显示图片
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x,y			:	起点坐标
 * @param	width		:	图片宽度
 * @param	height		:	图片高度
 * @param	pic[]		:	图片数组
 * @return	无
 ******************************************************************************/
static void __lcd_show_image(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{
	uint16_t i, j; 
	uint32_t k = 0;
	
	if(pDev->info.useDMA == LCD_USE_DMA)
	{
		for(i = y; i < y + height; i++)
		{
			for(j = x; j < x + width; j++)
			{
				gLCDDisplayBuf[i][j * 2] = pic[k * 2];
                gLCDDisplayBuf[i][j * 2 + 1] = pic[k * 2 + 1];
				k++;
			}
		}
	}
	else if(pDev->info.useDMA == LCD_NONUSE_DMA)
	{
		__lcd_set_address(pDev, x, y, x + width - 1, y + height - 1);	//设置显示范围
		for(i = 0; i < height; i++)
		{													   	 	
			for(j = 0; j < width; j++)
			{
				__lcd_write_byte(pDev, pic[k * 2]);
				__lcd_write_byte(pDev, pic[k * 2 + 1]);
				k++;
			}
		}
	}
}

/******************************************************************************
 * @brief	LCD在指定位置画点
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x,y		:	画点坐标
 * @param	color	:	点的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_point(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t color)
{
	if(pDev->info.useDMA == LCD_USE_DMA)
	{
		/*使用DMA的话，从对点刷屏到对显存数组写入数据，DMA传输数据的时候再统一进行传输*/
		gLCDDisplayBuf[y][x*2] = color >> 8;
		gLCDDisplayBuf[y][x*2+1] = color;
	}
	else if(pDev->info.useDMA == LCD_NONUSE_DMA)
	{
		__lcd_set_address(pDev, x, y, x, y);		//设置光标位置 
		__lcd_write_halfword(pDev, color);
	}
}

/******************************************************************************
 * @brief	LCD画线
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x1,y1	:	起始坐标
 * @param	x2,y2	:	终止坐标
 * @param	color	:	线的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_line(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
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
		__lcd_draw_point(pDev, uRow, uCol, color);
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
 * @param	pDev		:	LCDDev_t结构体指针
 * @param	x			:	指定矩形左上角的横坐标
 * @param	y			:	指定矩形左上角的纵坐标
 * @param	width		:	指定矩形的宽度
 * @param	height		:	指定矩形的高度
 * @param	color		:	矩形的颜色
 * @param	isFilled	:	指定是否填充
 * @return	无
 ******************************************************************************/
static void __lcd_draw_rectangle(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t isFilled)
{
	uint8_t i, j;
	if (!isFilled)		//指定矩形不填充
	{
		/*遍历上下x坐标，画矩形上下两条线*/
		for (i = x; i < x + width; i ++)
		{
			__lcd_draw_point(pDev, i, y, color);
			__lcd_draw_point(pDev, i, y + height - 1, color);
		}
		/*遍历左右y坐标，画矩形左右两条线*/
		for (i = y; i < y + height; i ++)
		{
			__lcd_draw_point(pDev, x, i, color);
			__lcd_draw_point(pDev, x + width - 1, i, color);
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
				__lcd_draw_point(pDev, i, j, color);
			}
		}
	}
}

/******************************************************************************
 * @brief	LCD画圆
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x,y	:	圆心坐标
 * @param	radius	:	半径
 * @param	color	:	圆的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_circle(LCDDev_t *pDev, uint16_t x, uint16_t y, uint8_t radius, uint16_t color)
{
	int a, b;
	a = 0;
	b = radius;
	
	while(a <= b)
	{
		__lcd_draw_point(pDev, x - b, y - a, color);	//3           
		__lcd_draw_point(pDev, x + b, y - a, color);	//0           
		__lcd_draw_point(pDev, x - a, y + b, color);	//1                
		__lcd_draw_point(pDev, x - a, y - b, color);	//2             
		__lcd_draw_point(pDev, x + b, y + a, color);	//4               
		__lcd_draw_point(pDev, x + a, y - b, color);	//5
		__lcd_draw_point(pDev, x + a, y + b, color);	//6 
		__lcd_draw_point(pDev, x - b, y + a, color);	//7
		a++;
		if((a * a + b * b) > (radius * radius))		//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
 * @brief	去初始化LCD
 * @param	pDev   :  OLEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __lcd_deinit(LCDDev_t *pDev)
{
	if (!pDev || !pDev->initFlag)
	return -1;
	
	LCDPrivData_t *pPrivData = (LCDPrivData_t *)pDev->pPrivData;
	
	/*去初始化硬件SPI*/
	pPrivData->lcdSPI.deinit(&pPrivData->lcdSPI);
	
	/*释放私有数据内存*/
	free(pDev->pPrivData);
	pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	//修改初始化标志
	return 0;
}

/**********************************************************************功能函数*/
