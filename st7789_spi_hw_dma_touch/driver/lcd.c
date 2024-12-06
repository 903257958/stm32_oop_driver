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
		//vTaskDelay(ms);
		timerDelay.delay_ms(&timerDelay, ms);
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
		//vTaskDelay(ms);
		timerDelay.delay_ms(&timerDelay, ms);
	}								  
	#endif

#endif
											
/* LCD私有数据结构体 */
typedef struct {
	SPIDev_t lcdSPI;	// 硬件SPI设备
	PWMDev_t lcdPWM;	// PWM背光引脚
}LCDPrivData_t;

/* 通信协议 */
static void __lcd_res_write(LCDDev_t *pDev, uint8_t bitValue);
static void __lcd_dc_write(LCDDev_t *pDev, uint8_t bitValue);
static void __lcd_reset(LCDDev_t *pDev);
static void __lcd_write_command(LCDDev_t *pDev, uint8_t command);
static void __lcd_write_byte(LCDDev_t *pDev, uint8_t data);
static void __lcd_write_halfword(LCDDev_t *pDev, uint16_t halfword);

/* 硬件配置 */
static void __lcd_set_direction(LCDDev_t *pDev);
static void __lcd_set_windows(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/* 功能函数 */
static void __lcd_clear(LCDDev_t *pDev, uint16_t color);
static void __lcd_fill(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
static void __lcd_color_fill(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *pColor);
static void __lcd_color_fill_dma(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size);
static void __lcd_show_char(LCDDev_t *pDev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_string(LCDDev_t *pDev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_num(LCDDev_t *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_hex_num(LCDDev_t *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_float_num(LCDDev_t *pDev, uint16_t x, uint16_t y, float num, uint8_t intLen, uint8_t fraLen, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_chinese(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_image(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
static void __lcd_draw_point(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t color);
static void __lcd_draw_line(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
static void __lcd_draw_rectangle(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
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
	pPrivData->lcdSPI.info.CSPort = pDev->info.CSPort;
	pPrivData->lcdSPI.info.CSPin = pDev->info.CSPin;
	pPrivData->lcdSPI.info.prescaler = 2;
	pPrivData->lcdSPI.info.mode = SPI_MODE_3;

	pPrivData->lcdPWM.info.timx = pDev->info.timx;
	pPrivData->lcdPWM.info.OCChannel = pDev->info.OCChannel;
	pPrivData->lcdPWM.info.psc = 82;
	pPrivData->lcdPWM.info.arr = 3000;
	pPrivData->lcdPWM.info.port = pDev->info.BLPort;
	pPrivData->lcdPWM.info.pin = pDev->info.BLPin;
	
	/* 配置硬件SPI */
	spi_init(&pPrivData->lcdSPI);
	
	/* 配置时钟与GPIO */
	__lcd_config_gpio_clock_enable(pDev->info.RESPort);
	__lcd_config_gpio_clock_enable(pDev->info.DCPort);
	
	__lcd_config_io_out_pp(pDev->info.RESPort, pDev->info.RESPin);
	__lcd_config_io_out_pp(pDev->info.DCPort, pDev->info.DCPin);
	
	__lcd_dc_write(pDev, 1);
	__lcd_res_write(pDev, 1);
	
	/* 配置背光 */
	pwm_init(&pPrivData->lcdPWM);
	pPrivData->lcdPWM.set_compare(&pPrivData->lcdPWM, 100);
	
	/* 配置LCD */
	__lcd_reset(pDev);			// 复位

	__lcd_set_direction(pDev);	// 设置屏幕方向

	__lcd_write_command(pDev, 0x36);
	if (pDev->info.dir == VERTICAL_FORWARD)
		__lcd_write_byte(pDev, 0x00);
    else if (pDev->info.dir == VERTICAL_REVERSE)
		__lcd_write_byte(pDev, 0xC0);
	else if (pDev->info.dir == HORIZONTAL_FORWARD)
		__lcd_write_byte(pDev, 0x70);
	else if (pDev->info.dir == HORIZONTAL_REVERSE)
		__lcd_write_byte(pDev, 0xA0);

    __lcd_write_command(pDev, 0x3A);
    __lcd_write_byte(pDev, 0x05);

    __lcd_write_command(pDev, 0xB2);
    __lcd_write_byte(pDev, 0x0B);
    __lcd_write_byte(pDev, 0x0B);
    __lcd_write_byte(pDev, 0x00);
    __lcd_write_byte(pDev, 0x33);
    __lcd_write_byte(pDev, 0x35);

    __lcd_write_command(pDev, 0xB7);
    __lcd_write_byte(pDev, 0x11);

    __lcd_write_command(pDev, 0xBB);
    __lcd_write_byte(pDev, 0x35);

    __lcd_write_command(pDev, 0xC0);
    __lcd_write_byte(pDev, 0x2C);

    __lcd_write_command(pDev, 0xC2);
    __lcd_write_byte(pDev, 0x01);

    __lcd_write_command(pDev, 0xC3);
    __lcd_write_byte(pDev, 0x0D);

    __lcd_write_command(pDev, 0xC4);
    __lcd_write_byte(pDev, 0x20);

    __lcd_write_command(pDev, 0xC6);
    __lcd_write_byte(pDev, 0x13);

    __lcd_write_command(pDev, 0xD0);
    __lcd_write_byte(pDev, 0xA4);
    __lcd_write_byte(pDev, 0xA1);

    __lcd_write_command(pDev, 0xD6);
    __lcd_write_byte(pDev, 0xA1);

    __lcd_write_command(pDev, 0xE0);
    __lcd_write_byte(pDev, 0xF0);
    __lcd_write_byte(pDev, 0x06);
    __lcd_write_byte(pDev, 0x0B);
    __lcd_write_byte(pDev, 0x0A);
    __lcd_write_byte(pDev, 0x09);
    __lcd_write_byte(pDev, 0x26);
    __lcd_write_byte(pDev, 0x29);
    __lcd_write_byte(pDev, 0x33);
    __lcd_write_byte(pDev, 0x41);
    __lcd_write_byte(pDev, 0x18);
    __lcd_write_byte(pDev, 0x16);
    __lcd_write_byte(pDev, 0x15);
    __lcd_write_byte(pDev, 0x29);
    __lcd_write_byte(pDev, 0x2D);

    __lcd_write_command(pDev, 0xE1);
    __lcd_write_byte(pDev, 0xF0);
    __lcd_write_byte(pDev, 0x04);
    __lcd_write_byte(pDev, 0x08);
    __lcd_write_byte(pDev, 0x08);
    __lcd_write_byte(pDev, 0x07);
    __lcd_write_byte(pDev, 0x03);
    __lcd_write_byte(pDev, 0x28);
    __lcd_write_byte(pDev, 0x32);
    __lcd_write_byte(pDev, 0x40);
    __lcd_write_byte(pDev, 0x3B);
    __lcd_write_byte(pDev, 0x19);
    __lcd_write_byte(pDev, 0x18);
    __lcd_write_byte(pDev, 0x2A);
    __lcd_write_byte(pDev, 0x2E);

    __lcd_write_command(pDev, 0xE4);
    __lcd_write_byte(pDev, 0x25);
    __lcd_write_byte(pDev, 0x00);
    __lcd_write_byte(pDev, 0x00);

    __lcd_write_command(pDev, 0x21);

    __lcd_write_command(pDev, 0x11);
    __lcd_delay_ms(120);
    __lcd_write_command(pDev, 0x29);
	
	/* 屏幕初始化为黑色 */
	__lcd_clear(pDev, BLACK);
	
	/* 函数指针赋值 */
	pDev->clear = __lcd_clear;
	pDev->fill = __lcd_fill;
	pDev->color_fill = __lcd_color_fill;
	pDev->color_fill_dma = __lcd_color_fill_dma;
	pDev->show_char = __lcd_show_char;
	pDev->show_string = __lcd_show_string;
	pDev->show_num = __lcd_show_num;
	pDev->show_hex_num = __lcd_show_hex_num;
	pDev->show_float_num = __lcd_show_float_num;
	pDev->show_chinese = __lcd_show_chinese;
	pDev->show_image = __lcd_show_image;
	pDev->draw_point = __lcd_draw_point;
	pDev->draw_line = __lcd_draw_line;
	pDev->draw_rectangle = __lcd_draw_rectangle;
	pDev->draw_circle = __lcd_draw_circle;
	pDev->deinit = __lcd_deinit;
	
	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	LCD配置DMA传输，用于LVGL
 * @param	pDev			:	LCDDev_t结构体指针
 * @param	memoryBaseAddr	:	DMA内存地址
 * @return	无
 ******************************************************************************/
void lcd_dma_init(LCDDev_t *pDev, uint32_t memoryBaseAddr)
{
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	__lcd_config_dma_clock_enable(pDev->info.spix);	// 开启DMA时钟
	DMA_DeInit(__lcd_get_dma_channel(pDev->info.spix));
	DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&pDev->info.spix->DR;	// SPI数据寄存器地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)memoryBaseAddr;			// 内存地址
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
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)memoryBaseAddr;			// 内存地址
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
 * @brief	LCD复位
 * @param	pDev	:	LCDDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_reset(LCDDev_t *pDev)
{
	__lcd_res_write(pDev, 1);
	__lcd_delay_ms(100);
	__lcd_res_write(pDev, 0);
	__lcd_delay_ms(100);
	__lcd_res_write(pDev, 1);
	__lcd_delay_ms(100);
}

/******************************************************************************
 * @brief	LCD写命令
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	byte	:	要写入的命令值，范围：0x00~0xFF
 * @return	无
 ******************************************************************************/
static void __lcd_write_command(LCDDev_t *pDev, uint8_t command)
{
	LCDPrivData_t *pPrivData = (LCDPrivData_t *)pDev->pPrivData;
	
	__lcd_dc_write(pDev, 0);			// 写命令
	pPrivData->lcdSPI.start(&pPrivData->lcdSPI);				// 拉低CS，开始通信
	pPrivData->lcdSPI.swap_byte(&pPrivData->lcdSPI, command);	// 写入指定命令
	pPrivData->lcdSPI.stop(&pPrivData->lcdSPI);					// 拉高CS，结束通信
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
	
	__lcd_dc_write(pDev, 1);			// 写数据
	pPrivData->lcdSPI.start(&pPrivData->lcdSPI);				// 拉低CS，开始通信
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
	
	__lcd_dc_write(pDev, 1);			// 写数据
	pPrivData->lcdSPI.start(&pPrivData->lcdSPI);					// 拉低CS，开始通信
	pPrivData->lcdSPI.swap_byte(&pPrivData->lcdSPI, halfword >> 8);	// 写入数据
	pPrivData->lcdSPI.swap_byte(&pPrivData->lcdSPI, halfword);		// 写入数据
	pPrivData->lcdSPI.stop(&pPrivData->lcdSPI);						// 拉高CS，结束通信
}

/**********************************************************************通信协议*/

/*硬件配置**********************************************************************/

/******************************************************************************
 * @brief	LCD设置屏幕方向
 * @param	pDev	:	LCDDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_set_direction(LCDDev_t *pDev)
{
    uint8_t memoryAccessReg = 0x00;

    if (pDev->info.dir == HORIZONTAL_FORWARD || pDev->info.dir == HORIZONTAL_REVERSE)
	{
		pDev->width = LCD_H;
		pDev->height = LCD_W;
        memoryAccessReg = 0X70;
    }
    else if (pDev->info.dir == VERTICAL_FORWARD || pDev->info.dir == VERTICAL_REVERSE)
	{    
		pDev->width = LCD_W;
		pDev->height = LCD_H;
        memoryAccessReg = 0X00;
    }

    __lcd_write_command(pDev, 0x36);
    __lcd_write_byte(pDev, memoryAccessReg);
}

/******************************************************************************
 * @brief	LCD设置起始和结束地址
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x1,x2	:	设置列的起始和结束地址
 * @param	y1,y2	:	设置行的起始和结束地址
 * @return	无
 ******************************************************************************/
static void __lcd_set_windows(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	if (pDev->info.dir == VERTICAL_FORWARD || pDev->info.dir == VERTICAL_REVERSE)	// 竖屏
	{
		__lcd_write_command(pDev, 0x2a);			// 列地址设置
		__lcd_write_halfword(pDev, x1);
		__lcd_write_halfword(pDev, x2);

		__lcd_write_command(pDev, 0x2b);			// 行地址设置
		__lcd_write_halfword(pDev, y1 + 20);
		__lcd_write_halfword(pDev, y2 + 20);

	}
	else if (pDev->info.dir == HORIZONTAL_FORWARD || pDev->info.dir == HORIZONTAL_REVERSE)	// 横屏
	{
		__lcd_write_command(pDev, 0x2a);			// 列地址设置
		__lcd_write_byte(pDev, (x1 + 20) >> 8);
        __lcd_write_byte(pDev, x1 + 20);
        __lcd_write_byte(pDev, (x2 + 20) >> 8);
        __lcd_write_byte(pDev, x2 + 20);

		__lcd_write_command(pDev, 0x2b);			// 行地址设置
        __lcd_write_byte(pDev, y1 >> 8);
        __lcd_write_byte(pDev, y1);
        __lcd_write_byte(pDev, (y2) >> 8);
        __lcd_write_byte(pDev, y2);
	}

	__lcd_write_command(pDev, 0x2c);				// 储存器写 
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
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	color	:	要填充的颜色
 * @return	无
 ******************************************************************************/
static void __lcd_clear(LCDDev_t *pDev, uint16_t color)
{          
	uint16_t i, j;
    
    __lcd_set_windows(pDev, 0, 0, pDev->width, pDev->height);

	__lcd_dc_write(pDev, 1);

    for(i = 0; i < LCD_W; i++)
	{
        for(j = 0; j < LCD_H; j++)
		{
			__lcd_write_halfword(pDev, color);
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
static void __lcd_fill(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
	uint16_t i, j;

	__lcd_set_windows(pDev, x, y, x + width - 1, y + height - 1);	// 设置显示范围
	for (i = y; i < y + height; i++)
	{													   	 	
		for (j = x; j < x + width; j++)
		{
			__lcd_write_halfword(pDev, color);
		}
	}
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色，用于LVGL
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x1		:	填充起点x坐标
 * @param	y1		:	填充起点y坐标
 * @param	x2		:	填充终点x坐标
 * @param	y2		:	填充终点y坐标
 * @param	color	:	要填充的颜色数组
 * @return	无
 ******************************************************************************/
static void __lcd_color_fill(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color)
{
    uint16_t height, width;
    uint16_t i, j;
    width = x2 - x1 + 1;            // 填充的宽度
    height = y2 - y1 + 1;           // 填充的高度

	__lcd_set_windows(pDev, x1, y1, x2, y2);					//设置光标位置
	
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            __lcd_write_halfword(pDev, color[i * width + j]);	// 写入数据
        }
    }
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色，并使用DMA传输，用于LVGL
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x1		:	填充起点x坐标
 * @param	y1		:	填充起点y坐标
 * @param	x2		:	填充终点x坐标
 * @param	y2		:	填充终点y坐标
 * @param	size	:	传输大小
 * @return	无
 ******************************************************************************/
static void __lcd_color_fill_dma(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t size)
{
	LCDPrivData_t *pPrivData = (LCDPrivData_t *)pDev->pPrivData;
	
	__lcd_set_windows(pDev, x1, y1, x2, y2);

	__lcd_dc_write(pDev, 1);										// 写数据

	pPrivData->lcdSPI.start(&pPrivData->lcdSPI);					// 拉低CS，开始通信
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	/* STM32F10X代码待编写 */
	#elif defined(STM32F40_41xxx)
	/* 开启SPI的DMA接收 */
	SPI_I2S_DMACmd(pDev->info.spix, SPI_I2S_DMAReq_Tx, ENABLE);

	/* 使能DMA */
	DMA_Cmd(__lcd_get_dma_stream(pDev->info.spix), DISABLE);
	while (DMA_GetCmdStatus(__lcd_get_dma_stream(pDev->info.spix)) != DISABLE);
	DMA_SetCurrDataCounter(__lcd_get_dma_stream(pDev->info.spix), size);
	DMA_Cmd(__lcd_get_dma_stream(pDev->info.spix), ENABLE);
	
	/* 等待传输完成 */
	while (DMA_GetFlagStatus(__lcd_get_dma_stream(pDev->info.spix),
							 __lcd_get_dma_flag(pDev->info.spix)) == RESET);

	/* 清除标志位 */
	DMA_ClearFlag(__lcd_get_dma_stream(pDev->info.spix),
				  __lcd_get_dma_flag(pDev->info.spix));
	
	pPrivData->lcdSPI.stop(&pPrivData->lcdSPI);						// 拉高CS，结束通信
	#endif
}

/******************************************************************************
 * @brief	LCD显示单个字符
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chr		:	要显示的字符
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_char(LCDDev_t *pDev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	uint8_t temp, sizex, t, m = 0;
	uint16_t i, TypefaceNum;									// 一个字符所占字节大小
	uint16_t x0 = x;
	sizex = size / 2;
	TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * size;
	chr = chr - ' ';    								// 得到偏移后的值

	for (i = 0; i < TypefaceNum; i++)
	{ 
		if (size==12)temp=LCD_F6x12[chr][i];			// 调用6x12字体
		else if (size==16)temp=LCD_F8x16[chr][i];		// 调用8x16字体
		else if (size==24)temp=LCD_F12x24[chr][i];		// 调用12x24字体
		else if (size==32)temp=LCD_F16x32[chr][i];		// 调用16x32字体
		else return;

		for (t = 0;t < 8;t++)
		{
			if (!mode)									// 非叠加模式
			{
				if (temp & (0x01 << t)) __lcd_draw_point(pDev, x, y, fc);
                else __lcd_draw_point(pDev, x, y, bc);
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
				if (temp & (0x01 << t)) __lcd_draw_point(pDev, x, y, fc);
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
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	str		:	要显示的字符
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_string(LCDDev_t *pDev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{         
	while (*str != '\0')
	{       
		__lcd_show_char(pDev, x, y, *str, fc, bc, size, mode);
		x += size / 2;
		str++;
	}
}

/******************************************************************************
 * @brief	LCD显示整数变量
 * @param	pDev	:	LCDDev_t结构体指针
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
static void __lcd_show_num(LCDDev_t *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
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
                __lcd_show_char(pDev, x + (size / 2)*t, y, ' ', fc, bc, size, mode);
                continue;
            }
            else enshow = 1;
        }

        __lcd_show_char(pDev, x + (size / 2)*t, y, temp + '0', fc, bc, size, mode);
    }
}

/******************************************************************************
 * @brief	LCD显示十六进制数
 * @param	pDev	:	LCDDev_t结构体指针
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
static void __lcd_show_hex_num(LCDDev_t *pDev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	uint8_t i, singleNumber;

	for (i = 0; i < len; i++)		// 遍历数字的每一位
	{
		/* 以十六进制提取数字的每一位 */
		singleNumber = num / __lcd_pow(16, len - i - 1) % 16;
		
		if (singleNumber < 10)			// 单个数字小于10
		{
			/* 调用__oled_show_char函数，显示此数字 */
			/* + '0' 可将数字转换为字符格式 */
			__lcd_show_char(pDev, x + i * size / 2, y, singleNumber + '0', fc, bc, size, mode);
		}
		else							// 单个数字大于10
		{
			/* 调用__oled_show_char函数，显示此数字 */
			/* + 'A' 可将数字转换为从A开始的十六进制字符 */
			__lcd_show_char(pDev, x + i * size / 2, y, singleNumber - 10 + 'A', fc, bc, size, mode);
		}
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
static void __lcd_show_float_num(LCDDev_t *pDev, uint16_t x, uint16_t y, float num, uint8_t intLen, uint8_t fraLen, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
    uint32_t powNum, intNum, fraNum;
	
	if (num < 0)
	{
		__lcd_show_char(pDev, x, y, '-', fc, bc, size, mode);	// 显示-号
	}
	
	/* 提取整数部分和小数部分 */
	intNum = num;						// 直接赋值给整型变量，提取整数
	num -= intNum;						// 将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	powNum = __lcd_pow(10, fraLen);		// 根据指定小数的位数，确定乘数
	fraNum = round(num * powNum);		// 将小数乘到整数，同时四舍五入，避免显示误差
	intNum += fraNum / powNum;			// 若四舍五入造成了进位，则需要再加给整数
	
	if (num >= 0)
	{
		__lcd_show_num(pDev, x, y, intNum, intLen, fc, bc, size, mode);								// 显示整数部分
		__lcd_show_char(pDev, x + (intLen) * size / 2, y, '.', fc, bc, size, mode);					// 显示小数点
		__lcd_show_num(pDev, x + (intLen + 1) * size / 2, y, fraNum, fraLen, fc, bc, size, mode);	// 显示小数部分
	}
	else
	{
		num = -num;
		__lcd_show_num(pDev, x + size / 2, y, intNum, intLen, fc, bc, size, mode);					// 显示整数部分
		__lcd_show_char(pDev, x + (intLen + 1) * size / 2, y, '.', fc, bc, size, mode);				// 显示小数点
		__lcd_show_num(pDev, x + (intLen + 2) * size / 2, y, fraNum, fraLen, fc, bc, size, mode);	// 显示小数部分
	}
}

/******************************************************************************
 * @brief	LCD显示单个12x12汉字
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese12x12(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(12); 					// 汉字数目
	uint16_t TypefaceNum = (12 / 8 + ((12 % 8) ? 1 : 0)) * 12;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF12x12[k].Index[0] == *(Chinese)) && (LCD_CF12x12[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_windows(pDev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF12x12[k].Msk[i] & (0x01 << j))
					__lcd_draw_point(pDev, x, y, fc); // 画一个点
					else if (mode == 0)
						__lcd_draw_point(pDev, x, y, bc);
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
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese16x16(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(16); 					// 汉字数目
	uint16_t TypefaceNum = (16 / 8 + ((16 % 8) ? 1 : 0)) * 16;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF16x16[k].Index[0] == *(Chinese)) && (LCD_CF16x16[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_windows(pDev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF16x16[k].Msk[i] & (0x01 << j))
						__lcd_draw_point(pDev, x, y, fc); // 画一个点
					else if (mode == 0)
						__lcd_draw_point(pDev, x, y, bc);
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
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese24x24(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(24); 					// 汉字数目
	uint16_t TypefaceNum = (24 / 8 + ((24 % 8) ? 1 : 0)) * 24;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF24x24[k].Index[0] == *(Chinese)) && (LCD_CF24x24[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_windows(pDev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF24x24[k].Msk[i] & (0x01 << j))
						__lcd_draw_point(pDev, x, y, fc); // 画一个点
					else if (mode == 0)
						__lcd_draw_point(pDev, x, y, bc);
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
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese32x32(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t ChineseNum = GetChineseNum(32); 					// 汉字数目
	uint16_t TypefaceNum = (32 / 8 + ((32 % 8) ? 1 : 0)) * 32;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < ChineseNum; k++)
	{
		if ((LCD_CF32x32[k].Index[0] == *(Chinese)) && (LCD_CF32x32[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_windows(pDev, x, y, x + 12 - 1, y + 12 - 1);
			for (i = 0; i < TypefaceNum; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF32x32[k].Msk[i] & (0x01 << j))
						__lcd_draw_point(pDev, x, y, fc); //画一个点
					else if (mode == 0)
						__lcd_draw_point(pDev, x, y, bc);
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
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字串
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese(LCDDev_t *pDev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	while (*Chinese != 0)
	{
		if (size==LCD_12X12)		__lcd_show_chinese12x12(pDev, x, y, Chinese, fc, bc, mode);
		else if (size==LCD_16X16)	__lcd_show_chinese16x16(pDev, x, y, Chinese, fc, bc, mode);
		else if (size==LCD_24X24)	__lcd_show_chinese24x24(pDev, x, y, Chinese, fc, bc, mode);
		else if (size==LCD_32X32)	__lcd_show_chinese32x32(pDev, x, y, Chinese, fc, bc, mode);
		else return;
		
		Chinese += LCD_CHN_CHAR_WIDTH;
		x += size;
	}
}

/******************************************************************************
 * @brief	LCD显示图片
 * @param	pDev	:	LCDDev_t结构体指针
 * @param	x		:	图片左上角x坐标
 * @param	y		:	图片左上角y坐标
 * @param	width	:	图片宽度
 * @param	height	:	图片高度
 * @param	pic[]	:	图片数组
 * @return	无
 ******************************************************************************/
static void __lcd_show_image(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{
	uint16_t i, j; 
	uint32_t k = 0;

	__lcd_set_windows(pDev, x, y, x + width - 1, y + height - 1);	// 设置显示范围
	for (i = 0; i < height; i++)
	{													   	 	
		for (j = 0; j < width; j++)
		{
			__lcd_write_byte(pDev, pic[k * 2]);
			__lcd_write_byte(pDev, pic[k * 2 + 1]);
			k++;
		}
	}
}

/******************************************************************************
 * @brief	LCD画点
 * @param	pDev	:  LCDDev_t结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_point(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t color)
{
	__lcd_set_windows(pDev, x, y, x, y);		// 设置光标位置 
	__lcd_write_halfword(pDev, color);
}

/******************************************************************************
 * @brief	LCD画线
 * @param	pDev	:  LCDDev_t结构体指针
 * @param	x1		:  起始x坐标
 * @param	y1		:  起始y坐标
 * @param	x2		:  终点x坐标
 * @param	y2		:  终点y坐标
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_line(LCDDev_t *pDev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
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
        __lcd_draw_point(pDev, uRow, uCol, color); // 画点
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
 * @param	pDev	:  LCDDev_t结构体指针
 * @param	x		:  矩形左上角x坐标
 * @param	y		:  矩形左上角y坐标
 * @param	width	:  矩形宽度
 * @param	height	:  矩形高度
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_rectangle(LCDDev_t *pDev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
	uint16_t x_end = x + width - 1;
    uint16_t y_end = y + height - 1;

	__lcd_draw_line(pDev, x, y, x_end, y, color);
	__lcd_draw_line(pDev, x, y, x, y_end, color);
	__lcd_draw_line(pDev, x, y_end, x_end, y_end, color);
	__lcd_draw_line(pDev, x_end, y, x_end, y_end, color);
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
	
	while (a <= b)
	{
		__lcd_draw_point(pDev, x - b, y - a, color);	// 3           
		__lcd_draw_point(pDev, x + b, y - a, color);	// 0           
		__lcd_draw_point(pDev, x - a, y + b, color);	// 1                
		__lcd_draw_point(pDev, x - a, y - b, color);	// 2             
		__lcd_draw_point(pDev, x + b, y + a, color);	// 4               
		__lcd_draw_point(pDev, x + a, y - b, color);	// 5
		__lcd_draw_point(pDev, x + a, y + b, color);	// 6 
		__lcd_draw_point(pDev, x - b, y + a, color);	// 7
		a++;
		if ((a * a + b * b) > (radius * radius))		// 判断要画的点是否过远
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
	
	/* 去初始化硬件SPI */
	pPrivData->lcdSPI.deinit(&pPrivData->lcdSPI);

	/* 去初始化PWM */
	pPrivData->lcdPWM.deinit(&pPrivData->lcdPWM);
	
	/* 释放私有数据内存 */
	free(pDev->pPrivData);
	pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	// 修改初始化标志
	return 0;
}

/**********************************************************************功能函数*/
