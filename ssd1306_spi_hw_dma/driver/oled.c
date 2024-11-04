#include "oled.h"
#include "oled_data.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__oled_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
													else					{OLED_Log("oled gpio clock no enable\r\n");} \
												}

#define	__oled_config_gpio_clock_disable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);} \
													else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, DISABLE);} \
													else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);} \
													else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, DISABLE);} \
													else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, DISABLE);} \
													else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, DISABLE);} \
													else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, DISABLE);} \
													else					{OLED_Log("oled gpio clock no disable\r\n");} \
												}
												
#define	__oled_config_dma_clock_enable(spix)	{	if(spix == SPI1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if(spix == SPI2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);} \
													else if(spix == SPI3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);} \
													else					{OLED_Log("oled dma clock no enable\r\n");} \
												}

#define	__oled_config_dma_clock_disable(spix)	{	if(spix == SPI1)		{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);} \
													else if(spix == SPI2)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);} \
													else if(spix == SPI3)	{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, DISABLE);} \
													else					{OLED_Log("oled dma clock no disable\r\n");} \
												}
													
#define	__oled_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __oled_io_write(port ,pin, value)	GPIO_WriteBit(port ,pin, (BitAction)value)

#define	__oled_get_dma_channel(spix)		(	spix == SPI1 ? DMA1_Channel3 : \
												spix == SPI2 ? DMA1_Channel5 : \
												spix == SPI3 ? DMA2_Channel2 : \
												(int)0)
												
#define	__oled_get_dma_flag(spix)		(	spix == SPI1 ? DMA1_FLAG_TC3 : \
											spix == SPI2 ? DMA1_FLAG_TC5 : \
											spix == SPI3 ? DMA2_FLAG_TC2 : \
											(int)0)

#define __oled_gpio_deinit(port)	GPIO_DeInit(port)
			
#elif defined(STM32F40_41xxx)

#define	__oled_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													else					{OLED_Log("oled gpio clock no enable\r\n");} \
												}

#define	__oled_config_gpio_clock_disable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, DISABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, DISABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, DISABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, DISABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, DISABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, DISABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, DISABLE);} \
													else					{OLED_Log("oled gpio clock no disable\r\n");} \
												}

#define	__oled_config_gpio_clock_disable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, DISABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, DISABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, DISABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, DISABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, DISABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, DISABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, DISABLE);} \
													else					{OLED_Log("oled gpio clock no disable\r\n");} \
												}

#define	__oled_config_dma_clock_enable(spix)	{	if(spix == SPI1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);} \
													else if(spix == SPI2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else if(spix == SPI3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);} \
													else					{OLED_Log("oled dma clock no enable\r\n");} \
												}

#define	__oled_config_dma_clock_disable(spix)	{	if(spix == SPI1)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, DISABLE);} \
													else if(spix == SPI2)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, DISABLE);} \
													else if(spix == SPI3)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, DISABLE);} \
													else					{OLED_Log("oled dma clock no disable\r\n");} \
												}
													
#define	__oled_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin ; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __oled_io_write(port ,pin, value)	GPIO_WriteBit(port ,pin, (BitAction)value)
											
#define __oled_get_dma_stream(spix)	(	spix == SPI1 ? DMA2_Stream3 : \
										spix == SPI2 ? DMA1_Stream4 : \
										spix == SPI3 ? DMA1_Stream5 : \
										(int)0)

#define	__oled_get_dma_channel(spix)		(	spix == SPI1 ? DMA_Channel_3 : \
												spix == SPI2 ? DMA_Channel_0 : \
												spix == SPI3 ? DMA_Channel_0 : \
												(int)0)
												
#define	__oled_get_dma_flag(spix)		(	spix == SPI1 ? DMA_FLAG_TCIF3 : \
											spix == SPI2 ? DMA_FLAG_TCIF4 : \
											spix == SPI3 ? DMA_FLAG_TCIF5 : \
											(int)0)

#define __oled_gpio_deinit(port)	GPIO_DeInit(port)
											
#endif

/* 最大OLED设备数量 */			
#define MAX_OLED_NUM 4

/**
  * OLED显存数组
  * 所有的显示函数，都只是对此显存数组进行读写
  * 随后调用__oled_update函数或__oled_update_area函数
  * 才会将显存数组的数据发送到OLED硬件，进行显示
  */
uint8_t gOLEDDisplayBuf[MAX_OLED_NUM][8][128];			
						
/* 全局变量用于给注册的OLED设备分配索引 */
static uint8_t gIndex = 0;

/* OLED私有数据结构体 */
typedef struct {
	SPIDev_t oled;		// 硬件SPI设备
	uint8_t index;		// 索引
}OLEDPrivData_t;
				
/* 通信协议 */
static void __oled_res_write(OLEDDev_t *pDev, uint8_t bitValue);
static void __oled_dc_write(OLEDDev_t *pDev, uint8_t bitValue);
static int __oled_write_command(OLEDDev_t *pDev, uint8_t command);
static int __oled_write_data(OLEDDev_t *pDev, uint8_t *data, uint8_t count);
static int __oled_write_data_dma(OLEDDev_t *pDev, uint8_t *data, uint16_t count);

/* 功能函数 */
static int __oled_update(OLEDDev_t *pDev);
static int __oled_update_area(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
static int __oled_clear(OLEDDev_t *pDev);
static int __oled_clear_area(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
static int __oled_reverse(OLEDDev_t *pDev);
static int __oled_reverse_area(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
static int __oled_show_image(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image);
static int __oled_show_char(OLEDDev_t *pDev, uint8_t x, uint8_t y, char Char, uint8_t fontSize);
static int __oled_show_string(OLEDDev_t *pDev, uint8_t x, uint8_t y, char *string, uint8_t fontSize);
static int __oled_show_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
static int __oled_show_signed_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, int32_t number, uint8_t length, uint8_t fontSize);
static int __oled_show_hex_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
static int __oled_show_bin_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize);
static int __oled_show_float_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, double number, uint8_t intLength, uint8_t fraLength, uint8_t fontSize);
static int __oled_show_chinese(OLEDDev_t *pDev, uint8_t x, uint8_t y, char *Chinese);
static int __oled_printf(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t fontSize, char *format, ...);
static int __oled_draw_point(OLEDDev_t *pDev, uint8_t x, uint8_t y);
static uint8_t __oled_get_point(OLEDDev_t *pDev, uint8_t x, uint8_t y);
static int __oled_draw_line(OLEDDev_t *pDev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
static int __oled_draw_rectangle(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t isFilled);
static int __oled_draw_triangle(OLEDDev_t *pDev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t IsFilled);
static int __oled_draw_circle(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t radius, uint8_t IsFilled);
static int __oled_draw_ellipse(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t isFilled);
static int __oled_draw_arc(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t radius, int16_t startAngle, int16_t endAngle, uint8_t isFilled);
static int __oled_deinit(OLEDDev_t *pDev);

/******************************************************************************
 * @brief	初始化OLED
 * @param	pDev		:	OLEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int oled_init(OLEDDev_t *pDev)
{
	if (!pDev)
		return -1;
	
	/* 在初始化前，加入适量延时，待OLED供电稳定 */
	for (uint32_t i = 0; i < 1000; i ++)
		for (uint32_t j = 0; j < 1000; j ++);
	
	/* 保存私有数据 */
	pDev->pPrivData = (OLEDPrivData_t *)malloc(sizeof(OLEDPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	pPrivData->oled.info.spix = pDev->info.spix;
	pPrivData->oled.info.CSPort = pDev->info.CSPort;
	pPrivData->oled.info.CSPin = pDev->info.CSPin;
	pPrivData->oled.info.prescaler = pDev->info.prescaler;
	pPrivData->oled.info.mode = pDev->info.mode;
	
	pPrivData->index = gIndex++;
	
	/* 配置硬件SPI */
	spi_init(&pPrivData->oled);
	
	/* 配置时钟与GPIO */
	__oled_config_gpio_clock_enable(pDev->info.RESPort);
	__oled_config_gpio_clock_enable(pDev->info.DCPort);
	
	__oled_config_io_out_pp(pDev->info.RESPort, pDev->info.RESPin);
	__oled_config_io_out_pp(pDev->info.DCPort, pDev->info.DCPin);
	
	/* 置引脚默认电平 */
	__oled_res_write(pDev, 1);
	__oled_dc_write(pDev, 1);
	pPrivData->oled.cs_write(&pPrivData->oled, 1);
	
	/* 函数指针赋值 */
	pDev->update = __oled_update;
	pDev->update_area = __oled_update_area;
	pDev->clear = __oled_clear;
	pDev->clear_area = __oled_clear_area;
	pDev->reverse = __oled_reverse;
	pDev->reverse_area = __oled_reverse_area;
	pDev->show_image = __oled_show_image;
	pDev->show_char = __oled_show_char;
	pDev->show_string = __oled_show_string;
	pDev->show_num = __oled_show_num;
	pDev->show_signed_num = __oled_show_signed_num;
	pDev->show_hex_num = __oled_show_hex_num;
	pDev->show_bin_num = __oled_show_bin_num;
	pDev->show_float_num = __oled_show_float_num;
	pDev->show_chinese = __oled_show_chinese;
	pDev->printf = __oled_printf;
	pDev->draw_point = __oled_draw_point;
	pDev->get_point = __oled_get_point;
	pDev->draw_line = __oled_draw_line;
	pDev->draw_rectangle = __oled_draw_rectangle;
	pDev->draw_triangle = __oled_draw_triangle;
	pDev->draw_circle = __oled_draw_circle;
	pDev->draw_ellipse = __oled_draw_ellipse;
	pDev->draw_arc = __oled_draw_arc;
	pDev->deinit = __oled_deinit;
	
	pDev->initFlag = true;
	
	/* 配置OLED */
	__oled_write_command(pDev, 0xAE);	// 设置显示开启/关闭，0xAE关闭，0xAF开启
	
	__oled_write_command(pDev, 0xD5);	// 设置显示时钟分频比/振荡器频率
	__oled_write_command(pDev, 0x80);	// 0x00~0xFF
	
	__oled_write_command(pDev, 0xA8);	// 设置多路复用率
	__oled_write_command(pDev, 0x3F);	// 0x0E~0x3F
	
	__oled_write_command(pDev, 0xD3);	// 设置显示偏移
	__oled_write_command(pDev, 0x00);	// 0x00~0x7F
	
	__oled_write_command(pDev, 0x40);	// 设置显示开始行，0x40~0x7F
	
	if(!OLED_DIRECTION)
	{
		__oled_write_command(pDev, 0xA1);	// 设置左右方向，0xA1正常，0xA0左右反置
		__oled_write_command(pDev, 0xC8);	// 设置上下方向，0xC8正常，0xC0上下反置
	}
	else
	{
		__oled_write_command(pDev, 0xA0);	// 设置左右方向，0xA1正常，0xA0左右反置
		__oled_write_command(pDev, 0xC0);	// 设置上下方向，0xC8正常，0xC0上下反置
	}
	
	__oled_write_command(pDev, 0xDA);	// 设置COM引脚硬件配置
	__oled_write_command(pDev, 0x12);
	
	__oled_write_command(pDev, 0x81);	// 设置对比度
	__oled_write_command(pDev, 0xCF);	// 0x00~0xFF
	
	__oled_write_command(pDev, 0xD9);	// 设置预充电周期
	__oled_write_command(pDev, 0xF1);
	
	__oled_write_command(pDev, 0xDB);	// 设置VCOMH取消选择级别
	__oled_write_command(pDev, 0x30);
	
	__oled_write_command(pDev, 0xA4);	// 设置整个显示打开/关闭
	
	__oled_write_command(pDev, 0xA6);	// 设置正常/反色显示，0xA6正常，0xA7反色
	
	__oled_write_command(pDev, 0x8D);	// 设置充电泵
	__oled_write_command(pDev, 0x14);

	__oled_write_command(pDev, 0xAF);	// 开启显示
	
	__oled_clear(pDev);					// 清空显存数组
	__oled_update(pDev);				// 更新显示，清屏，防止初始化后未显示内容时花屏

	return 0;
}

/*通信协议*********************************************************************/

/******************************************************************************
 * @brief	OLED写RES高低电平
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	bitValue	:	要写入RES的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __oled_res_write(OLEDDev_t *pDev, uint8_t bitValue)
{
	__oled_io_write(pDev->info.RESPort, pDev->info.RESPin, bitValue);
}

/******************************************************************************
 * @brief	OLED写DC高低电平
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	bitValue	:	要写入DC的电平值，范围：0/1
 * @return	无
 ******************************************************************************/
static void __oled_dc_write(OLEDDev_t *pDev, uint8_t bitValue)
{
	__oled_io_write(pDev->info.DCPort, pDev->info.DCPin, bitValue);
}

/******************************************************************************
 * @brief	OLED写命令
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	command	:	要写入的命令值，范围：0x00~0xFF
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_write_command(OLEDDev_t *pDev, uint8_t command)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	pPrivData->oled.start(&pPrivData->oled);					// 拉低CS，开始通信
	__oled_dc_write(pDev, 0);									// 拉低DC，表示即将发送命令
	pPrivData->oled.swap_byte(&pPrivData->oled, command);		// 写入指定命令
	pPrivData->oled.stop(&pPrivData->oled);						// 拉高CS，结束通信
	
	return 0;
}

/******************************************************************************
 * @brief	OLED写数据
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	data	:	要写入数据的起始地址
 * @param	count	:	要写入数据的数量
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_write_data(OLEDDev_t *pDev, uint8_t *data, uint8_t count)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i;
	
	pPrivData->oled.start(&pPrivData->oled);				// 拉低CS，开始通信
	__oled_dc_write(pDev, 1);								// 拉高DC，表示即将发送数据
	/* 循环count次，进行连续的数据写入 */
	for (i = 0; i < count; i++)
	{
		pPrivData->oled.swap_byte(&pPrivData->oled, data[i]);
	}
	pPrivData->oled.stop(&pPrivData->oled);					// 拉高CS，结束通信
	
	return 0;
}

/******************************************************************************
 * @brief	OLED写数据（DMA）
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	data	:	要写入数据的起始地址
 * @param	count	:	要写入数据的数量
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_write_data_dma(OLEDDev_t *pDev, uint8_t *data, uint16_t count)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	if (!pDev || !pDev->initFlag) return -1;
	
	pPrivData->oled.start(&pPrivData->oled);				// SPI起始
	__oled_dc_write(pDev, 1);								// 拉高DC，表示即将发送数据
	
	__oled_config_dma_clock_enable(pDev->info.spix);		// 开启DMA时钟
	
	#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(__oled_get_dma_channel(pDev->info.spix));
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&pDev->info.spix->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(__oled_get_dma_channel(pDev->info.spix), &DMA_InitStructure);
	
	SPI_I2S_DMACmd(pDev->info.spix, SPI_I2S_DMAReq_Tx, ENABLE);			// 配置SPI为DMA模式
	
	DMA_Cmd(__oled_get_dma_channel(pDev->info.spix), ENABLE);			// 启动DMA传输
	
	while (!DMA_GetFlagStatus(__oled_get_dma_flag(pDev->info.spix)));	// 等待DMA传输完成
	
	DMA_ClearFlag(__oled_get_dma_flag(pDev->info.spix));				// 清除DMA传输完成标志
	
	DMA_Cmd(__oled_get_dma_channel(pDev->info.spix), DISABLE);			// 禁用DMA传输
	
	#elif defined(STM32F40_41xxx)
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(__oled_get_dma_stream(pDev->info.spix));
	DMA_InitStructure.DMA_Channel = __oled_get_dma_channel(pDev->info.spix);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&pDev->info.spix->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(__oled_get_dma_stream(pDev->info.spix), &DMA_InitStructure);
	
	SPI_I2S_DMACmd(pDev->info.spix, SPI_I2S_DMAReq_Tx, ENABLE);				// 配置SPI为DMA模式
	
	DMA_Cmd(__oled_get_dma_stream(pDev->info.spix), ENABLE);				// 启动DMA传输
	
	while (!DMA_GetFlagStatus(	__oled_get_dma_stream(pDev->info.spix), 
								__oled_get_dma_flag(pDev->info.spix))	);	// 等待DMA传输完成
	
	DMA_ClearFlag(	__oled_get_dma_stream(pDev->info.spix), 
					__oled_get_dma_flag(pDev->info.spix)	);				// 清除DMA传输完成标志
	
	DMA_Cmd(__oled_get_dma_stream(pDev->info.spix), DISABLE);				// 禁用DMA传输
	
	#endif
	
	pPrivData->oled.stop(&pPrivData->oled);	// SPI终止
	
	return 0;
}

/**********************************************************************通信协议*/

/*硬件配置**********************************************************************/

/******************************************************************************
 * @brief	OLED设置显示光标位置，OLED默认的y轴，只能8个Bit为一组写入，即1页等于8个y轴坐标
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	page	:	指定光标所在的页，范围：0~7
 * @param	x		:	指定光标所在的x轴坐标，范围：0~127
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int OLED_SetCursor(OLEDDev_t *pDev, uint8_t page, uint8_t x)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	/*如果使用此程序驱动1.3寸的OLED显示屏，则需要解除此注释*/
	/*因为1.3寸的OLED驱动芯片（SH1106）有132列*/
	/*屏幕的起始列接在了第2列，而不是第0列*/
	/*所以需要将x加2，才能正常显示*/
//	x += 2;
	
	/* 通过指令设置页地址和列地址 */
	__oled_write_command(pDev, 0xB0 | page);				// 设置页位置
	__oled_write_command(pDev, 0x10 | ((x & 0xF0) >> 4));	// 设置x位置高4位
	__oled_write_command(pDev, 0x00 | (x & 0x0F));			// 设置x位置低4位
	
	return 0;
}

/**********************************************************************硬件配置*/

/*工具函数**********************************************************************/

/******************************************************************************
 * @brief	次方函数
 * @param	x	:	底数
 * @param	y	:	指数
 * @return	x^y
 ******************************************************************************/
uint32_t OLED_Pow(uint32_t x, uint32_t y)
{
	uint32_t result = 1;	// 结果默认为1
	while (y --)			// 累乘y次
	{
		result *= x;		// 每次把x累乘到结果上
	}
	return result;
}

/******************************************************************************
 * @brief	判断指定点是否在指定多边形内部
 * @param	nvert		:	多边形的顶点数
 * @param	vertx verty	:	包含多边形顶点的x和y坐标的数组
 * @param	testx testy	:	测试点的x和y坐标
 * @return	指定点是否在指定多边形内部，1：在内部，0：不在内部
 ******************************************************************************/
uint8_t OLED_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
	int16_t i, j, c = 0;
	
	/*此算法由W. Randolph Franklin提出*/
	/*参考链接：https://wrfranklin.org/Research/Short_Notes/pnpoly.html*/
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
		{
			c = !c;
		}
	}
	return c;
}

/******************************************************************************
 * @brief	判断指定点是否在指定角度内部
 * @param	x y					:	指定点的坐标
 * @param	StartAngle EndAngle	:	起始角度和终止角度，范围：-180~180
			水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
 * @param	testx testy	:  测试点的x和y坐标
 * @return	指定点是否在指定角度内部，1：在内部，0：不在内部
 ******************************************************************************/
uint8_t OLED_IsInAngle(int16_t x, int16_t y, int16_t startAngle, int16_t endAngle)
{
	int16_t PointAngle;
	PointAngle = atan2(y, x) / 3.14 * 180;	// 计算指定点的弧度，并转换为角度表示
	if (startAngle < endAngle)	// 起始角度小于终止角度的情况
	{
		/* 如果指定角度在起始终止角度之间，则判定指定点在指定角度 */
		if (PointAngle >= startAngle && PointAngle <= endAngle)
		{
			return 1;
		}
	}
	else			// 起始角度大于于终止角度的情况
	{
		/* 如果指定角度大于起始角度或者小于终止角度，则判定指定点在指定角度 */
		if (PointAngle >= startAngle || PointAngle <= endAngle)
		{
			return 1;
		}
	}
	return 0;		// 不满足以上条件，则判断判定指定点不在指定角度
}

/**********************************************************************工具函数*/

/*功能函数**********************************************************************/

/******************************************************************************
 * @brief	将OLED显存数组更新到OLED屏幕
 * @param	pDev	:	OLEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_update(OLEDDev_t *pDev)
{
    OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
    if (!pDev || !pDev->initFlag) return -1;

    uint8_t j;
    /* 遍历每一页 */
    for (j = 0; j < 8; j++) {
        /* 设置光标位置为每一页的第一列 */
        OLED_SetCursor(pDev, j, 0);

        /* 使用DMA传输数据 */
        __oled_write_data_dma(pDev, gOLEDDisplayBuf[pPrivData->index][j], 128);
		//__oled_write_data(pDev, gOLEDDisplayBuf[pPrivData->index][j], 128);
    }
    return 0;
}
/******************************************************************************
 * @brief	将OLED显存数组部分更新到OLED屏幕，此函数会至少更新参数指定的区域
			如果更新区域y轴只包含部分页，则同一页的剩余部分会跟随一起更新
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	x		:	指定区域左上角的横坐标，范围：0~127
 * @param	y		:	指定区域左上角的纵坐标，范围：0~63
 * @param	width	:	指定区域的宽度，范围：0~128
 * @param	height	:	指定区域的高度，范围：0~64
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_update_area(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t j;
	
	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (x > 127) {return -1;}
	if (y > 63) {return -1;}
	if (x + width > 128) {width = 128 - x;}
	if (y + height > 64) {height = 64 - y;}
	
	/* 遍历指定区域涉及的相关页 */
	/* (y + height - 1) / 8 + 1的目的是(y + height) / 8并向上取整 */
	for (j = y / 8; j < (y + height - 1) / 8 + 1; j ++)
	{
		/* 设置光标位置为相关页的指定列 */
		OLED_SetCursor(pDev, j, x);
		/* 连续写入width个数据，将显存数组的数据写入到OLED硬件 */
		__oled_write_data(pDev, &gOLEDDisplayBuf[pPrivData->index][j][x], width);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	将OLED显存数组全部清零
 * @param	pDev	:	OLEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_clear(OLEDDev_t *pDev)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i, j;
	for (j = 0; j < 8; j ++)				// 遍历8页
	{
		for (i = 0; i < 128; i ++)			// 遍历128列
		{
			gOLEDDisplayBuf[pPrivData->index][j][i] = 0x00;	// 将显存数组数据全部清零
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	将OLED显存数组部分清零
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	x		:	指定区域左上角的横坐标，范围：0~127
 * @param	y		:	指定区域左上角的纵坐标，范围：0~63
 * @param	width	:	指定区域的宽度，范围：0~128
 * @param	height	:	指定区域的高度，范围：0~64
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_clear_area(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i, j;
	
	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (x > 127) {return -1;}
	if (y > 63) {return -1;}
	if (x + width > 128) {width = 128 - x;}
	if (y + height > 64) {height = 64 - y;}
	
	for (j = y; j < y + height; j ++)		// 遍历指定页
	{
		for (i = x; i < x + width; i ++)	// 遍历指定列
		{
			gOLEDDisplayBuf[pPrivData->index][j / 8][i] &= ~(0x01 << (j % 8));	// 将显存数组指定数据清零
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	将OLED显存数组全部取反
 * @param	pDev	:	OLEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_reverse(OLEDDev_t *pDev)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i, j;
	for (j = 0; j < 8; j ++)				// 遍历8页
	{
		for (i = 0; i < 128; i ++)			// 遍历128列
		{
			gOLEDDisplayBuf[pPrivData->index][j][i] ^= 0xFF;	// 将显存数组数据全部取反
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	将OLED显存数组部分取反
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	x		:	指定区域左上角的横坐标，范围：0~127
 * @param	y		:	指定区域左上角的纵坐标，范围：0~63
 * @param	width	:	指定区域的宽度，范围：0~128
 * @param	height	:	指定区域的高度，范围：0~64
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_reverse_area(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i, j;
	
	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (x > 127) {return -1;}
	if (y > 63) {return -1;}
	if (x + width > 128) {width = 128 - x;}
	if (y + height > 64) {height = 64 - y;}
	
	for (j = y; j < y + height; j ++)		// 遍历指定页
	{
		for (i = x; i < x + width; i ++)	// 遍历指定列
		{
			gOLEDDisplayBuf[pPrivData->index][j / 8][i] ^= 0x01 << (j % 8);	// 将显存数组指定数据取反
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示图像
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	x		:	指定图像左上角的横坐标，范围：0~127
 * @param	y		:	指定图像左上角的纵坐标，范围：0~63
 * @param	width	:	指定图像的宽度，范围：0~128
 * @param	height	:	指定图像的高度，范围：0~64
 * @param	image	:	指定要显示的图像
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_image(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i, j;
	
	/* 参数检查，保证指定图像不会超出屏幕范围 */
	if (x > 127) {return -1;}
	if (y > 63) {return -1;}
	
	/* 将图像所在区域清空 */
	__oled_clear_area(pDev, x, y, width, height);
	
	/* 遍历指定图像涉及的相关页*/
	/* (height - 1) / 8 + 1的目的是height / 8并向上取整 */
	for (j = 0; j < (height - 1) / 8 + 1; j ++)
	{
		/* 遍历指定图像涉及的相关列 */
		for (i = 0; i < width; i ++)
		{
			/* 超出边界，则跳过显示 */
			if (x + i > 127) {break;}
			if (y / 8 + j > 7) {return -1;}
			
			/* 显示图像在当前页的内容 */
			gOLEDDisplayBuf[pPrivData->index][y / 8 + j][x + i] |= image[j * width + i] << (y % 8);
			
			/* 超出边界，则跳过显示 */
			/* 使用continue的目的是，下一页超出边界时，上一页的后续内容还需要继续显示 */
			if (y / 8 + j + 1 > 7) {continue;}
			
			/* 显示图像在下一页的内容 */
			gOLEDDisplayBuf[pPrivData->index][y / 8 + j + 1][x + i] |= image[j * width + i] >> (8 - y % 8);
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示一个字符
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定字符左上角的横坐标，范围：0~127
 * @param	y			:	指定区域左上角的纵坐标，范围：0~63
 * @param	character	:	指定要显示的字符，范围：ASCII码可见字符
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_char(OLEDDev_t *pDev, uint8_t x, uint8_t y, char Char, uint8_t fontSize)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	if (fontSize == OLED_8X16)		// 字体为宽8像素，高16像素
	{
		/* 将ASCII字模库OLED_F8x16的指定数据以8*16的图像格式显示 */
		__oled_show_image(pDev, x, y, 8, 16, OLED_F8x16[Char - ' ']);
	}
	else if(fontSize == OLED_6X8)	//字体为宽6像素，高8像素
	{
		/* 将ASCII字模库OLED_F6x8的指定数据以6*8的图像格式显示 */
		__oled_show_image(pDev, x, y, 6, 8, OLED_F6x8[Char - ' ']);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示字符串
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定字符串左上角的横坐标，范围：0~127
 * @param	y			:	指定字符串左上角的纵坐标，范围：0~63
 * @param	string		:	指定要显示的字符串，范围：ASCII码可见字符组成的字符串
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_string(OLEDDev_t *pDev, uint8_t x, uint8_t y, char *string, uint8_t fontSize)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i;
	for (i = 0; string[i] != '\0'; i++)		//遍历字符串的每个字符
	{
		/* 调用__oled_show_char函数，依次显示每个字符 */
		__oled_show_char(pDev, x + i * fontSize, y, string[i], fontSize);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示数字（十进制，正整数）
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定数字左上角的横坐标，范围：0~127
 * @param	y			:	指定数字左上角的纵坐标，范围：0~63
 * @param	number		:	指定要显示的数字，范围：0~4294967295
 * @param	length		:	指定数字的长度，范围：0~10
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i;
	for (i = 0; i < length; i++)		// 遍历数字的每一位							
	{
		/* 调用__oled_show_char函数，依次显示每个数字 */
		/* Number / OLED_Pow(10, length - i - 1) % 10 可以十进制提取数字的每一位 */
		/* + '0' 可将数字转换为字符格式 */
		__oled_show_char(pDev, x + i * fontSize, y, number / OLED_Pow(10, length - i - 1) % 10 + '0', fontSize);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示有符号数字（十进制，整数）
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定数字左上角的横坐标，范围：0~127
 * @param	y			:	指定数字左上角的纵坐标，范围：0~63
 * @param	number		:	指定要显示的数字，范围：-2147483648~2147483647
 * @param	length		:	指定数字的长度，范围：0~10
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_signed_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, int32_t number, uint8_t length, uint8_t fontSize)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i;
	uint32_t Number1;
	
	if (number >= 0)								// 数字大于等于0
	{
		__oled_show_char(pDev, x, y, '+', fontSize);// 显示+号
		Number1 = number;							// Number1直接等于Number
	}
	else											// 数字小于0
	{
		__oled_show_char(pDev, x, y, '-', fontSize);// 显示-号
		Number1 = -number;							// Number1等于Number取负
	}
	
	for (i = 0; i < length; i++)					// 遍历数字的每一位								
	{
		/* 调用__oled_show_char函数，依次显示每个数字 */
		/* Number1 / OLED_Pow(10, length - i - 1) % 10 可以十进制提取数字的每一位 */
		/* + '0' 可将数字转换为字符格式 */
		__oled_show_char(pDev, x + (i + 1) * fontSize, y, Number1 / OLED_Pow(10, length - i - 1) % 10 + '0', fontSize);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示十六进制数字（十六进制，正整数）
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定数字左上角的横坐标，范围：0~127
 * @param	y			:	指定数字左上角的纵坐标，范围：0~63
 * @param	number		:	指定要显示的数字，范围：0x00000000~0xFFFFFFFF
 * @param	length		:	指定数字的长度，范围：0~8
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_hex_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i, SingleNumber;
	for (i = 0; i < length; i++)		// 遍历数字的每一位
	{
		/* 以十六进制提取数字的每一位 */
		SingleNumber = number / OLED_Pow(16, length - i - 1) % 16;
		
		if (SingleNumber < 10)			// 单个数字小于10
		{
			/* 调用__oled_show_char函数，显示此数字 */
			/* + '0' 可将数字转换为字符格式 */
			__oled_show_char(pDev, x + i * fontSize, y, SingleNumber + '0', fontSize);
		}
		else							// 单个数字大于10
		{
			/* 调用__oled_show_char函数，显示此数字 */
			/* + 'A' 可将数字转换为从A开始的十六进制字符 */
			__oled_show_char(pDev, x + i * fontSize, y, SingleNumber - 10 + 'A', fontSize);
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示二进制数字（二进制，正整数）
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定数字左上角的横坐标，范围：0~127
 * @param	y			:	指定数字左上角的纵坐标，范围：0~63
 * @param	number		:	指定要显示的数字，范围：0x00000000~0xFFFFFFFF
 * @param	length		:	指定数字的长度，范围：0~16
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_bin_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint32_t number, uint8_t length, uint8_t fontSize)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i;
	for (i = 0; i < length; i++)		// 遍历数字的每一位	
	{
		/* 调用__oled_show_char函数，依次显示每个数字 */
		/* Number / OLED_Pow(2, length - i - 1) % 2 可以二进制提取数字的每一位 */
		/* + '0' 可将数字转换为字符格式 */
		__oled_show_char(pDev, x + i * fontSize, y, number / OLED_Pow(2, length - i - 1) % 2 + '0', fontSize);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示二进制数字（二进制，正整数）
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定数字左上角的横坐标，范围：0~127
 * @param	y			:	指定数字左上角的纵坐标，范围：0~63
 * @param	number		:	指定要显示的数字，范围：-4294967295.0~4294967295.0
 * @param	intLength	:	指定数字的整数位长度，范围：0~10
 * @param	fraLength	:	指定数字的小数位长度，范围：0~9，小数进行四舍五入显示
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_float_num(OLEDDev_t *pDev, uint8_t x, uint8_t y, double number, uint8_t intLength, uint8_t fraLength, uint8_t fontSize)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint32_t PowNum, IntNum, FraNum;
	
	if (number >= 0)						// 数字大于等于0
	{
		__oled_show_char(pDev, x, y, '+', fontSize);	// 显示+号
	}
	else									// 数字小于0
	{
		__oled_show_char(pDev, x, y, '-', fontSize);	// 显示-号
		number = -number;					// Number取负
	}
	
	/* 提取整数部分和小数部分 */
	IntNum = number;						// 直接赋值给整型变量，提取整数
	number -= IntNum;						// 将Number的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	PowNum = OLED_Pow(10, fraLength);		// 根据指定小数的位数，确定乘数
	FraNum = round(number * PowNum);		// 将小数乘到整数，同时四舍五入，避免显示误差
	IntNum += FraNum / PowNum;				// 若四舍五入造成了进位，则需要再加给整数
	
	/* 显示整数部分 */
	__oled_show_num(pDev, x + fontSize, y, IntNum, intLength, fontSize);
	
	/* 显示小数点 */
	__oled_show_char(pDev, x + (intLength + 1) * fontSize, y, '.', fontSize);
	
	/* 显示小数部分 */
	__oled_show_num(pDev, x + (intLength + 2) * fontSize, y, FraNum, fraLength, fontSize);
	
	return 0;
}

/******************************************************************************
 * @brief	OLED显示汉字串
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定汉字串左上角的横坐标，范围：0~127
 * @param	y			:	指定汉字串左上角的纵坐标，范围：0~63
 * @param	Chinese		:	指定要显示的汉字串，范围：必须全部为汉字或者全角字符，不要加入任何半角字符
							显示的汉字需要在OLED_Data.c里的OLED_CF16x16数组定义
							未找到指定汉字时，会显示默认图形（一个方框，内部一个问号）
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_show_chinese(OLEDDev_t *pDev, uint8_t x, uint8_t y, char *Chinese)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t pChinese = 0;
	uint8_t pIndex;
	uint8_t i;
	char SingleChinese[OLED_CHN_CHAR_WIDTH + 1] = {0};
	
	for (i = 0; Chinese[i] != '\0'; i ++)		// 遍历汉字串
	{
		SingleChinese[pChinese] = Chinese[i];	// 提取汉字串数据到单个汉字数组
		pChinese ++;							// 计次自增
		
		/* 当提取次数到达OLED_CHN_CHAR_WIDTH时，即代表提取到了一个完整的汉字 */
		if (pChinese >= OLED_CHN_CHAR_WIDTH)
		{
			pChinese = 0;		// 计次归零
			
			/* 遍历整个汉字字模库，寻找匹配的汉字 */
			/* 如果找到最后一个汉字（定义为空字符串），则表示汉字未在字模库定义，停止寻找 */
			for (pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex ++)
			{
				/* 找到匹配的汉字 */
				if (strcmp(OLED_CF16x16[pIndex].Index, SingleChinese) == 0)
				{
					break;		// 跳出循环，此时pIndex的值为指定汉字的索引
				}
			}
			
			/* 将汉字字模库OLED_CF16x16的指定数据以16*16的图像格式显示 */
			__oled_show_image(pDev, x + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 16, y, 16, 16, OLED_CF16x16[pIndex].Data);
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED使用printf函数打印格式化字符串
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定格式化字符串左上角的横坐标，范围：0~127
 * @param	y			:	指定格式化字符串左上角的纵坐标，范围：0~63
 * @param	fontSize	:	指定字体大小
			范围	OLED_8X16	宽8像素，高16像素
					OLED_6X8	宽6像素，高8像素
 * @param	format		:	指定要显示的格式化字符串，范围：ASCII码可见字符组成的字符串
 * @param	...			:	格式化字符串参数列表
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_printf(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t fontSize, char *format, ...)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	char string[30];						// 定义字符数组
	va_list arg;							// 定义可变参数列表数据类型的变量arg
	va_start(arg, format);					// 从format开始，接收参数列表到arg变量
	vsprintf(string, format, arg);			// 使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);							// 结束变量arg
	__oled_show_string(pDev, x, y, string, fontSize);// OLED显示字符数组（字符串）
	
	return 0;
}

/******************************************************************************
 * @brief	OLED在指定位置画一个点
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	x		:	指定点的横坐标，范围：0~127
 * @param	y		:	指定点的纵坐标，范围：0~63
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_draw_point(OLEDDev_t *pDev, uint8_t x, uint8_t y)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	/* 参数检查，保证指定位置不会超出屏幕范围 */
	if (x > 127) {return -1;}
	if (y > 63) {return -1;}
	
	/* 将显存数组指定位置的一个Bit数据置1 */
	gOLEDDisplayBuf[pPrivData->index][y / 8][x] |= 0x01 << (y % 8);
	
	return 0;
}

/******************************************************************************
 * @brief	OLED获取指定位置点的值
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	x		:	指定点的横坐标，范围：0~127
 * @param	y		:	指定点的纵坐标，范围：0~63
 * @return	指定位置点是否处于点亮状态，1：点亮，0：熄灭
 ******************************************************************************/
static uint8_t __oled_get_point(OLEDDev_t *pDev, uint8_t x, uint8_t y)
{
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	/* 参数检查，保证指定位置不会超出屏幕范围 */
	if (x > 127) {return 0;}
	if (y > 63) {return 0;}
	
	/* 判断指定位置的数据 */
	if (gOLEDDisplayBuf[pPrivData->index][y / 8][x] & 0x01 << (y % 8))
	{
		return 1;	// 为1，返回1
	}
	return 0;		// 否则，返回0
}

/******************************************************************************
 * @brief	OLED画线
 * @param	pDev	:	OLEDDev_t结构体指针
 * @param	x0		:	指定一个端点的横坐标，范围：0~127
 * @param	y0		:	指定一个端点的纵坐标，范围：0~63
 * @param	x1		:	指定另一个端点的横坐标，范围：0~127
 * @param	y1		:	指定另一个端点的纵坐标，范围：0~63
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_draw_line(OLEDDev_t *pDev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	int16_t x, y, dx, dy, d, incrE, incrNE, temp;
	uint8_t yflag = 0, xyflag = 0;
	
	if (y0 == y1)		// 横线单独处理
	{
		/* 0号点X坐标大于1号点X坐标，则交换两点X坐标 */
		if (x0 > x1) {temp = x0; x0 = x1; x1 = temp;}
		
		/* 遍历X坐标 */
		for (x = x0; x <= x1; x ++)
		{
			__oled_draw_point(pDev, x, y0);	// 依次画点
		}
	}
	else if (x0 == x1)	// 竖线单独处理
	{
		/* 0号点Y坐标大于1号点Y坐标，则交换两点Y坐标 */
		if (y0 > y1) {temp = y0; y0 = y1; y1 = temp;}
		
		/* 遍历Y坐标 */
		for (y = y0; y <= y1; y ++)
		{
			__oled_draw_point(pDev, x0, y);	// 依次画点
		}
	}
	else				// 斜线
	{
		/* 使用Bresenham算法画直线，可以避免耗时的浮点运算，效率更高 */
		/* 参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf */
		/* 参考教程：https://www.bilibili.com/video/BV1364y1d7Lo */
		
		if (x0 > x1)	// 0号点X坐标大于1号点X坐标
		{
			/* 交换两点坐标 */
			/* 交换后不影响画线，但是画线方向由第一、二、三、四象限变为第一、四象限 */
			temp = x0; x0 = x1; x1 = temp;
			temp = y0; y0 = y1; y1 = temp;
		}
		
		if (y0 > y1)	// 0号点Y坐标大于1号点Y坐标
		{
			/* 将Y坐标取负 */
			/* 取负后影响画线，但是画线方向由第一、四象限变为第一象限 */
			y0 = -y0;
			y1 = -y1;
			
			/* 置标志位yflag，记住当前变换，在后续实际画线时，再将坐标换回来 */
			yflag = 1;
		}
		
		if (y1 - y0 > x1 - x0)	// 画线斜率大于1
		{
			/* 将X坐标与Y坐标互换 */
			/* 互换后影响画线，但是画线方向由第一象限0~90度范围变为第一象限0~45度范围 */
			temp = x0; x0 = y0; y0 = temp;
			temp = x1; x1 = y1; y1 = temp;
			
			/* 置标志位xyflag，记住当前变换，在后续实际画线时，再将坐标换回来 */
			xyflag = 1;
		}
		
		/* 以下为Bresenham算法画直线 */
		/* 算法要求，画线方向必须为第一象限0~45度范围 */
		dx = x1 - x0;
		dy = y1 - y0;
		incrE = 2 * dy;
		incrNE = 2 * (dy - dx);
		d = 2 * dy - dx;
		x = x0;
		y = y0;
		
		/* 画起始点，同时判断标志位，将坐标换回来 */
		if (yflag && xyflag){__oled_draw_point(pDev, y, -x);}
		else if (yflag)		{__oled_draw_point(pDev, x, -y);}
		else if (xyflag)	{__oled_draw_point(pDev, y, x);}
		else				{__oled_draw_point(pDev, x, y);}
		
		while (x < x1)		// 遍历X轴的每个点
		{
			x ++;
			if (d < 0)		// 下一个点在当前点东方
			{
				d += incrE;
			}
			else			// 下一个点在当前点东北方
			{
				y ++;
				d += incrNE;
			}
			
			/* 画每一个点，同时判断标志位，将坐标换回来 */
			if (yflag && xyflag){__oled_draw_point(pDev, y, -x);}
			else if (yflag)		{__oled_draw_point(pDev, x, -y);}
			else if (xyflag)	{__oled_draw_point(pDev, y, x);}
			else				{__oled_draw_point(pDev, x, y);}
		}	
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED矩形
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定矩形左上角的横坐标，范围：0~127
 * @param	y			:	指定矩形左上角的纵坐标，范围：0~63
 * @param	width		:	指定矩形的宽度，范围：0~128
 * @param	height		:	指定矩形的高度，范围：0~64
 * @param	isFilled	:	指定矩形是否填充
			范围		:	OLED_UNFILLED	不填充
							OLED_FILLED		填充
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_draw_rectangle(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t isFilled)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t i, j;
	if (!isFilled)		// 指定矩形不填充
	{
		/* 遍历上下x坐标，画矩形上下两条线 */
		for (i = x; i < x + width; i ++)
		{
			__oled_draw_point(pDev, i, y);
			__oled_draw_point(pDev, i, y + height - 1);
		}
		/* 遍历左右y坐标，画矩形左右两条线 */
		for (i = y; i < y + height; i ++)
		{
			__oled_draw_point(pDev, x, i);
			__oled_draw_point(pDev, x + width - 1, i);
		}
	}
	else				// 指定矩形填充
	{
		/* 遍历x坐标 */
		for (i = x; i < x + width; i ++)
		{
			/* 遍历y坐标 */
			for (j = y; j < y + height; j ++)
			{
				/* 在指定区域画点，填充满矩形 */
				__oled_draw_point(pDev, i, j);
			}
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED三角形
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x0			:	指定第一个端点的横坐标，范围：0~127
 * @param	y0			:	指定第一个端点的纵坐标，范围：0~63
 * @param	x1			:	指定第二个端点的横坐标，范围：0~127
 * @param	y1			:	指定第二个端点的纵坐标，范围：0~63
 * @param	x2			:	指定第三个端点的横坐标，范围：0~127
 * @param	y2			:	指定第三个端点的纵坐标，范围：0~63
 * @param	isFilled	:	指定三角形是否填充
			范围		:	OLED_UNFILLED	不填充
							OLED_FILLED		填充
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_draw_triangle(OLEDDev_t *pDev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t IsFilled)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	uint8_t minx = x0, miny = y0, maxx = x0, maxy = y0;
	uint8_t i, j;
	int16_t vx[] = {x0, x1, x2};
	int16_t vy[] = {y0, y1, y2};
	
	if (!IsFilled)			// 指定三角形不填充
	{
		/* 调用画线函数，将三个点用直线连接 */
		__oled_draw_line(pDev, x0, y0, x1, y1);
		__oled_draw_line(pDev, x0, y0, x2, y2);
		__oled_draw_line(pDev, x1, y1, x2, y2);
	}
	else					// 指定三角形填充
	{
		/* 找到三个点最小的X、Y坐标 */
		if (x1 < minx) {minx = x1;}
		if (x2 < minx) {minx = x2;}
		if (y1 < miny) {miny = y1;}
		if (y2 < miny) {miny = y2;}
		
		/* 找到三个点最大的X、Y坐标 */
		if (x1 > maxx) {maxx = x1;}
		if (x2 > maxx) {maxx = x2;}
		if (y1 > maxy) {maxy = y1;}
		if (y2 > maxy) {maxy = y2;}
		
		/* 最小最大坐标之间的矩形为可能需要填充的区域 */
		/* 遍历此区域中所有的点 */
		/* 遍历X坐标 */		
		for (i = minx; i <= maxx; i ++)
		{
			/* 遍历Y坐标 */	
			for (j = miny; j <= maxy; j ++)
			{
				/* 调用OLED_pnpoly，判断指定点是否在指定三角形之中 */
				/* 如果在，则画点，如果不在，则不做处理 */
				if (OLED_pnpoly(3, vx, vy, i, j)) {__oled_draw_point(pDev, i, j);}
			}
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED画圆
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定圆的圆心横坐标，范围：0~127
 * @param	y			:	指定圆的圆心纵坐标，范围：0~63
 * @param	radius		:	指定圆的半径，范围：0~255
 * @param	isFilled	:	指定圆是否填充
			范围		:	OLED_UNFILLED	不填充
							OLED_FILLED		填充
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_draw_circle(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t radius, uint8_t IsFilled)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	int16_t x1, y1, d, j;
	
	/* 使用Bresenham算法画圆，可以避免耗时的浮点运算，效率更高 */
	/* 参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf */
	/* 参考教程：https://www.bilibili.com/video/BV1VM4y1u7wJ */
	
	d = 1 - radius;
	x1 = 0;
	y1 = radius;
	
	/* 画每个八分之一圆弧的起始点 */
	__oled_draw_point(pDev, x + x1, y + y1);
	__oled_draw_point(pDev, x - x1, y - y1);
	__oled_draw_point(pDev, x + y1, y + x1);
	__oled_draw_point(pDev, x - y1, y - x1);
	
	if (IsFilled)		// 指定圆填充
	{
		/* 遍历起始点Y坐标 */
		for (j = -y1; j < y1; j ++)
		{
			/* 在指定区域画点，填充部分圆 */
			__oled_draw_point(pDev, x, y + j);
		}
	}
	
	while (x1 < y1)		// 遍历X轴的每个点
	{
		x1 ++;
		if (d < 0)		// 下一个点在当前点东方
		{
			d += 2 * x1 + 1;
		}
		else			// 下一个点在当前点东南方
		{
			y1 --;
			d += 2 * (x1 - y1) + 1;
		}
		
		/* 画每个八分之一圆弧的点 */
		__oled_draw_point(pDev, x + x1, y + y1);
		__oled_draw_point(pDev, x + y1, y + x1);
		__oled_draw_point(pDev, x - x1, y - y1);
		__oled_draw_point(pDev, x - y1, y - x1);
		__oled_draw_point(pDev, x + x1, y - y1);
		__oled_draw_point(pDev, x + y1, y - x1);
		__oled_draw_point(pDev, x - x1, y + y1);
		__oled_draw_point(pDev, x - y1, y + x1);
		
		if (IsFilled)	// 指定圆填充
		{
			/* 遍历中间部分 */
			for (j = -y1; j < y1; j ++)
			{
				/* 在指定区域画点，填充部分圆 */
				__oled_draw_point(pDev, x + x1, y + j);
				__oled_draw_point(pDev, x - x1, y + j);
			}
			
			/* 遍历两侧部分 */
			for (j = -x1; j < x1; j ++)
			{
				/* 在指定区域画点，填充部分圆 */
				__oled_draw_point(pDev, x - y1, y + j);
				__oled_draw_point(pDev, x + y1, y + j);
			}
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED画椭圆
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定椭圆的圆心横坐标，范围：0~127
 * @param	y			:	指定椭圆的圆心纵坐标，范围：0~63
 * @param	a			:	指定椭圆的横向半轴长度，范围：0~255
 * @param	b			:	指定椭圆的纵向半轴长度，范围：0~255
 * @param	isFilled	:	指定椭圆是否填充
			范围		:	OLED_UNFILLED	不填充
							OLED_FILLED		填充
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_draw_ellipse(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t isFilled)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	int16_t x1, y1, j;
	float d1, d2;
	
	/* 使用Bresenham算法画椭圆，可以避免部分耗时的浮点运算，效率更高 */
	/* 参考链接：https://blog.csdn.net/myf_666/article/details/128167392 */
	
	x1 = 0;
	y1 = b;
	d1 = b * b + a * a * (-b + 0.5);
	
	if (isFilled)	// 指定椭圆填充
	{
		/* 遍历起始点Y坐标 */
		for (j = -y1; j < y1; j ++)
		{
			/* 在指定区域画点，填充部分椭圆 */
			__oled_draw_point(pDev, x, y + j);
			__oled_draw_point(pDev, x, y + j);
		}
	}
	
	/* 画椭圆弧的起始点 */
	__oled_draw_point(pDev, x + x1, y + y1);
	__oled_draw_point(pDev, x - x1, y - y1);
	__oled_draw_point(pDev, x - x1, y + y1);
	__oled_draw_point(pDev, x + x1, y - y1);
	
	/* 画椭圆中间部分 */
	while (b * b * (x1 + 1) < a * a * (y1 - 0.5))
	{
		if (d1 <= 0)		// 下一个点在当前点东方
		{
			d1 += b * b * (2 * x1 + 3);
		}
		else				// 下一个点在当前点东南方
		{
			d1 += b * b * (2 * x1 + 3) + a * a * (-2 * y1 + 2);
			y1 --;
		}
		x1 ++;
		
		if (isFilled)	// 指定椭圆填充
		{
			/* 遍历中间部分 */
			for (j = -y1; j < y1; j ++)
			{
				/* 在指定区域画点，填充部分椭圆 */
				__oled_draw_point(pDev, x + x1, y + j);
				__oled_draw_point(pDev, x - x1, y + j);
			}
		}
		
		/* 画椭圆中间部分圆弧 */
		__oled_draw_point(pDev, x + x1, y + y1);
		__oled_draw_point(pDev, x - x1, y - y1);
		__oled_draw_point(pDev, x - x1, y + y1);
		__oled_draw_point(pDev, x + x1, y - y1);
	}                               
	
	/* 画椭圆两侧部分*/
	d2 = b * b * (x1 + 0.5) * (x1 + 0.5) + a * a * (y1 - 1) * (y1 - 1) - a * a * b * b;
	
	while (y1 > 0)
	{
		if (d2 <= 0)		// 下一个点在当前点东方
		{
			d2 += b * b * (2 * x1 + 2) + a * a * (-2 * y1 + 3);
			x1 ++;
			
		}
		else				// 下一个点在当前点东南方
		{
			d2 += a * a * (-2 * y1 + 3);
		}
		y1 --;
		
		if (isFilled)	// 指定椭圆填充
		{
			/* 遍历两侧部分 */
			for (j = -y1; j < y1; j ++)
			{
				/* 在指定区域画点，填充部分椭圆 */
				__oled_draw_point(pDev, x + x1, y + j);
				__oled_draw_point(pDev, x - x1, y + j);
			}
		}
		
		/* 画椭圆两侧部分圆弧 */
		__oled_draw_point(pDev, x + x1, y + y1);
		__oled_draw_point(pDev, x - x1, y - y1);
		__oled_draw_point(pDev, x - x1, y + y1);
		__oled_draw_point(pDev, x + x1, y - y1);
	}
	
	return 0;
}

/******************************************************************************
 * @brief	OLED画圆弧
 * @param	pDev		:	OLEDDev_t结构体指针
 * @param	x			:	指定圆弧的圆心横坐标，范围：0~127
 * @param	y			:	指定圆弧的圆心纵坐标，范围：0~63
 * @param	radius		:	指定圆弧的半径，范围：0~255
 * @param	startAngle	:	指定圆弧的起始角度，范围：-180~180
			水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
 * @param	endAngle	:	指定圆弧的终止角度，范围：-180~180
			水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
 * @param	isFilled	:	指定圆弧是否填充，填充后为扇形
			范围		:	OLED_UNFILLED	不填充
							OLED_FILLED		填充
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_draw_arc(OLEDDev_t *pDev, uint8_t x, uint8_t y, uint8_t radius, int16_t startAngle, int16_t endAngle, uint8_t isFilled)
{	
	if (!pDev || !pDev->initFlag)
		return -1;
	
	int16_t x1, y1, d, j;
	
	/* 此函数借用Bresenham算法画圆的方法 */
	
	d = 1 - radius;
	x1 = 0;
	y1 = radius;
	
	/* 在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
	if (OLED_IsInAngle(x1, y1, startAngle, endAngle))	{__oled_draw_point(pDev, x + x1, y + y1);}
	if (OLED_IsInAngle(-x1, -y1, startAngle, endAngle)) {__oled_draw_point(pDev, x - x1, y - y1);}
	if (OLED_IsInAngle(y1, x1, startAngle, endAngle)) {__oled_draw_point(pDev, x + y1, y + x1);}
	if (OLED_IsInAngle(-y1, -x1, startAngle, endAngle)) {__oled_draw_point(pDev, x - y1, y - x1);}
	
	if (isFilled)	// 指定圆弧填充
	{
		/* 遍历起始点Y坐标 */
		for (j = -y1; j < y1; j ++)
		{
			/* 在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
			if (OLED_IsInAngle(0, j, startAngle, endAngle)) {__oled_draw_point(pDev, x, y + j);}
		}
	}
	
	while (x1 < y1)		// 遍历X轴的每个点
	{
		x1 ++;
		if (d < 0)		// 下一个点在当前点东方
		{
			d += 2 * x1 + 1;
		}
		else			// 下一个点在当前点东南方
		{
			y1 --;
			d += 2 * (x1 - y1) + 1;
		}
		
		/* 在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
		if (OLED_IsInAngle(x1, y1, startAngle, endAngle)) {__oled_draw_point(pDev, x + x1, y + y1);}
		if (OLED_IsInAngle(y1, x1, startAngle, endAngle)) {__oled_draw_point(pDev, x + y1, y + x1);}
		if (OLED_IsInAngle(-x1, -y1, startAngle, endAngle)) {__oled_draw_point(pDev, x - x1, y - y1);}
		if (OLED_IsInAngle(-y1, -x1, startAngle, endAngle)) {__oled_draw_point(pDev, x - y1, y - x1);}
		if (OLED_IsInAngle(x1, -y1, startAngle, endAngle)) {__oled_draw_point(pDev, x + x1, y - y1);}
		if (OLED_IsInAngle(y1, -x1, startAngle, endAngle)) {__oled_draw_point(pDev, x + y1, y - x1);}
		if (OLED_IsInAngle(-x1, y1, startAngle, endAngle)) {__oled_draw_point(pDev, x - x1, y + y1);}
		if (OLED_IsInAngle(-y1, x1, startAngle, endAngle)) {__oled_draw_point(pDev, x - y1, y + x1);}
		
		if (isFilled)	// 指定圆弧填充
		{
			/* 遍历中间部分 */
			for (j = -y1; j < y1; j ++)
			{
				/* 在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
				if (OLED_IsInAngle(x1, j, startAngle, endAngle)) {__oled_draw_point(pDev, x + x1, y + j);}
				if (OLED_IsInAngle(-x1, j, startAngle, endAngle)) {__oled_draw_point(pDev, x - x1, y + j);}
			}
			
			/*遍历两侧部分*/
			for (j = -x1; j < x1; j ++)
			{
				/* 在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
				if (OLED_IsInAngle(-y1, j, startAngle, endAngle)) {__oled_draw_point(pDev, x - y1, y + j);}
				if (OLED_IsInAngle(y1, j, startAngle, endAngle)) {__oled_draw_point(pDev, x + y1, y + j);}
			}
		}
	}
	
	return 0;
}

/******************************************************************************
 * @brief	去初始化OLED
 * @param	pDev   :  OLEDDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __oled_deinit(OLEDDev_t *pDev)
{  
	if (!pDev || !pDev->initFlag)
		return -1;
	
	OLEDPrivData_t *pPrivData = (OLEDPrivData_t *)pDev->pPrivData;
	
	/* 关闭时钟 */
	__oled_config_gpio_clock_disable(pDev->info.RESPort);
	__oled_config_gpio_clock_disable(pDev->info.DCPort);
	__oled_config_dma_clock_disable(pDev->info.spix);
	
	/* 复位GPIO */
	__oled_gpio_deinit(pDev->info.RESPort);
	__oled_gpio_deinit(pDev->info.DCPort);
	
	/* 去初始化硬件SPI */
	pPrivData->oled.deinit(&pPrivData->oled);
	
	/* 释放私有数据内存 */
	free(pDev->pPrivData);
	pDev->pPrivData = NULL;
	
	pDev->initFlag = false;	// 修改初始化标志
	return 0;
}

/**********************************************************************功能函数*/