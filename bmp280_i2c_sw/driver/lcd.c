#include "delay.h"
#include "lcd.h"
#include "lcd_data.h"

#if defined(STM32F40_41xxx)

#define	__lcd_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												}

#define	__lcd_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define	__lcd_config_io_af_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
												GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; \
												GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
												GPIO_InitStructure.GPIO_Pin = pin; \
												GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
												GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; \
												GPIO_Init(port, &GPIO_InitStructure); \
											}

#define __lcd_get_gpio_pin_sourse(pin)	(	pin == GPIO_Pin_0 ? GPIO_PinSource0 : \
											pin == GPIO_Pin_1 ? GPIO_PinSource1 : \
											pin == GPIO_Pin_2 ? GPIO_PinSource2 : \
											pin == GPIO_Pin_3 ? GPIO_PinSource3 : \
											pin == GPIO_Pin_4 ? GPIO_PinSource4 : \
											pin == GPIO_Pin_5 ? GPIO_PinSource5 : \
											pin == GPIO_Pin_6 ? GPIO_PinSource6 : \
											pin == GPIO_Pin_7 ? GPIO_PinSource7 : \
											pin == GPIO_Pin_8 ? GPIO_PinSource8 : \
											pin == GPIO_Pin_9 ? GPIO_PinSource9 : \
											pin == GPIO_Pin_10 ? GPIO_PinSource10 : \
											pin == GPIO_Pin_11 ? GPIO_PinSource11 : \
											pin == GPIO_Pin_12 ? GPIO_PinSource12 : \
											pin == GPIO_Pin_13 ? GPIO_PinSource13 : \
											pin == GPIO_Pin_14 ? GPIO_PinSource14 : \
											pin == GPIO_Pin_15 ? GPIO_PinSource15 : \
											(int) 0	)

#define __lcd_io_set(port, pin)				port->BSRRL = pin

#define __lcd_io_reset(port, pin)			port->BSRRH = pin

#endif

/* LCD数据端口与引脚数组 */
LCDGPIOPort_t gLCDDataPorts[] = {	GPIOD, GPIOD, GPIOD, GPIOD, 
                                	GPIOE, GPIOE, GPIOE, GPIOE,
                                	GPIOE, GPIOE, GPIOE, GPIOE, 
									GPIOE, GPIOD, GPIOD, GPIOD	};

uint32_t gLCDDataPins[] = {	GPIO_Pin_14, GPIO_Pin_15, GPIO_Pin_0, GPIO_Pin_1, 
                            GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10,
                            GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, 
							GPIO_Pin_15, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10	};
							
/* LCD地址结构体 */
typedef struct {
	uint16_t reg;
	uint16_t ram;
}LCDAddr_t;

#define LCD_BASE	((uint32_t)(0x6001FFFE))
#define LCD_ADDR	((LCDAddr_t *)LCD_BASE)

/* LCD私有数据结构体 */
typedef struct {
	LCDGPIOPort_t	bl_port;	// 背光端口
	uint32_t		bl_pin;		// 背光引脚
	LCDGPIOPort_t	cs_port;	// 片选端口
	uint32_t		cs_pin;		// 片选引脚
	LCDGPIOPort_t	rs_port;	// 1数据/0命令端口
	uint32_t		rs_pin;		// 1数据/0命令引脚
	LCDGPIOPort_t	wr_port;	// 写信号端口
	uint32_t		wr_pin;		// 写信号引脚
	LCDGPIOPort_t	rd_port;	// 读信号端口
	uint32_t		rd_pin;		// 读信号引脚
	uint16_t		id;			// ID
	uint8_t			dir;		// 横屏还是竖屏控制：0竖屏/1横屏
	uint16_t		wramcmd;	// 开始写gram指令
	uint16_t		setxcmd;	// 设置x坐标指令
	uint16_t		setycmd;	// 设置y坐标指令
}LCDPrivData_t;

/* 函数声明 */
static void __lcd_clear(LCDDev_t *dev, uint16_t color);
static void __lcd_fill(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
static void __lcd_color_fill(LCDDev_t *dev, uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color);
static void __lcd_show_char(LCDDev_t *dev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_string(LCDDev_t *dev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_num(LCDDev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_hex_num(LCDDev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_float_num(LCDDev_t *dev, uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_chinese(LCDDev_t *dev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode);
static void __lcd_show_image(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[]);
static void __lcd_draw_point(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t color);
static uint16_t __lcd_read_point(LCDDev_t *dev, uint16_t x, uint16_t y);
static void __lcd_draw_line(LCDDev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
static void __lcd_draw_rectangle(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
static void __lcd_draw_circle(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t r, uint16_t color);
static int __lcd_deinit(LCDDev_t *dev);

/*读写接口函数******************************************************************/

/******************************************************************************
 * @brief	LCD写数据
 * @param	data	:  要写入的16位数据
 * @return	无
 ******************************************************************************/
static void __lcd_write_data(uint16_t data)
{
	data = data;			// 使用-O2优化的时候，必须插入的延时
	LCD_ADDR->ram = data;	
}

/******************************************************************************
 * @brief	LCD写命令
 * @param	regno	:  要写入的16位命令
 * @return	无
 ******************************************************************************/
static void __lcd_write_cmd(volatile uint16_t regno)
{
	regno = regno;			// 使用-O2优化的时候，必须插入的延时
	LCD_ADDR->reg = regno;	// 写入要写的寄存器序号	
}

/******************************************************************************
 * @brief	LCD寄存器写值
 * @param	regno	:  要写入的16位命令
 * @param	data	:  要写入的16位数据
 * @return	无
 ******************************************************************************/
static void __lcd_write_reg(uint16_t regno, uint16_t data)
{
	__lcd_write_cmd(regno);
	__lcd_write_data(data);
}

/******************************************************************************
 * @brief	LCD读数据
 * @param	无
 * @return	读到的16位数据
 ******************************************************************************/
static uint16_t __lcd_read_data(void)
{
	volatile uint16_t ram;

	ram = LCD_ADDR->ram;

	return ram;
}

/******************************************************************读写接口函数*/

/******************************************************************************
 * @brief	初始化LCD
 * @param	dev	:  LCDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int lcd_init(LCDDev_t *dev)
{
	if (!dev)
		return -1;

	uint16_t reg_val;
	
	/* 初始化私有数据 */
	dev->priv_data = (LCDPrivData_t *)malloc(sizeof(LCDPrivData_t));
	if (!dev->priv_data)
		return -1;
	
	LCDPrivData_t *priv_data = (LCDPrivData_t *)dev->priv_data;

	priv_data->dir = LCD_DIRECTION;
	priv_data->wramcmd = 0X2C;
	priv_data->setxcmd = 0X2A;
	priv_data->setycmd = 0X2B;

	priv_data->bl_port = GPIOA;
	priv_data->bl_pin = GPIO_Pin_15;

	priv_data->cs_port = GPIOD;
	priv_data->cs_pin = GPIO_Pin_7;

	priv_data->rs_port = GPIOD;
	priv_data->rs_pin = GPIO_Pin_11;

	priv_data->wr_port = GPIOD;
	priv_data->wr_pin = GPIO_Pin_5;

	priv_data->rd_port = GPIOD;
	priv_data->rd_pin = GPIO_Pin_4;

	/* 开启时钟 */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);
	__lcd_config_gpio_clock_enable(priv_data->cs_port);
	__lcd_config_gpio_clock_enable(priv_data->rs_port);
	__lcd_config_gpio_clock_enable(priv_data->wr_port);
	__lcd_config_gpio_clock_enable(priv_data->rd_port);
	__lcd_config_gpio_clock_enable(priv_data->bl_port);
	for (uint8_t i = 0; i < 16; i++)
		__lcd_config_gpio_clock_enable(gLCDDataPorts[i]);

	/* 配置为复用推挽输出 */
	__lcd_config_io_af_pp(priv_data->cs_port, priv_data->cs_pin);
	__lcd_config_io_af_pp(priv_data->rs_port, priv_data->rs_pin);
	__lcd_config_io_af_pp(priv_data->wr_port, priv_data->wr_pin);
	__lcd_config_io_af_pp(priv_data->rd_port, priv_data->rd_pin);
	__lcd_config_io_out_pp(priv_data->bl_port, priv_data->bl_pin);
	for (uint8_t i = 0; i < 16; i++)
		__lcd_config_io_af_pp(gLCDDataPorts[i], gLCDDataPins[i]);

	GPIO_PinAFConfig(priv_data->cs_port, __lcd_get_gpio_pin_sourse(priv_data->cs_pin), GPIO_AF_FSMC);
	GPIO_PinAFConfig(priv_data->rs_port, __lcd_get_gpio_pin_sourse(priv_data->rs_pin), GPIO_AF_FSMC);
	GPIO_PinAFConfig(priv_data->wr_port, __lcd_get_gpio_pin_sourse(priv_data->wr_pin), GPIO_AF_FSMC);
	GPIO_PinAFConfig(priv_data->rd_port, __lcd_get_gpio_pin_sourse(priv_data->rd_pin), GPIO_AF_FSMC);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);

	/* 默认输出高电平 */
	__lcd_io_set(priv_data->wr_port, priv_data->wr_pin);
	__lcd_io_set(priv_data->rd_port, priv_data->rd_pin);
	__lcd_io_set(priv_data->cs_port, priv_data->cs_pin);
	__lcd_io_set(priv_data->rs_port, priv_data->rs_pin);
	for (uint8_t i = 0; i < 16; i++)
		__lcd_io_set(gLCDDataPorts[i], gLCDDataPins[i]);

	/* 读写时序 */
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
	readWriteTiming.FSMC_AddressSetupTime = 0XF;			// 地址建立时间（ADDSET）为16个HCLK 1/168M=6ns*16=96ns
	readWriteTiming.FSMC_AddressHoldTime = 0x00;			// 地址保持时间（ADDHLD）模式A未用到
	readWriteTiming.FSMC_DataSetupTime = 60;				// 数据保存时间为60个HCLK	=6*60=360ns
	readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
	readWriteTiming.FSMC_CLKDivision = 0x00;
	readWriteTiming.FSMC_DataLatency = 0x00;
	readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;	// 模式A
    
	/* 写时序 */
	FSMC_NORSRAMTimingInitTypeDef  writeTiming;
	writeTiming.FSMC_AddressSetupTime = 9;					// 地址建立时间（ADDSET）为9个HCLK =54ns
	writeTiming.FSMC_AddressHoldTime = 0x00;				// 地址保持时间
	writeTiming.FSMC_DataSetupTime = 8;						// 数据保存时间为6ns*9个HCLK=54ns
	writeTiming.FSMC_BusTurnAroundDuration = 0x00;
	writeTiming.FSMC_CLKDivision = 0x00;
	writeTiming.FSMC_DataLatency = 0x00;
	writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;		// 模式A

	/* 初始化FSMC */
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;									// NE1，对应BTCR[6],[7]。
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;				// 不复用数据地址
	FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;							// SRAM   
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;					// 存储器数据宽度为16bit   
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;					// 存储器写使能
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;						// 读写使用不同的时序
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;					// 读写时序
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &writeTiming;							// 写时序

	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  // 初始化FSMC配置

	/* 使能BANK1 */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
	
 	delay_ms(50);
	
	/* 读取ID：0x9341 */
	__lcd_write_cmd(0xD3);
	priv_data->id = __lcd_read_data();
	priv_data->id = __lcd_read_data();	// 0x0000
	priv_data->id = __lcd_read_data();	// 0x0093
	priv_data->id <<= 8;				// 0x9300
	priv_data->id |= __lcd_read_data();	// 0x9341

	/* 设置WR时序为最快 */
	if (priv_data->id == 0x9341)
	{
		/* 重新配置写时序控制寄存器的时序 */  	 							    
		FSMC_Bank1E->BWTR[6] &= ~(0XF << 0);	// 地址建立时间(ADDSET)清零 	 
		FSMC_Bank1E->BWTR[6] &= ~(0XF << 8);	// 数据保存时间清零
		FSMC_Bank1E->BWTR[6] |= 3 << 0;			// 地址建立时间(ADDSET)为3个HCLK = 18ns  	 
        if (priv_data->id == 0X7789)
        {
            FSMC_Bank1E->BWTR[6] |= 3 << 8; 	// 数据保存时间(DATAST)为6ns*3个HCLK = 18ns
        }
        else
        {
            FSMC_Bank1E->BWTR[6] |= 2 << 8; 	// 数据保存时间(DATAST)为6ns*3个HCLK = 18ns
        }
	}

	/* 完成初始化序列 */
	if (priv_data->id == 0x9341)
	{
		__lcd_write_cmd(0xCF);  
		__lcd_write_data(0x00); 
		__lcd_write_data(0xC1); 
		__lcd_write_data(0X30); 
		__lcd_write_cmd(0xED);  
		__lcd_write_data(0x64); 
		__lcd_write_data(0x03); 
		__lcd_write_data(0X12); 
		__lcd_write_data(0X81); 
		__lcd_write_cmd(0xE8);  
		__lcd_write_data(0x85); 
		__lcd_write_data(0x10); 
		__lcd_write_data(0x7A); 
		__lcd_write_cmd(0xCB);  
		__lcd_write_data(0x39); 
		__lcd_write_data(0x2C); 
		__lcd_write_data(0x00); 
		__lcd_write_data(0x34); 
		__lcd_write_data(0x02); 
		__lcd_write_cmd(0xF7);  
		__lcd_write_data(0x20); 
		__lcd_write_cmd(0xEA);  
		__lcd_write_data(0x00); 
		__lcd_write_data(0x00); 
		__lcd_write_cmd(0xC0);		// Power control
		__lcd_write_data(0x1B);		// VRH[5:0]
		__lcd_write_cmd(0xC1);		// Power control
		__lcd_write_data(0x01);		// SAP[2:0];BT[3:0]
		__lcd_write_cmd(0xC5);		// VCM control
		__lcd_write_data(0x30);		// 3F
		__lcd_write_data(0x30);		// 3C
		__lcd_write_cmd(0xC7);		// VCM control2
		__lcd_write_data(0XB7); 
		__lcd_write_cmd(0x36);		// Memory Access Control
		__lcd_write_data(0x48); 
		__lcd_write_cmd(0x3A);   
		__lcd_write_data(0x55); 
		__lcd_write_cmd(0xB1);   
		__lcd_write_data(0x00);   
		__lcd_write_data(0x1A); 
		__lcd_write_cmd(0xB6);		// Display Function Control
		__lcd_write_data(0x0A); 
		__lcd_write_data(0xA2); 
		__lcd_write_cmd(0xF2);		// 3Gamma Function Disable
		__lcd_write_data(0x00); 
		__lcd_write_cmd(0x26);		// Gamma curve selected
		__lcd_write_data(0x01); 
		__lcd_write_cmd(0xE0);		// Set Gamma
		__lcd_write_data(0x0F); 
		__lcd_write_data(0x2A); 
		__lcd_write_data(0x28); 
		__lcd_write_data(0x08); 
		__lcd_write_data(0x0E); 
		__lcd_write_data(0x08); 
		__lcd_write_data(0x54); 
		__lcd_write_data(0XA9); 
		__lcd_write_data(0x43); 
		__lcd_write_data(0x0A); 
		__lcd_write_data(0x0F); 
		__lcd_write_data(0x00); 
		__lcd_write_data(0x00); 
		__lcd_write_data(0x00); 
		__lcd_write_data(0x00); 		 
		__lcd_write_cmd(0XE1);		// Set Gamma
		__lcd_write_data(0x00); 
		__lcd_write_data(0x15); 
		__lcd_write_data(0x17); 
		__lcd_write_data(0x07); 
		__lcd_write_data(0x11); 
		__lcd_write_data(0x06); 
		__lcd_write_data(0x2B); 
		__lcd_write_data(0x56); 
		__lcd_write_data(0x3C); 
		__lcd_write_data(0x05); 
		__lcd_write_data(0x10); 
		__lcd_write_data(0x0F); 
		__lcd_write_data(0x3F); 
		__lcd_write_data(0x3F); 
		__lcd_write_data(0x0F); 
		__lcd_write_cmd(0x2B); 
		__lcd_write_data(0x00);
		__lcd_write_data(0x00);
		__lcd_write_data(0x01);
		__lcd_write_data(0x3f);
		__lcd_write_cmd(0x2A); 
		__lcd_write_data(0x00);
		__lcd_write_data(0x00);
		__lcd_write_data(0x00);
		__lcd_write_data(0xef);	 
		__lcd_write_cmd(0x11);		// Exit Sleep
		delay_ms(120);
		__lcd_write_cmd(0x29);		// display on
	}

	/* 设置列地址的x坐标的EC初始值 */
	__lcd_write_cmd(priv_data->setxcmd);
	__lcd_write_data(0);									// SC高8位 = 0
	__lcd_write_data(0);									// SC低8位 = 0
	__lcd_write_data((dev->width - 1) >> 8);			// EC高8位 = 宽度-1的高8位
	__lcd_write_data((dev->width - 1) & 0xFF);		// EC低8位 = 宽度-1的低8位

	/* 设置页地址的y坐标的EP初始值 */
	__lcd_write_cmd(priv_data->setycmd);
	__lcd_write_data(0);									// SP高8位 = 0
	__lcd_write_data(0);									// SP低8位 = 0
	__lcd_write_data((dev->height - 1) >> 8);			// EP高8位 = 高度-1的高8位
	__lcd_write_data((dev->height - 1) & 0xFF);		// EP低8位 = 高度-1的低8位

	/* 设置屏幕方向 */
	if (priv_data->dir == 0)	// 竖屏
    {
        dev->width = 240;
        dev->height = 320;
    }
    else						// 横屏
    {
        dev->width = 320;
        dev->height = 240;
    }

	/* 设置扫描方向 */
	if (priv_data->dir == 0)	// 竖屏
    {
        __lcd_write_reg(0x36, 1 << 3);
    }
    else						// 横屏
    {
        reg_val |= (1 << 7) | (0 << 6) | (1 << 5);
		reg_val |= 0X08;
		__lcd_write_reg(0x36, reg_val);
    }

	/* 点亮背光 */
	__lcd_io_set(priv_data->bl_port, priv_data->bl_pin);		// BL引脚置高电平

	/* 清屏 */
	__lcd_clear(dev, 0xFFFF);	// 白屏

	/* 函数指针赋值 */
	dev->clear = __lcd_clear;
	dev->fill = __lcd_fill;
	dev->color_fill = __lcd_color_fill;
	dev->show_char = __lcd_show_char;
	dev->show_string = __lcd_show_string;
	dev->show_num = __lcd_show_num;
	dev->show_hex_num = __lcd_show_hex_num;
	dev->show_float_num = __lcd_show_float_num;
	dev->show_chinese = __lcd_show_chinese;
	dev->show_image = __lcd_show_image;
	dev->draw_point = __lcd_draw_point;
	dev->read_point = __lcd_read_point;
	dev->draw_line = __lcd_draw_line;
	dev->draw_rectangle = __lcd_draw_rectangle;
	dev->draw_circle = __lcd_draw_circle;
	dev->deinit = __lcd_deinit;
	
	dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	LCD准备写GRAM
 * @param	dev	:  LCDDev_t 结构体指针
 * @return	无
 ******************************************************************************/
static void __lcd_write_ram_prepare(LCDDev_t *dev)
{
	LCDPrivData_t *priv_data = (LCDPrivData_t *)dev->priv_data;
	
	LCD_ADDR->reg = priv_data->wramcmd;
}

/******************************************************************************
 * @brief	LCD设置光标位置
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @return	无
 ******************************************************************************/
static void __lcd_set_cursor(LCDDev_t *dev, uint16_t x, uint16_t y)
{
	LCDPrivData_t *priv_data = (LCDPrivData_t *)dev->priv_data;

	__lcd_write_cmd(priv_data->setxcmd);
	__lcd_write_data(x >> 8);
	__lcd_write_data(x & 0xFF);

	__lcd_write_cmd(priv_data->setycmd);
	__lcd_write_data(y >> 8);
	__lcd_write_data(y & 0xFF);
}

/******************************************************************************
 * @brief	LCD清屏
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	color	:  清屏的填充颜色
 * @return	无
 ******************************************************************************/
static void __lcd_clear(LCDDev_t *dev, uint16_t color)
{
	uint32_t index = 0;
    uint32_t totalpoint = dev->width;
    totalpoint *= dev->height;    	// 得到总点数

	__lcd_set_cursor(dev, 0x0000, 0x0000);	// 设置光标位置
    __lcd_write_ram_prepare(dev);			// 准备写GRAM

	/* 连续写入totalpoint个点 */
    for (index = 0; index < totalpoint; index++)
    {
        LCD_ADDR->ram = color;
    }
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @param	width	:  填充宽度
 * @param	height	:  填充高度
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_fill(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    uint16_t i, j;

    // 设置起始和结束的坐标
    uint16_t x_end = x + width - 1;
    uint16_t y_end = y + height - 1;

    // 填充矩形区域
    for (i = y; i <= y_end; i++)
    {
        __lcd_set_cursor(dev, x, i);	// 设置光标位置
        __lcd_write_ram_prepare(dev);	// 开始写入GRAM

        for (j = x; j <= x_end; j++)
        {
            LCD_ADDR->ram = color;
        }
    }
}

/******************************************************************************
 * @brief	LCD在指定区域填充颜色块，颜色可以不同
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @param	width	:  填充宽度
 * @param	height	:  填充高度
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_color_fill(LCDDev_t *dev, uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t *color)
{
    uint16_t height, width;
    uint16_t i, j;
    width = ex - sx + 1;            //得到填充的宽度
    height = ey - sy + 1;           //高度

    for (i = 0; i < height; i++)
    {
        __lcd_set_cursor(dev, sx, sy + i);	//设置光标位置
        __lcd_write_ram_prepare(dev);		//开始写入GRAM

        for (j = 0; j < width; j++)
        {
            LCD_ADDR->ram=color[i * width + j];  //写入数据
        }
    }
}

/******************************************************************************
 * @brief	LCD显示单个字符
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	chr		:	要显示的字符
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_char(LCDDev_t *dev, uint16_t x, uint16_t y, uint8_t chr, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	uint8_t temp, i, j;
	uint16_t y0 = y;		// 记录y的初始坐标
	uint8_t csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);		// 得到字体一个字符对应点阵集所占的字节数	
 	chr = chr - ' ';	// 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
	for(i = 0; i < csize; i++)
	{   
		if(size == 12)temp = LCD_F6x12[chr][i]; 	 	// 调用1206字体
		else if(size == 16)temp = LCD_F8x16[chr][i];	// 调用1608字体
		else if(size == 24)temp = LCD_F12x24[chr][i];	// 调用2412字体
		else if(size == 32)temp = LCD_F16x32[chr][i];	// 调用3216字体
		else return;									// 没有的字库
		for(j = 0; j < 8; j++)
		{			    
			if(temp & 0x80)__lcd_draw_point(dev, x, y, fc);
			else if(mode == 0)__lcd_draw_point(dev, x, y, bc);
			temp <<= 1;
			y++;
			if(y >= dev->height)	return;		// 超区域了
			if((y - y0) == size)
			{
				y = y0;
				x++;
				if(x >= dev->width)	return;	// 超区域了
				break;
			}
		}  	 
	}
}

/******************************************************************************
 * @brief	LCD显示字符串
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	str		:	要显示的字符
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_string(LCDDev_t *dev, uint16_t x, uint16_t y, char *str, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{         
	while (*str != '\0')
	{       
		__lcd_show_char(dev, x, y, *str, fc, bc, size, mode);
		x += size / 2;
		str++;
	}
}

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

/******************************************************************************
 * @brief	LCD显示整数变量
 * @param	dev	:	LCDDev_t 结构体指针
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
static void __lcd_show_num(LCDDev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
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
 * @param	dev	:	LCDDev_t 结构体指针
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
static void __lcd_show_hex_num(LCDDev_t *dev, uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	uint8_t i, single_num;

	for (i = 0; i < len; i++)		// 遍历数字的每一位
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
 * @param	dev		:	LCDDev_t 结构体指针
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

static void __lcd_show_float_num(LCDDev_t *dev, uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
    uint8_t t, temp, sizex;
    uint32_t num1;
    sizex = size / 2;
    
    if (num < 0)
	{
        __lcd_show_char(dev, x, y, '-', fc, bc, size, mode);
        x += sizex; // 移动x坐标以适应负号
        num = -num; // 取绝对值
    }

    num1 = (uint32_t)(num * 10000); // 确保小数点后有四位

    for (t = 0; t < len; t++)
	{
        temp = (num1 / __lcd_pow(10, len - t - 1)) % 10;
        if (t == (len - 4))// 小数点前加一位
		{ 
            __lcd_show_char(dev, x + (len - 4) * sizex, y, '.', fc, bc, size, mode);
            t++;
            len += 1;
        }
        __lcd_show_char(dev, x + t * sizex, y, temp + 48, fc, bc, size, mode);
    }
}

/******************************************************************************
 * @brief	LCD显示单个12x12汉字
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese12x12(LCDDev_t *dev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t Chinese_num = get_Chinese_num(12); 					// 汉字数目
	uint16_t typeface_num = (12 / 8 + ((12 % 8) ? 1 : 0)) * 12;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < Chinese_num; k++)
	{
		if ((LCD_CF12x12[k].Index[0] == *(Chinese)) && (LCD_CF12x12[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_cursor(dev, x, y);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF12x12[k].Msk[i] & (0x01 << j))
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
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese16x16(LCDDev_t *dev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t Chinese_num = get_Chinese_num(16); 					// 汉字数目
	uint16_t typeface_num = (16 / 8 + ((16 % 8) ? 1 : 0)) * 16;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < Chinese_num; k++)
	{
		if ((LCD_CF16x16[k].Index[0] == *(Chinese)) && (LCD_CF16x16[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_cursor(dev, x, y);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF16x16[k].Msk[i] & (0x01 << j))
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
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese24x24(LCDDev_t *dev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t Chinese_num = get_Chinese_num(24); 					// 汉字数目
	uint16_t typeface_num = (24 / 8 + ((24 % 8) ? 1 : 0)) * 24;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < Chinese_num; k++)
	{
		if ((LCD_CF24x24[k].Index[0] == *(Chinese)) && (LCD_CF24x24[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_cursor(dev, x, y);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF24x24[k].Msk[i] & (0x01 << j))
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
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese32x32(LCDDev_t *dev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t mode)
{
	uint8_t i, j;
	uint16_t k;
	uint16_t Chinese_num = get_Chinese_num(32); 					// 汉字数目
	uint16_t typeface_num = (32 / 8 + ((32 % 8) ? 1 : 0)) * 32;	// 一个字符所占字节大小
	uint16_t x0 = x;
	
	for (k = 0; k < Chinese_num; k++)
	{
		if ((LCD_CF32x32[k].Index[0] == *(Chinese)) && (LCD_CF32x32[k].Index[1] == *(Chinese + 1)))
		{
			__lcd_set_cursor(dev, x, y);
			for (i = 0; i < typeface_num; i++)
			{
				for (j = 0; j < 8; j++)
				{
					if (LCD_CF32x32[k].Msk[i] & (0x01 << j))
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
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	x坐标
 * @param	y		:	y坐标
 * @param	Chinese	:	要显示的汉字串
 * @param	fc		:	字的颜色
 * @param	bc		:	字的背景色
 * @param	size	:	指定字体大小：LCD_16X32 / LCD_12X24 / LCD_8X16 / LCD_6X12
 * @param	mode	:	模式：0非叠加/1叠加
 * @return	无
 ******************************************************************************/
static void __lcd_show_chinese(LCDDev_t *dev, uint16_t x, uint16_t y, char *Chinese, uint16_t fc, uint16_t bc, uint8_t size, uint8_t mode)
{
	while(*Chinese != 0)
	{
		if(size==LCD_12X12)			__lcd_show_chinese12x12(dev, x, y, Chinese, fc, bc, mode);
		else if(size==LCD_16X16)	__lcd_show_chinese16x16(dev, x, y, Chinese, fc, bc, mode);
		else if(size==LCD_24X24)	__lcd_show_chinese24x24(dev, x, y, Chinese, fc, bc, mode);
		else if(size==LCD_32X32)	__lcd_show_chinese32x32(dev, x, y, Chinese, fc, bc, mode);
		else return;
		
		Chinese += LCD_CHN_CHAR_WIDTH;
		x += size;
	}
}

/******************************************************************************
 * @brief	LCD显示图片
 * @param	dev	:	LCDDev_t 结构体指针
 * @param	x		:	图片左上角x坐标
 * @param	y		:	图片左上角y坐标
 * @param	width	:	图片宽度
 * @param	height	:	图片高度
 * @param	pic[]	:	图片数组
 * @return	无
 ******************************************************************************/
static void __lcd_show_image(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{
	for (uint16_t i = 0; i < height; i++)           // 一行一行地显示
    {
        LCD_ADDR->reg = 0X2A;                        // 设置x坐标
        LCD_ADDR->ram = x >> 8;                      // X坐标的高8位
        LCD_ADDR->ram = x ;                          // X坐标的低8位，因为指令读值只是低8位有效，所以等效于 x1 & 0XFF
        LCD_ADDR->reg = 0X2B;                        // 设置y坐标
        LCD_ADDR->ram = (y + i) >> 8;                // Y坐标的高8位
        LCD_ADDR->ram = y + i;                       // Y坐标的低8位，因为指令读值只是低8位有效，所以等效于 Y & 0XFF
        LCD_ADDR->reg = 0X2C;                        // 开始写GRAM
        for (uint16_t j = 0; j < width; j++)        // 一行中，从左到事，逐个像素处理
        {
            LCD_ADDR->ram = pic[1] << 8 | *pic;  	// 写入16位颜色数据
            pic += 2;                             	// 数据指针向后移两字节
        }
    }
}

/******************************************************************************
 * @brief	LCD画点
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_point(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t color)
{
	__lcd_set_cursor(dev, x, y);	// 设置光标位置
	__lcd_write_ram_prepare(dev);	// 准备写GRAM
	LCD_ADDR->ram = color;			// 写入该点颜色
}

/******************************************************************************
 * @brief	LCD读点
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @return	要读的点的16位颜色数据
 ******************************************************************************/
static uint16_t __lcd_read_point(LCDDev_t *dev, uint16_t x, uint16_t y)
{
	uint16_t r = 0, g = 0, b = 0;
	__lcd_set_cursor(dev, x, y);	// 设置光标位置
	__lcd_write_cmd(0x2E);			// 读点命令
	r = __lcd_read_data();			// 假读
	r = __lcd_read_data();			// 读rg
	b = __lcd_read_data();			// 读b
	g = r & 0xFF;					// 计算得到g
	
	return (((r >> 11) << 11) | ((g >> 2) << 5) | (b >> 11));
}

/******************************************************************************
 * @brief	LCD画线
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x1		:  起始x坐标
 * @param	y1		:  起始y坐标
 * @param	x2		:  终点x坐标
 * @param	y2		:  终点y坐标
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_line(LCDDev_t *dev, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
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
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x		:  矩形左上角x坐标
 * @param	y		:  矩形左上角y坐标
 * @param	width	:  矩形宽度
 * @param	height	:  矩形高度
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_rectangle(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
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
 * @param	dev	:  LCDDev_t 结构体指针
 * @param	x		:  x坐标
 * @param	y		:  y坐标
 * @param	r		:  半径
 * @param	color	:  颜色
 * @return	无
 ******************************************************************************/
static void __lcd_draw_circle(LCDDev_t *dev, uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
    int a, b;
	a = 0;
	b = r;
	
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
		if((a * a + b * b) > (r * r))		//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
 * @brief	去初始化LCD
 * @param	dev   :  LCDDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __lcd_deinit(LCDDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;
	
	dev->init_flag = false;	// 修改初始化标志
	
	return 0;
}
