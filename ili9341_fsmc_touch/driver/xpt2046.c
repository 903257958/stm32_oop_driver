#include "xpt2046.h"
            
#if defined(STM32F40_41xxx)

#define	__xpt2046_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
													    else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
													    else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
													    else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
													    else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
													    else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
													    else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
													    else					{xpt2046_log("xpt2046 gpio clock no enable\r\n");} \
												    }

#define	__xpt2046_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
											    	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
											    	GPIO_InitStructure.GPIO_Pin = pin; \
											    	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
											    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											    	GPIO_Init(port, &GPIO_InitStructure); \
											    }

#define	__xpt2046_config_io_in_pu(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; \
											    	GPIO_InitStructure.GPIO_Pin = pin; \
											    	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; \
											    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											    	GPIO_Init(port, &GPIO_InitStructure); \
											    }

#define	__xpt2046_io_read(port, pin)    GPIO_ReadInputDataBit(port, pin)

#define	__xpt2046_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

static void __xpt2046_delay_us(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++)
    {
        uint8_t uc = 12;     //设置值为12，大约延1微秒
        while (uc --);       //延1微秒
    }
}

#endif

#define XPT2046_CHANNEL_X   0x90    // 控制字：检测通道Y+电压值    
#define XPT2046_CHANNEL_Y   0xD0    // 控制字：检测通道X+电压值

/* LCD屏幕设备，需在外部定义 */
extern LCDDev_t lcd;

/* XPT2046私有数据结构体 */
typedef struct {
    XPT2046_GPIO_Port	PENPort;
	uint32_t		    PENPin;
	XPT2046_GPIO_Port	CSPort;
	uint32_t		    CSPin;
	XPT2046_GPIO_Port	SCLKPort;
	uint32_t		    SCLKPin;
	XPT2046_GPIO_Port	MOSIPort;
	uint32_t		    MOSIPin;
	XPT2046_GPIO_Port	MISOPort;
	uint32_t		    MISOPin;
    uint8_t             EN;         // 触摸检测开关
    uint16_t            lcdX;       // 当前按下的X坐标值
    uint16_t            lcdY;       // 当前按下的Y坐标值
    uint16_t            adcX;       // 保存最后读取的触摸屏X方向的ADC值，已用平均值滤波
    uint16_t            adcY;       // 保存最后读取的触摸屏Y方向的ADC值，已用平均值滤波                                                              
    uint8_t             dir;        // 显示方向, 0竖屏, 1横屏
}XPT2046PrivData_t;

/* 函数声明 */
static void __xpt2046_cmd(XPT2046Dev_t *pDev, uint8_t status);
static bool __xpt2046_is_pressed(XPT2046Dev_t *pDev);
static uint16_t  __xpt2046_get_x(XPT2046Dev_t *pDev);
static uint16_t  __xpt2046_get_y(XPT2046Dev_t *pDev);
static int  __xpt2046_recalibration(XPT2046Dev_t *pDev);
static void __xpt2046_draw(XPT2046Dev_t *pDev, uint16_t color, uint16_t bc);

/******************************************************************************
 * @brief	初始化触摸屏
 * @param	pDev	:  XPT2046Dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int xpt2046_init(XPT2046Dev_t *pDev)
{
    if (!pDev)
		return -1;
	
	/* 初始化私有数据 */
	pDev->pPrivData = (XPT2046PrivData_t *)malloc(sizeof(XPT2046PrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    /* 触摸屏的引脚一般不会改变，直接在此私有数据中定义引脚，不提供外部接口 */
    pPrivData->PENPort = GPIOE;
    pPrivData->PENPin = GPIO_Pin_4;
    pPrivData->CSPort = GPIOD;
    pPrivData->CSPin = GPIO_Pin_13;
    pPrivData->SCLKPort = GPIOE;
    pPrivData->SCLKPin = GPIO_Pin_0;
    pPrivData->MOSIPort = GPIOE;
    pPrivData->MOSIPin = GPIO_Pin_2;
    pPrivData->MISOPort = GPIOE;
    pPrivData->MISOPin = GPIO_Pin_3;

    /* 开启时钟 */
    __xpt2046_config_gpio_clock_enable(pPrivData->PENPort);
    __xpt2046_config_gpio_clock_enable(pPrivData->CSPort);
    __xpt2046_config_gpio_clock_enable(pPrivData->SCLKPort);
    __xpt2046_config_gpio_clock_enable(pPrivData->MOSIPort);
    __xpt2046_config_gpio_clock_enable(pPrivData->MISOPort);

    /* 初始化GPIO */
    __xpt2046_config_io_out_pp(pPrivData->CSPort, pPrivData->CSPin);
    __xpt2046_config_io_out_pp(pPrivData->SCLKPort, pPrivData->SCLKPin);
    __xpt2046_config_io_out_pp(pPrivData->MOSIPort, pPrivData->MOSIPin);
    __xpt2046_config_io_in_pu(pPrivData->MISOPort, pPrivData->MISOPin);
    __xpt2046_config_io_in_pu(pPrivData->PENPort, pPrivData->PENPin);

    /* 设置引脚初始状态 */
    __xpt2046_io_write(pPrivData->CSPort, pPrivData->CSPin, 1);     // 拉高片选，防止误操作
    __xpt2046_io_write(pPrivData->SCLKPort, pPrivData->SCLKPin, 0);   // XPT2046时序要求：CLK 闲时低电平，上升沿采样数据，下降沿改变数据
    __xpt2046_io_write(pPrivData->MOSIPort, pPrivData->MOSIPin, 0); // XPT2046时序要求：MOSI闲时低电平
    
    __xpt2046_io_write(pPrivData->CSPort, pPrivData->CSPin, 0);     // 拉低片选，使XTP2046开始通信

    /* 打开触摸检测 */
    __xpt2046_cmd(pDev, ENABLE);

    /* 函数指针赋值 */
    pDev->cmd = __xpt2046_cmd;
    pDev->is_pressed = __xpt2046_is_pressed;
    pDev->get_x = __xpt2046_get_x;
    pDev->get_y = __xpt2046_get_y;
	pDev->recalibration = __xpt2046_recalibration;
    pDev->draw = __xpt2046_draw;

	pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	触摸屏写入命令字
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @param	cmd	    :  0x90：通道Y+的选择控制字, 0xd0：通道X+的选择控制字
 * @return	无
 ******************************************************************************/
static void __xpt2046_send_cmd(XPT2046Dev_t *pDev, uint8_t cmd)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    for (uint8_t i = 0; i < 8; i++)
    {
        ((cmd >> (7 - i)) & 0x01) ? __xpt2046_io_write(pPrivData->MOSIPort, pPrivData->MOSIPin, 1) : 
                                    __xpt2046_io_write(pPrivData->MOSIPort, pPrivData->MOSIPin, 0); // 高位先行

        /* CLK闲时低电平，上升沿采样数据，下降沿改变数据 */
        __xpt2046_delay_us(1);
		__xpt2046_io_write(pPrivData->SCLKPort, pPrivData->SCLKPin, GPIO_LEVEL_HIGH);
        __xpt2046_delay_us(1);
        __xpt2046_io_write(pPrivData->SCLKPort, pPrivData->SCLKPin, GPIO_LEVEL_LOW);
    }
}

/******************************************************************************
 * @brief	触摸屏读取返回数据
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	读取到的16位数据
 ******************************************************************************/
static uint16_t __xpt2046_recv_data(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    uint16_t data = 0;

    /* 给一个时钟，清除BUSY；这个时序是跟在发送命令字后面的，AD转换需要大约6US */
    __xpt2046_io_write(pPrivData->SCLKPort, pPrivData->SCLKPin, 1);
    __xpt2046_delay_us(5);
    __xpt2046_io_write(pPrivData->SCLKPort, pPrivData->SCLKPin, 0);
    __xpt2046_delay_us(5);

    for (uint8_t i = 0; i < 12; i++)
    {
        data = data << 1;
        __xpt2046_io_write(pPrivData->SCLKPort, pPrivData->SCLKPin, 1);
        __xpt2046_delay_us(1);
        data |= __xpt2046_io_read(pPrivData->MISOPort, pPrivData->MISOPin) ;    // 高位先行
        __xpt2046_io_write(pPrivData->SCLKPort, pPrivData->SCLKPin, 0);
        __xpt2046_delay_us(1);
    }

    return data;
}

/******************************************************************************
 * @brief	触摸屏返回模拟通道x的ADC采样结果
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	模拟通道x的16位ADC采样结果
 ******************************************************************************/
static int16_t __xpt2046_read_adc_x(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    if (pPrivData->dir == 0)    __xpt2046_send_cmd(pDev, XPT2046_CHANNEL_Y);
    if (pPrivData->dir == 1)    __xpt2046_send_cmd(pDev, XPT2046_CHANNEL_X);
    return  __xpt2046_recv_data(pDev);
}

/******************************************************************************
 * @brief	触摸屏返回模拟通道y的ADC采样结果
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	模拟通道y的16位ADC采样结果
 ******************************************************************************/
static int16_t __xpt2046_read_adc_y(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    if (pPrivData->dir == 0)    __xpt2046_send_cmd(pDev, XPT2046_CHANNEL_X);
    if (pPrivData->dir == 1)    __xpt2046_send_cmd(pDev, XPT2046_CHANNEL_Y);
    return  __xpt2046_recv_data(pDev);
}

/******************************************************************************
 * @brief	触摸屏获取触摸屏按下时X、Y的ADC值，并滤波
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __xpt2046_read_adc_xy(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    static uint8_t  cnt = 0;
    static uint16_t xSum = 0, ySum = 0;
    static int16_t  xyArray [2] [10] = {{0}, {0}};    // 临时二维数组，用于存放坐标X、Y的10次采样
    int32_t  xMin, xMax, yMin, yMax;                  // 存储采样中的最小值、最大值;　采样多次后，去头去尾求平均值

    cnt = 0;
    xSum = 0;
    ySum = 0;
    memset(xyArray, 0, 20);
    xMin = 0;
    xMax = 0;
    yMin = 0;
    yMax = 0;

    /* 多次采集 */
    while ((__xpt2046_io_read(pPrivData->PENPort, pPrivData->PENPin) == 0) && (cnt < 4))
    {
        xyArray[0][cnt] = __xpt2046_read_adc_x(pDev);
        xyArray[1][cnt] = __xpt2046_read_adc_y(pDev);
        cnt++;
    }

    /* 开始求平均值 */
    xMax = xMin = xyArray [0] [0];                              // 筛选等会要去掉的最小值、最大值
    yMax = yMin = xyArray [1] [0];
    for (uint8_t i = 1; i < cnt; i++)
    {
        if (xyArray[0][i] < xMin)    xMin = xyArray [0] [i];    // 求x的10次采样最小ADC值
        if (xyArray[0][i] > xMax)    xMax = xyArray [0] [i];    // 求x的10次采样最大ADC值

        if (xyArray[1][i] < yMin)    yMin = xyArray [1] [i];    // 求y的10次采样最小ADC值
        if (xyArray[1][i] > yMax)    yMax = xyArray [1] [i];    // 求y的10次采样最小ADC值
    }

    /* 去除最小值和最大值之后求平均值 */
    for (uint8_t i = 0; i < cnt; i++)
    {
        xSum = xSum + xyArray[0][i];
        ySum = ySum + xyArray[1][i];
    }
    pPrivData->adcX = (xSum - xMin - xMax) >> 1;  // 去除最小值和最大值之后，除2
    pPrivData->adcY = (ySum - yMin - yMax) >> 1;  // 去除最小值和最大值之后，除2

    return 0;
}

/******************************************************************************
 * @brief	把电压值换算成对应的触摸屏坐标值
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __xpt2046_adc_xy_to_lcd_xy(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    static int16_t lcdX = 0;
    static int16_t lcdY = 0;

    /* 计算比例系数 */
    lcdX = pPrivData->adcX * pDev->info.xfac + pDev->info.xoff ;
    lcdY = pPrivData->adcY * pDev->info.yfac + pDev->info.yoff ;

    /* 限制坐标值范围 */
    if (lcdX < 0)  lcdX = 0;
    if (lcdX > pDev->info.lcdWidth)  lcdX = pDev->info.lcdWidth;
    if (lcdY < 0)  lcdY = 0;
    if (lcdY > pDev->info.lcdHeight)  lcdY = pDev->info.lcdHeight;

    /* 经换算, 及限值后的坐标值, 转存到结构体, 随时可调用 */
    pPrivData->lcdX = lcdX;
    pPrivData->lcdY = lcdY;
}

/******************************************************************************
 * @brief	触摸屏开关，不使用触屏时, 可关闭检测以节省芯片资源
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @param	status  :  1打开，0关闭
 * @return	无
 ******************************************************************************/
static void __xpt2046_cmd(XPT2046Dev_t *pDev, uint8_t status)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    if (status != 0)
    {
        pPrivData->EN = 1;
    }
    else
    {
        pPrivData->EN = 0;
    }
}

/******************************************************************************
 * @brief	触摸屏是否按下
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	false未按下，true按下
 ******************************************************************************/
static bool __xpt2046_is_pressed(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    static uint8_t status = 0;

    /* 检查触摸是否开启 */
    if (pPrivData->EN == 0)
    {
        return false;
    }

    /* PEN引脚闲时为高电平，按下时为低电平 */
    status = __xpt2046_io_read(pPrivData->PENPort, pPrivData->PENPin) ? 0 : 1 ;

    /* 已按下 */
    if (status == true)
    {
        __xpt2046_read_adc_xy(pDev);        // 获取XPT2046的按下位置(电压值)
        __xpt2046_adc_xy_to_lcd_xy(pDev);   // 换算成显示屏的坐标值;
        uint16_t x1 = pPrivData->lcdX;
        uint16_t y1 = pPrivData->lcdY;

        __xpt2046_read_adc_xy(pDev);        // 获取XPT2046的按下位置(电压值)
        __xpt2046_adc_xy_to_lcd_xy(pDev);   // 换算成显示屏的坐标值;
        uint16_t x2 = pPrivData->lcdX;
        uint16_t y2 = pPrivData->lcdY;

        /* 计算采值差 */
        uint8_t x, y;
        if (x1 > x2)
            x = x1 - x2;
        else
            x = x2 - x1;

        if (y1 > y2)
            y = y1 - y2;
        else
            y = y2 - y1;

        if (x > 3 || y > 3)
            return false;

        /* 计算平均值 */
        pPrivData->lcdX = (x1 + x2) >> 1;
        pPrivData->lcdY = (y1 + y2) >> 1;
    }    

    return status;     
}

/******************************************************************************
 * @brief	触摸屏获取按下位置的x坐标值
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	按下位置的16位x坐标值
 ******************************************************************************/
static uint16_t  __xpt2046_get_x(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    return pPrivData->lcdX;
}

/******************************************************************************
 * @brief	触摸屏获取按下位置的y坐标值
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	按下位置的16位y坐标值
 ******************************************************************************/
static uint16_t  __xpt2046_get_y(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    return pPrivData->lcdY;
}

/******************************************************************************
* @brief	触摸屏重新校准，校准完毕后数据显示在屏幕上，修改代码并重新烧录
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int  __xpt2046_recalibration(XPT2046Dev_t *pDev)
{
    XPT2046PrivData_t *pPrivData = (XPT2046PrivData_t *)pDev->pPrivData;

    uint16_t pixelOff = 30;   // 偏移像素,用来画十字
    uint16_t adcX1, adcX2, adcX3, adcX4, adcY1, adcY2, adcY3, adcY4; // 记录校准过程中的坐标值
    float xfac = 0;
    float yfac = 0;
    float xoff = 0;
    float yoff = 0;
    uint16_t crossX = 0;      // 用于画十字线
    uint16_t crossY = 0;      // 用于画十字线
    char strTemp[30];
    uint16_t lcdWidth  = pDev->info.lcdWidth;
    uint16_t lcdHeight = pDev->info.lcdHeight;

    lcd.fill(&lcd, 0, 0, lcd.width, lcd.height, BLACK);
	
	lcd.show_string(&lcd, 20, 90, "Please use a pen to click", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 20, 115, "on the dots on the screen!", WHITE, BLACK, LCD_8X16, 0);

    /* 左上角 */
    crossX = pixelOff;
    crossY = pixelOff;
    pPrivData->adcX = 0;
    pPrivData->adcY = 0;
    __xpt2046_cmd(pDev, ENABLE);    // 打开触屏检测
	lcd.draw_point(&lcd, crossX, crossY, WHITE);
    while ((__xpt2046_read_adc_xy(pDev) != 0) || (pPrivData->adcX == 0));   // 等待按下
    __xpt2046_cmd(pDev, DISABLE);   // 关闭触屏检测
    adcX1 = pPrivData->adcX;        // 记录下取得的adc值
    adcY1 = pPrivData->adcY;        // 记录下取得的adc值
    lcd.draw_point(&lcd, crossX, crossY, BLACK);    // 抹去十字
    sprintf(strTemp, "X:%d", adcX1);
	lcd.show_string(&lcd, crossX - 12, crossY - 16, strTemp, WHITE, BLACK, LCD_6X12, 0);
    sprintf(strTemp, "Y:%d", adcY1);
	lcd.show_string(&lcd, crossX - 12, crossY, strTemp, WHITE, BLACK, LCD_6X12, 0);
    __xpt2046_delay_us(800000);

    // 右上角
    crossX = lcdWidth - pixelOff;
    crossY = pixelOff;
    pPrivData->adcX = 0;
    pPrivData->adcY = 0;
    __xpt2046_cmd(pDev, ENABLE);    // 打开触屏检测
	lcd.draw_point(&lcd, crossX, crossY, WHITE);
    while ((__xpt2046_read_adc_xy(pDev) != 0) || (pPrivData->adcX == 0));   // 等待按下
    __xpt2046_cmd(pDev, DISABLE);   // 关闭触屏检测
    adcX2 = pPrivData->adcX;        // 记录下取得的adc值
    adcY2 = pPrivData->adcY;        // 记录下取得的adc值
    lcd.draw_point(&lcd, crossX, crossY, BLACK);    // 抹去十字
    sprintf(strTemp, "X:%d", adcX2);
	lcd.show_string(&lcd, crossX - 12, crossY - 16, strTemp, WHITE, BLACK, LCD_6X12, 0);
    sprintf(strTemp, "Y:%d", adcY2);
	lcd.show_string(&lcd, crossX - 12, crossY, strTemp, WHITE, BLACK, LCD_6X12, 0);
    __xpt2046_delay_us(800000);

    // 左下角
    crossX = pixelOff;
    crossY = lcdHeight - pixelOff;
    pPrivData->adcX = 0;
    pPrivData->adcY = 0;
    __xpt2046_cmd(pDev, ENABLE);    // 打开触屏检测
    lcd.draw_point(&lcd, crossX, crossY, WHITE);
    while ((__xpt2046_read_adc_xy(pDev) != 0) || (pPrivData->adcX == 0));   // 等待按下
    __xpt2046_cmd(pDev, DISABLE);   // 关闭触屏检测
    adcX3 = pPrivData->adcX;        // 记录下取得的adc值
    adcY3 = pPrivData->adcY;        // 记录下取得的adc值
    lcd.draw_point(&lcd, crossX, crossY, BLACK);    // 抹去十字
    sprintf(strTemp, "X:%d", adcX3);
	lcd.show_string(&lcd, crossX - 12, crossY - 16, strTemp, WHITE, BLACK, LCD_6X12, 0);
    sprintf(strTemp, "Y:%d", adcY3);
	lcd.show_string(&lcd, crossX - 12, crossY, strTemp, WHITE, BLACK, LCD_6X12, 0);
    __xpt2046_delay_us(800000);

    // 右下角
    crossX = lcdWidth - pixelOff;
    crossY = lcdHeight - pixelOff;
    pPrivData->adcX = 0;
    pPrivData->adcY = 0;
    __xpt2046_cmd(pDev, ENABLE);    // 打开触屏检测
    lcd.draw_point(&lcd, crossX, crossY, WHITE);
    while ((__xpt2046_read_adc_xy(pDev) != 0) || (pPrivData->adcX == 0));   // 等待按下
    __xpt2046_cmd(pDev, DISABLE);   // 关闭触屏检测
    adcX4 = pPrivData->adcX;        // 记录下取得的adc值
    adcY4 = pPrivData->adcY;        // 记录下取得的adc值
    lcd.draw_point(&lcd, crossX, crossY, BLACK);    // 抹去十字
    sprintf(strTemp, "X:%d", adcX4);
	lcd.show_string(&lcd, crossX - 12, crossY - 16, strTemp, WHITE, BLACK, LCD_6X12, 0);
    sprintf(strTemp, "Y:%d", adcY4);
	lcd.show_string(&lcd, crossX - 12, crossY, strTemp, WHITE, BLACK, LCD_6X12, 0);
    __xpt2046_delay_us(400000);

    /* 取adcX和adcY的平均值; 如果不取平均值, 在对角画两个十字线即可 */
    adcX1 = (adcX1 + adcX3) / 2;
    adcX2 = (adcX2 + adcX4) / 2;

    adcY1 = (adcY1 + adcY2) / 2;
    adcY2 = (adcY3 + adcY4) / 2;

    xfac = (float)(pixelOff - (lcdWidth - pixelOff)) / (adcX1 - adcX2);   // 触摸屏与XPT2046的坐标比例系数,  xfac=(float)(20-320)/(t1x-t2x);
    yfac = (float)(pixelOff - (lcdHeight - pixelOff)) / (adcY1 - adcY2);
    xoff = (lcdWidth - xfac * (adcX1 + adcX2)) / 2;                       // 像素点偏移值, xoff=(320-xfac*(t1x+t2x))/2;
    yoff = (lcdHeight - yfac * (adcY1 + adcY2)) / 2;

    lcd.fill(&lcd, 0, 0, lcd.width, lcd.height, BLACK);
    lcd.show_string(&lcd, 40, 50, "Please modify the ", WHITE, BLACK, LCD_8X16, 0);
    lcd.show_string(&lcd, 40, 70, "code and re burn it!", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 60, 110, "xfac:", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 60, 130, "yfac:", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 60, 150, "xoff:", WHITE, BLACK, LCD_8X16, 0);
	lcd.show_string(&lcd, 60, 170, "yoff:", WHITE, BLACK, LCD_8X16, 0);
    lcd.show_float_num(&lcd, 100, 110, xfac, 6, WHITE, BLACK, LCD_8X16, 0);
    lcd.show_float_num(&lcd, 100, 130, yfac, 6, WHITE, BLACK, LCD_8X16, 0);
    lcd.show_float_num(&lcd, 100, 150, xoff, 6, WHITE, BLACK, LCD_8X16, 0);
    lcd.show_float_num(&lcd, 100, 170, yoff, 6, WHITE, BLACK, LCD_8X16, 0);

    while(1);

    // /* 软复位，重新运行 */
    // SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;
    // return 0;
}

/******************************************************************************
 * @brief	触摸屏在按下的位置画点
 * @param	pDev    :  XPT2046Dev_t结构体指针
 * @param	color	:  画笔颜色
 * @param	bc		:  背景板颜色
 * @return	按下位置的16位y坐标值
 ******************************************************************************/
static void __xpt2046_draw(XPT2046Dev_t *pDev, uint16_t color, uint16_t bc)
{
	uint16_t x, y;
	
	lcd.draw_rectangle(&lcd, 230, 0, 10, 10, color);
	
    /* 检查触摸屏是否按下 */
    if (__xpt2046_is_pressed(pDev))
    {
		x = __xpt2046_get_x(pDev);
		y = __xpt2046_get_y(pDev);
		
		if (x > 230 && y < 10)	// 触摸右上角清屏
		{
			lcd.clear(&lcd, bc);
			return;
		}
        lcd.draw_point(&lcd, x, y, color);	// 在按下的位置画点
        static char str[20] = {0};			// 存放坐标字符串
        sprintf(str, "X:%3d  Y:%3d", x, y); // 格式化坐标字符串
        lcd.show_string(&lcd, 84, 0, str, color, bc, LCD_6X12, 0);	// 显示坐标字符串
    }
}
