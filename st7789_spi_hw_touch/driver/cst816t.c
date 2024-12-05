#include "cst816t.h"

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__cst816t_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
												    	else if(port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
												    	else if(port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
												    	else if(port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
												    	else if(port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
												    	else if(port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
												    	else if(port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
												    	else					{cst816t_log("cst816t gpio clock no enable\r\n");} \
												    }

#define	__cst816t_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
											    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											    	GPIO_InitStructure.GPIO_Pin = pin ; \
											    	GPIO_Init(port, &GPIO_InitStructure); \
											    }
											
#define	__cst816t_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

    #if !FREERTOS
        static void __cst816t_delay_ms(uint32_t ms)
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
        static void __cst816t_delay_ms(uint32_t ms)
        {
            vTaskDelay(ms);
        }								  
        #endif

#elif defined(STM32F40_41xxx)

#define	__cst816t_config_gpio_clock_enable(port)	{	if(port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
												    	else if(port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
												    	else if(port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
												    	else if(port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
												    	else if(port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
												    	else if(port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
												    	else if(port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
												    	else					{cst816t_log("cst816t gpio clock no enable\r\n");} \
												    }

#define	__cst816t_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; \
											    	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; \
											    	GPIO_InitStructure.GPIO_Pin = pin; \
											    	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; \
											    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											    	GPIO_Init(port, &GPIO_InitStructure); \
											    }
											
#define	__cst816t_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#if !FREERTOS
	static void __cst816t_delay_ms(uint32_t ms)
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
	static void __cst816t_delay_ms(uint32_t ms)
	{
		vTaskDelay(ms);
	}								  
	#endif

#endif

/* CST816T私有数据结构体 */
typedef struct {
	I2CDev_t cst816tI2C;	// 软件I2C设备
}CST816TPrivData_t;

/* 函数声明 */
static void __cst816t_write_reg(CST816TDev_t *pDev, uint8_t addr, uint8_t data);
static void __cst816t_read_reg(CST816TDev_t *pDev, uint8_t addr, uint8_t *data);
static void __cst816t_read_regs(CST816TDev_t *pDev, uint8_t addr, uint8_t num, uint8_t data[]);
static void __cst816t_get_id(CST816TDev_t *pDev, uint8_t *id);
static void __cst816t_get_firmware_ver(CST816TDev_t *pDev, uint8_t *fwVer);
static void __cst816t_get_finger_num(CST816TDev_t *pDev, uint8_t *num);
static void __cst816t_get_action(CST816TDev_t *pDev);
static int __cst816t_deinit(CST816TDev_t *pDev);

/******************************************************************************
 * @brief	初始化CST816T
 * @param	pDev	:  CST816TDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int cst816t_init(CST816TDev_t *pDev)
{
    if (!pDev)
		return -1;

    /* 保存私有数据 */
	pDev->pPrivData = (CST816TPrivData_t *)malloc(sizeof(CST816TPrivData_t));
	if (!pDev->pPrivData)
		return -1;
	
	CST816TPrivData_t *pPrivData = (CST816TPrivData_t *)pDev->pPrivData;
	
	pPrivData->cst816tI2C.info.SCLPort = pDev->info.SCLPort;
	pPrivData->cst816tI2C.info.SCLPin = pDev->info.SCLPin;
	pPrivData->cst816tI2C.info.SDAPort = pDev->info.SDAPort;
	pPrivData->cst816tI2C.info.SDAPin = pDev->info.SDAPin;
	
	/* 配置软件I2C */
	i2c_init(&pPrivData->cst816tI2C);
	
	/* 配置时钟与GPIO */
	__cst816t_config_gpio_clock_enable(pDev->info.RSTPort);
	__cst816t_config_io_out_pp(pDev->info.RSTPort, pDev->info.RSTPin);
    __cst816t_io_write(pDev->info.RSTPort, pDev->info.RSTPin, GPIO_LEVEL_HIGH);

	/* 复位 */
    __cst816t_io_write(pDev->info.RSTPort, pDev->info.RSTPin, GPIO_LEVEL_LOW);
    __cst816t_delay_ms(10);
    __cst816t_io_write(pDev->info.RSTPort, pDev->info.RSTPin, GPIO_LEVEL_HIGH);
    __cst816t_delay_ms(50);

    /* 函数指针赋值 */
	pDev->get_id = __cst816t_get_id;
	pDev->get_firmware_ver = __cst816t_get_firmware_ver;
	pDev->get_finger_num = __cst816t_get_finger_num;
	pDev->get_action = __cst816t_get_action;
	pDev->deinit = __cst816t_deinit;

    pDev->initFlag = true;
	return 0;
}

/******************************************************************************
 * @brief	CST816T写寄存器
 * @param	pDev   :  CST816TDev_t结构体指针
 * @param	addr   :  要写入的寄存器地址
 * @param	data   :  要写入的寄存器数据
 * @return	无
 ******************************************************************************/
static void __cst816t_write_reg(CST816TDev_t *pDev, uint8_t addr, uint8_t data)
{
    CST816TPrivData_t *pPrivData = (CST816TPrivData_t *)pDev->pPrivData;

    pPrivData->cst816tI2C.start(&pPrivData->cst816tI2C);						// I2C起始
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, CST816T_ADDRESS);	// 发送从机地址，读写位为0，表示即将写入
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);					    // 接收应答
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, addr);	            // 发送寄存器地址
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);					    // 接收应答
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, data);			    // 发送要写入寄存器的数据
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);					    // 接收应答
	pPrivData->cst816tI2C.stop(&pPrivData->cst816tI2C);						    // I2C终止
}

/******************************************************************************
 * @brief	CST816T读寄存器
 * @param	pDev    :   CST816TDev_t结构体指针
 * @param	addr    :   要读的寄存器地址
 * @param	data    :   要读的寄存器数据
 * @return	无
 ******************************************************************************/
static void __cst816t_read_reg(CST816TDev_t *pDev, uint8_t addr, uint8_t *data)
{
    CST816TPrivData_t *pPrivData = (CST816TPrivData_t *)pDev->pPrivData;
	
	pPrivData->cst816tI2C.start(&pPrivData->cst816tI2C);							// I2C起始
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, CST816T_ADDRESS);		// 发送从机地址，读写位为0，表示即将写入
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);							// 接收应答
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, addr);					// 发送寄存器地址
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);							// 接收应答
	
	pPrivData->cst816tI2C.start(&pPrivData->cst816tI2C);							// I2C重复起始
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, CST816T_ADDRESS | 0x01);// 发送从机地址，读写位为1，表示即将读取
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);							// 接收应答
	*data = pPrivData->cst816tI2C.recv_byte(&pPrivData->cst816tI2C);				// 接收指定寄存器的数据
	pPrivData->cst816tI2C.send_ack(&pPrivData->cst816tI2C, 1);						// 发送应答，给从机非应答，终止从机的数据输出
	pPrivData->cst816tI2C.stop(&pPrivData->cst816tI2C);								// I2C终止					
}

/******************************************************************************
 * @brief	CST816T读多个寄存器
 * @param	pDev    :   CST816TDev_t结构体指针
 * @param	addr    :   要读的寄存器首地址
 * @param	num		:   要读的寄存器个数
 * @param	data    :   要读的寄存器数据
 * @return	无
 ******************************************************************************/
static void __cst816t_read_regs(CST816TDev_t *pDev, uint8_t addr, uint8_t num, uint8_t data[])
{
	uint8_t i;
	uint8_t val;

    CST816TPrivData_t *pPrivData = (CST816TPrivData_t *)pDev->pPrivData;
	
	pPrivData->cst816tI2C.start(&pPrivData->cst816tI2C);							// I2C起始
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, CST816T_ADDRESS);		// 发送从机地址，读写位为0，表示即将写入
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);							// 接收应答
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, addr);					// 发送寄存器地址
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);							// 接收应答
	
	pPrivData->cst816tI2C.start(&pPrivData->cst816tI2C);							// I2C重复起始
	pPrivData->cst816tI2C.send_byte(&pPrivData->cst816tI2C, CST816T_ADDRESS | 0x01);// 发送从机地址，读写位为1，表示即将读取
	pPrivData->cst816tI2C.recv_ack(&pPrivData->cst816tI2C);							// 接收应答

	for (i = 0; i < num; i++)
    {
        data[i] = pPrivData->cst816tI2C.recv_byte(&pPrivData->cst816tI2C);			// 接收数据
        if (i == num - 1)
        {
            pPrivData->cst816tI2C.send_ack(&pPrivData->cst816tI2C, 1);				// 发送非应答信号
        }
        else
        {
            pPrivData->cst816tI2C.send_ack(&pPrivData->cst816tI2C, 0);				// 发送应答信号
        }
    }

	pPrivData->cst816tI2C.stop(&pPrivData->cst816tI2C);								// I2C终止					
}

/******************************************************************************
 * @brief	CST816T读取ID
 * @param	pDev	:  CST816TDev_t结构体指针
 * @param	id		:  ID
 * @return	无
 ******************************************************************************/
static void __cst816t_get_id(CST816TDev_t *pDev, uint8_t *id)
{
    __cst816t_read_reg(pDev, CHIP_ID, id);
}

/******************************************************************************
 * @brief	CST816T读取固件版本号
 * @param	pDev	:  CST816TDev_t结构体指针
 * @param	fwVer	:  固件版本号
 * @return	无
 ******************************************************************************/
static void __cst816t_get_firmware_ver(CST816TDev_t *pDev, uint8_t *fwVer)
{
    __cst816t_read_reg(pDev, FW_VER, fwVer);
}

/******************************************************************************
 * @brief	CST816T读取触摸手指个数
 * @param	pDev	:  CST816TDev_t结构体指针
 * @param	num		:  触摸手指个数
 * @return	无
 ******************************************************************************/
static void __cst816t_get_finger_num(CST816TDev_t *pDev, uint8_t *num)
{
    __cst816t_read_reg(pDev, FINGER_NUM, num);
}

/******************************************************************************
 * @brief	获取CST816T的动作信息，上下左右对应1234
 * @param	pDev    :   CST816TDev_t结构体指针
 * @return	无
 ******************************************************************************/
static void __cst816t_get_action(CST816TDev_t *pDev)
{
	uint8_t data[6];
	uint8_t gesture;
	uint16_t x, y;
	uint8_t retryCnt = 0;

	while (retryCnt < 5)
	{
		/* 读取寄存器 */
		__cst816t_read_regs(pDev, GESTURE_ID, 6, data);

		gesture = data[0];
		x = ((data[2] & 0x0F) << 8) | data[3];
		y = ((data[4] & 0x0F) << 8) | data[5];

		/* 检查坐标范围并根据方向调整坐标与状态码 */
		if ((pDev->info.dir == VERTICAL_FORWARD) && (x <= LCD_W && y <= LCD_H))
		{
			pDev->x = x;
			pDev->y = y;

            switch(gesture)
			{
				case 1: pDev->gesture = GESTURE_DOWN;	break;
				case 2: pDev->gesture = GESTURE_UP;		break;
				case 3: pDev->gesture = GESTURE_LEFT;	break;
				case 4: pDev->gesture = GESTURE_RIGHT;	break;
			}
			return;
        }
		else if ((pDev->info.dir == VERTICAL_REVERSE) && (x <= LCD_W && y <= LCD_H))
		{
			pDev->x = LCD_W - x;
			pDev->y = LCD_H - y;

			switch(gesture)
			{
				case 1: pDev->gesture = GESTURE_UP;		break;
				case 2: pDev->gesture = GESTURE_DOWN;	break;
				case 3: pDev->gesture = GESTURE_RIGHT;	break;
				case 4: pDev->gesture = GESTURE_LEFT;	break;
			}
			return;
        }
		else if ((pDev->info.dir == HORIZONTAL_FORWARD) && (x <= LCD_H && y <= LCD_W))
		{
			pDev->x = y;
			pDev->y = LCD_W - x;

			switch(gesture)
			{
				case 1: pDev->gesture = GESTURE_RIGHT;	break;
				case 2: pDev->gesture = GESTURE_LEFT;	break;
				case 3: pDev->gesture = GESTURE_DOWN;	break;
				case 4: pDev->gesture = GESTURE_UP;		break;
			}
			return;
		}
		else if ((pDev->info.dir == HORIZONTAL_REVERSE) && (x <= LCD_H && y <= LCD_W))
		{
			pDev->x = LCD_H - y;
			pDev->y = x;
			
			switch(gesture)
			{
				case 1: pDev->gesture = GESTURE_LEFT;	break;
				case 2: pDev->gesture = GESTURE_RIGHT;	break;
				case 3: pDev->gesture = GESTURE_UP;		break;
				case 4: pDev->gesture = GESTURE_DOWN;	break;
			}
			return;
        }

        /* 如果坐标超出范围，重置设备并重试 */
        __cst816t_io_write(pDev->info.RSTPort, pDev->info.RSTPin, GPIO_LEVEL_LOW);
        __cst816t_delay_ms(10);
        __cst816t_io_write(pDev->info.RSTPort, pDev->info.RSTPin, GPIO_LEVEL_HIGH);
        __cst816t_delay_ms(10);

		retryCnt++;
	}
}

/******************************************************************************
 * @brief	去初始化CST816T
 * @param	pDev   :  CST816TDev_t结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_deinit(CST816TDev_t *pDev)
{    
    if (!pDev || !pDev->initFlag)
        return -1;
	
	pDev->initFlag = false;	// 修改初始化标志
    
    return 0;
}
