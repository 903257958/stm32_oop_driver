#include "delay.h"
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

#elif defined(STM32F40_41xxx) || defined(STM32F411xE)

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

#endif

/* CST816T私有数据结构体 */
typedef struct {
	I2CDev_t i2c;	// 软件I2C设备
}CST816TPrivData_t;

/* 函数声明 */
static int __cst816t_get_id(CST816TDev_t *dev, uint8_t *id);
static int __cst816t_get_firmware_ver(CST816TDev_t *dev, uint8_t *fw_ver);
static int __cst816t_get_finger_num(CST816TDev_t *dev, uint8_t *num);
static int __cst816t_get_action(CST816TDev_t *dev);
static int __cst816t_deinit(CST816TDev_t *dev);

/******************************************************************************
 * @brief	初始化CST816T
 * @param	dev	:  CST816TDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int cst816t_init(CST816TDev_t *dev)
{
    if (!dev)
		return -1;

    /* 保存私有数据 */
	dev->priv_data = (CST816TPrivData_t *)malloc(sizeof(CST816TPrivData_t));
	if (!dev->priv_data)
		return -1;
	
	CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;
	
	priv_data->i2c.info.scl_port = dev->info.scl_port;
	priv_data->i2c.info.scl_pin = dev->info.scl_pin;
	priv_data->i2c.info.sda_port = dev->info.sda_port;
	priv_data->i2c.info.sda_pin = dev->info.sda_pin;
	
	/* 配置软件I2C */
	i2c_init(&priv_data->i2c);
	
	/* 配置时钟与GPIO */
	__cst816t_config_gpio_clock_enable(dev->info.rst_port);
	__cst816t_config_io_out_pp(dev->info.rst_port, dev->info.rst_pin);
    __cst816t_io_write(dev->info.rst_port, dev->info.rst_pin, GPIO_LEVEL_HIGH);

	/* 复位 */
    __cst816t_io_write(dev->info.rst_port, dev->info.rst_pin, GPIO_LEVEL_LOW);
    delay_ms(10);
    __cst816t_io_write(dev->info.rst_port, dev->info.rst_pin, GPIO_LEVEL_HIGH);
    delay_ms(50);

    /* 函数指针赋值 */
	dev->get_id = __cst816t_get_id;
	dev->get_firmware_ver = __cst816t_get_firmware_ver;
	dev->get_finger_num = __cst816t_get_finger_num;
	dev->get_action = __cst816t_get_action;
	dev->deinit = __cst816t_deinit;
	
    dev->init_flag = true;
	return 0;
}

/******************************************************************************
 * @brief	CST816T读取ID
 * @param	dev		:  CST816TDev_t 结构体指针
 * @param	id		:  ID
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_get_id(CST816TDev_t *dev, uint8_t *id)
{
	if (!dev || !dev->init_flag)
		return -1;

	CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;
	
	priv_data->i2c.read_reg(&priv_data->i2c, CST816T_ADDRESS, CHIP_ID, id);

	return 0;
}

/******************************************************************************
 * @brief	CST816T读取固件版本号
 * @param	dev		:  CST816TDev_t 结构体指针
 * @param	fw_ver	:  固件版本号
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_get_firmware_ver(CST816TDev_t *dev, uint8_t *fw_ver)
{
	if (!dev || !dev->init_flag)
		return -1;

	CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;
	
	priv_data->i2c.read_reg(&priv_data->i2c, CST816T_ADDRESS, FW_VER, fw_ver);

	return 0;
}

/******************************************************************************
 * @brief	CST816T读取触摸手指个数
 * @param	dev	:  CST816TDev_t 结构体指针
 * @param	num	:  触摸手指个数
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_get_finger_num(CST816TDev_t *dev, uint8_t *num)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;

	priv_data->i2c.read_reg(&priv_data->i2c, CST816T_ADDRESS, FINGER_NUM, num);

	return 0;
}

/******************************************************************************
 * @brief	获取CST816T的动作信息，上下左右对应1234
 * @param	dev    :   CST816TDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_get_action(CST816TDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;
	
	uint8_t data[6];
	uint8_t gesture;
	uint16_t x, y;
	uint8_t retry_cnt = 0;

	while (retry_cnt < 5)
	{
		/* 读取寄存器 */
		priv_data->i2c.read_regs(&priv_data->i2c, CST816T_ADDRESS, GESTURE_ID, 6, data);

		gesture = data[0];
		x = ((data[2] & 0x0F) << 8) | data[3];
		y = ((data[4] & 0x0F) << 8) | data[5];

		/* 检查坐标范围并根据方向调整坐标与状态码 */
		if ((dev->info.dir == VERTICAL_FORWARD) && (x <= LCD_W && y <= LCD_H))
		{
			dev->x = x;
			dev->y = y - 8;

            switch(gesture)
			{
				case 1: dev->gesture = GESTURE_DOWN;	break;
				case 2: dev->gesture = GESTURE_UP;		break;
				case 3: dev->gesture = GESTURE_LEFT;	break;
				case 4: dev->gesture = GESTURE_RIGHT;	break;
			}
			return -2;
        }
		else if ((dev->info.dir == VERTICAL_REVERSE) && (x <= LCD_W && y <= LCD_H))
		{
			dev->x = LCD_W - x;
			dev->y = LCD_H - y;

			switch(gesture)
			{
				case 1: dev->gesture = GESTURE_UP;		break;
				case 2: dev->gesture = GESTURE_DOWN;	break;
				case 3: dev->gesture = GESTURE_RIGHT;	break;
				case 4: dev->gesture = GESTURE_LEFT;	break;
			}
			return -3;
        }
		else if ((dev->info.dir == HORIZONTAL_FORWARD) && (x <= LCD_H && y <= LCD_W))
		{
			dev->x = y;
			dev->y = LCD_W - x;

			switch(gesture)
			{
				case 1: dev->gesture = GESTURE_RIGHT;	break;
				case 2: dev->gesture = GESTURE_LEFT;	break;
				case 3: dev->gesture = GESTURE_DOWN;	break;
				case 4: dev->gesture = GESTURE_UP;		break;
			}
			return -4;
		}
		else if ((dev->info.dir == HORIZONTAL_REVERSE) && (x <= LCD_H && y <= LCD_W))
		{
			dev->x = LCD_H - y;
			dev->y = x;
			
			switch(gesture)
			{
				case 1: dev->gesture = GESTURE_LEFT;	break;
				case 2: dev->gesture = GESTURE_RIGHT;	break;
				case 3: dev->gesture = GESTURE_UP;		break;
				case 4: dev->gesture = GESTURE_DOWN;	break;
			}
			return -5;
        }

        /* 如果坐标超出范围，重置设备并重试 */
        __cst816t_io_write(dev->info.rst_port, dev->info.rst_pin, GPIO_LEVEL_LOW);
        delay_ms(10);
        __cst816t_io_write(dev->info.rst_port, dev->info.rst_pin, GPIO_LEVEL_HIGH);
        delay_ms(10);

		retry_cnt++;
	}

	return 0;
}

/******************************************************************************
 * @brief	去初始化CST816T
 * @param	dev   :  CST816TDev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_deinit(CST816TDev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}
