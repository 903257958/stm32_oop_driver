#include "cst816t.h"

#ifdef USE_STDPERIPH_DRIVER

#if defined(STM32F10X_HD) || defined(STM32F10X_MD)
	
#define	__cst816t_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);} \
										}

#define	__cst816t_config_io_out_pp(port, pin)	{	GPIO_InitTypeDef GPIO_InitStructure; \
											    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; \
											    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
											    	GPIO_InitStructure.GPIO_Pin = pin; \
											    	GPIO_Init(port, &GPIO_InitStructure); \
											    }
											
#define	__cst816t_io_write(port, pin, value)	GPIO_WriteBit(port, pin, (BitAction)value)

#elif defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F429_439xx)

#define	__cst816t_io_clock_enable(port)	{	if (port == GPIOA)		{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);} \
											else if (port == GPIOB)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);} \
											else if (port == GPIOC)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);} \
											else if (port == GPIOD)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);} \
											else if (port == GPIOE)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);} \
											else if (port == GPIOF)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);} \
											else if (port == GPIOG)	{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);} \
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

#endif

/* CST816T私有数据结构体 */
typedef struct {
	i2c_soft_dev_t i2c;	// 软件I2C设备
} cst816t_priv_data_t;

/* 函数声明 */
static int8_t __cst816t_get_id(cst816t_dev_t *dev, uint8_t *id);
static int8_t __cst816t_get_firmware_ver(cst816t_dev_t *dev, uint8_t *fw_ver);
static int8_t __cst816t_get_finger_num(cst816t_dev_t *dev);
static int8_t __cst816t_get_action(cst816t_dev_t *dev);
static int8_t __cst816t_deinit(cst816t_dev_t *dev);

/******************************************************************************
 * @brief	初始化CST816T
 * @param	dev	:  cst816t_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
int8_t cst816t_init(cst816t_dev_t *dev)
{
    if (!dev)
		return -1;

    /* 保存私有数据 */
	dev->priv_data = (cst816t_priv_data_t *)malloc(sizeof(cst816t_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	cst816t_priv_data_t *priv_data = (cst816t_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.config.scl_port = dev->config.scl_port;
	priv_data->i2c.config.scl_pin = dev->config.scl_pin;
	priv_data->i2c.config.sda_port = dev->config.sda_port;
	priv_data->i2c.config.sda_pin = dev->config.sda_pin;
	
	/* 配置软件I2C */
	i2c_soft_init(&priv_data->i2c);
	
	/* 配置时钟与GPIO */
	__cst816t_io_clock_enable(dev->config.rst_port);
	__cst816t_config_io_out_pp(dev->config.rst_port, dev->config.rst_pin);
    __cst816t_io_write(dev->config.rst_port, dev->config.rst_pin, GPIO_LEVEL_HIGH);

	/* 复位 */
    __cst816t_io_write(dev->config.rst_port, dev->config.rst_pin, GPIO_LEVEL_LOW);
    CST816T_DELAY_MS(10);
    __cst816t_io_write(dev->config.rst_port, dev->config.rst_pin, GPIO_LEVEL_HIGH);
    CST816T_DELAY_MS(50);

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
 * @param	dev		:  cst816t_dev_t 结构体指针
 * @param	id		:  ID
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __cst816t_get_id(cst816t_dev_t *dev, uint8_t *id)
{
	if (!dev || !dev->init_flag)
		return -1;

	cst816t_priv_data_t *priv_data = (cst816t_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.read_reg(&priv_data->i2c, CST816T_ADDRESS, CHIP_ID, id);

	return 0;
}

/******************************************************************************
 * @brief	CST816T读取固件版本号
 * @param	dev		:  cst816t_dev_t 结构体指针
 * @param	fw_ver	:  固件版本号
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __cst816t_get_firmware_ver(cst816t_dev_t *dev, uint8_t *fw_ver)
{
	if (!dev || !dev->init_flag)
		return -1;

	cst816t_priv_data_t *priv_data = (cst816t_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.read_reg(&priv_data->i2c, CST816T_ADDRESS, FW_VER, fw_ver);

	return 0;
}

/******************************************************************************
 * @brief	CST816T读取触摸手指个数
 * @param	dev	:  cst816t_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __cst816t_get_finger_num(cst816t_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t num;

	cst816t_priv_data_t *priv_data = (cst816t_priv_data_t *)dev->priv_data;

	priv_data->i2c.read_reg(&priv_data->i2c, CST816T_ADDRESS, FINGER_NUM, &num);

	dev->data.finger_num = num;

	return 0;
}

/******************************************************************************
 * @brief	获取CST816T的动作信息，上下左右对应1234
 * @param	dev    :   cst816t_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __cst816t_get_action(cst816t_dev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;

	cst816t_priv_data_t *priv_data = (cst816t_priv_data_t *)dev->priv_data;
	
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
		if ((dev->config.dir == VERTICAL_FORWARD) && (x <= LCD_W && y <= LCD_H))
		{
			dev->data.x = x;
			dev->data.y = y - 8;

            switch(gesture)
			{
				case 1: dev->data.gesture = GESTURE_DOWN;	break;
				case 2: dev->data.gesture = GESTURE_UP;		break;
				case 3: dev->data.gesture = GESTURE_LEFT;	break;
				case 4: dev->data.gesture = GESTURE_RIGHT;	break;
			}
			return -2;
        }
		else if ((dev->config.dir == VERTICAL_REVERSE) && (x <= LCD_W && y <= LCD_H))
		{
			dev->data.x = LCD_W - x;
			dev->data.y = LCD_H - y;

			switch(gesture)
			{
				case 1: dev->data.gesture = GESTURE_UP;		break;
				case 2: dev->data.gesture = GESTURE_DOWN;	break;
				case 3: dev->data.gesture = GESTURE_RIGHT;	break;
				case 4: dev->data.gesture = GESTURE_LEFT;	break;
			}
			return -3;
        }
		else if ((dev->config.dir == HORIZONTAL_FORWARD) && (x <= LCD_H && y <= LCD_W))
		{
			dev->data.x = y;
			dev->data.y = LCD_W - x;

			switch(gesture)
			{
				case 1: dev->data.gesture = GESTURE_RIGHT;	break;
				case 2: dev->data.gesture = GESTURE_LEFT;	break;
				case 3: dev->data.gesture = GESTURE_DOWN;	break;
				case 4: dev->data.gesture = GESTURE_UP;		break;
			}
			return -4;
		}
		else if ((dev->config.dir == HORIZONTAL_REVERSE) && (x <= LCD_H && y <= LCD_W))
		{
			dev->data.x = LCD_H - y;
			dev->data.y = x;
			
			switch(gesture)
			{
				case 1: dev->data.gesture = GESTURE_LEFT;	break;
				case 2: dev->data.gesture = GESTURE_RIGHT;	break;
				case 3: dev->data.gesture = GESTURE_UP;		break;
				case 4: dev->data.gesture = GESTURE_DOWN;	break;
			}
			return -5;
        }

        /* 如果坐标超出范围，重置设备并重试 */
        __cst816t_io_write(dev->config.rst_port, dev->config.rst_pin, GPIO_LEVEL_LOW);
        CST816T_DELAY_MS(10);
        __cst816t_io_write(dev->config.rst_port, dev->config.rst_pin, GPIO_LEVEL_HIGH);
        CST816T_DELAY_MS(10);

		retry_cnt++;
	}

	return 0;
}

/******************************************************************************
 * @brief	去初始化CST816T
 * @param	dev   :  cst816t_dev_t 结构体指针
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int8_t __cst816t_deinit(cst816t_dev_t *dev)
{    
    if (!dev || !dev->init_flag)
        return -1;
	
	cst816t_priv_data_t *priv_data = (cst816t_priv_data_t *)dev->priv_data;
	
	/* 去初始化软件I2C */
	priv_data->i2c.deinit(&priv_data->i2c);

	/* 释放私有数据内存 */
	free(dev->priv_data);
    dev->priv_data = NULL;

	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}
