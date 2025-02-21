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
// static void __cst816t_write_reg(CST816TDev_t *dev, uint8_t addr, uint8_t data);
static int __cst816t_read_reg(CST816TDev_t *dev, uint8_t addr, uint8_t *data);
static int __cst816t_read_regs(CST816TDev_t *dev, uint8_t addr, uint8_t num, uint8_t data[]);
static int __cst816t_get_id(CST816TDev_t *dev, uint8_t *id);
static int __cst816t_get_firmware_ver(CST816TDev_t *dev, uint8_t *fw_ver);
static int __cst816t_get_finger_num(CST816TDev_t *dev);
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
 * @brief	CST816T写寄存器
 * @param	dev		:  CST816TDev_t 结构体指针
 * @param	addr	:  要写入的寄存器地址
 * @param	data	:  要写入的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
// static int __cst816t_write_reg(CST816TDev_t *dev, uint8_t addr, uint8_t data)
// {
// 	if (!dev || !dev->init_flag)
// 		return -1;

//     CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;

//     priv_data->i2c.start(&priv_data->i2c);						// I2C起始
// 	priv_data->i2c.send_byte(&priv_data->i2c, CST816T_ADDRESS);	// 发送从机地址，读写位为0，表示即将写入
// 	priv_data->i2c.recv_ack(&priv_data->i2c);					// 接收应答
// 	priv_data->i2c.send_byte(&priv_data->i2c, addr);	        // 发送寄存器地址
// 	priv_data->i2c.recv_ack(&priv_data->i2c);					// 接收应答
// 	priv_data->i2c.send_byte(&priv_data->i2c, data);			// 发送要写入寄存器的数据
// 	priv_data->i2c.recv_ack(&priv_data->i2c);					// 接收应答
// 	priv_data->i2c.stop(&priv_data->i2c);						// I2C终止

// 	return 0;
// }

/******************************************************************************
 * @brief	CST816T读寄存器
 * @param	dev		:   CST816TDev_t 结构体指针
 * @param	addr    :   要读的寄存器地址
 * @param	data    :   要读的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_read_reg(CST816TDev_t *dev, uint8_t addr, uint8_t *data)
{
	if (!dev || !dev->init_flag)
		return -1;
	
    CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;
	
	priv_data->i2c.start(&priv_data->i2c);								// I2C起始
	priv_data->i2c.send_byte(&priv_data->i2c, CST816T_ADDRESS);			// 发送从机地址，读写位为0，表示即将写入
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	priv_data->i2c.send_byte(&priv_data->i2c, addr);					// 发送寄存器地址
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	
	priv_data->i2c.start(&priv_data->i2c);								// I2C重复起始
	priv_data->i2c.send_byte(&priv_data->i2c, CST816T_ADDRESS | 0x01);	// 发送从机地址，读写位为1，表示即将读取
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	*data = priv_data->i2c.recv_byte(&priv_data->i2c);					// 接收指定寄存器的数据
	priv_data->i2c.send_ack(&priv_data->i2c, 1);						// 发送应答，给从机非应答，终止从机的数据输出
	priv_data->i2c.stop(&priv_data->i2c);								// I2C终止		
	
	return 0;
}

/******************************************************************************
 * @brief	CST816T读多个寄存器
 * @param	dev    	:   CST816TDev_t 结构体指针
 * @param	addr    :   要读的寄存器首地址
 * @param	num		:   要读的寄存器个数
 * @param	data    :   要读的寄存器数据
 * @return	0, 表示成功, 其他值表示失败
 ******************************************************************************/
static int __cst816t_read_regs(CST816TDev_t *dev, uint8_t addr, uint8_t num, uint8_t data[])
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t i;

    CST816TPrivData_t *priv_data = (CST816TPrivData_t *)dev->priv_data;
	
	priv_data->i2c.start(&priv_data->i2c);							// I2C起始
	priv_data->i2c.send_byte(&priv_data->i2c, CST816T_ADDRESS);		// 发送从机地址，读写位为0，表示即将写入
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	priv_data->i2c.send_byte(&priv_data->i2c, addr);					// 发送寄存器地址
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答
	
	priv_data->i2c.start(&priv_data->i2c);							// I2C重复起始
	priv_data->i2c.send_byte(&priv_data->i2c, CST816T_ADDRESS | 0x01);// 发送从机地址，读写位为1，表示即将读取
	priv_data->i2c.recv_ack(&priv_data->i2c);							// 接收应答

	for (i = 0; i < num; i++)
    {
        data[i] = priv_data->i2c.recv_byte(&priv_data->i2c);			// 接收数据
        if (i == num - 1)
        {
            priv_data->i2c.send_ack(&priv_data->i2c, 1);				// 发送非应答信号
        }
        else
        {
            priv_data->i2c.send_ack(&priv_data->i2c, 0);				// 发送应答信号
        }
    }

	priv_data->i2c.stop(&priv_data->i2c);								// I2C终止
	
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
	
    __cst816t_read_reg(dev, CHIP_ID, id);

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
	
    __cst816t_read_reg(dev, FW_VER, fw_ver);

	return 0;
}

/******************************************************************************
 * @brief	CST816T读取触摸手指个数
 * @param	dev	:  CST816TDev_t 结构体指针
 * @return	触摸手指个数，-1表示失败
 ******************************************************************************/
static int __cst816t_get_finger_num(CST816TDev_t *dev)
{
	if (!dev || !dev->init_flag)
		return -1;
	
	uint8_t num;

    __cst816t_read_reg(dev, FINGER_NUM, &num);

	return num;
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
	
	uint8_t data[6];
	uint8_t gesture;
	uint16_t x, y;
	uint8_t retryCnt = 0;

	while (retryCnt < 5)
	{
		/* 读取寄存器 */
		__cst816t_read_regs(dev, GESTURE_ID, 6, data);

		gesture = data[0];
		x = ((data[2] & 0x0F) << 8) | data[3];
		y = ((data[4] & 0x0F) << 8) | data[5];

		/* 检查坐标范围并根据方向调整坐标与状态码 */
		if ((dev->info.dir == VERTICAL_FORWARD) && (x <= LCD_W && y <= LCD_H))
		{
			dev->x = x;
			dev->y = y - 5;

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

		retryCnt++;
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
	
	dev->init_flag = false;	// 修改初始化标志
    
    return 0;
}
