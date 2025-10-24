#include "oled.h"
#include "oled_data.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 最大OLED设备数量 */
#define MAX_OLED_NUM 1

/*
 * OLED显存数组
 * 所有的显示函数，都只是对此显存数组进行读写
 * 随后调用oled_update函数或oled_update_area函数
 * 才会将显存数组的数据发送到OLED硬件，进行显示
 */
static uint8_t g_oled_diaplay_buf[MAX_OLED_NUM][8][128];			
						
/* 全局变量用于给注册的OLED设备分配索引 */
static uint8_t g_index = 0;

/* OLED私有数据 */
typedef struct {
	i2c_soft_dev_t i2c;
	uint8_t index;
} oled_priv_data_t;
				
/* 通信协议 */
static int oled_write_command(oled_dev_t *dev, uint8_t cmd);
static int oled_write_data(oled_dev_t *dev, uint8_t *data, uint8_t cnt);

/* 功能函数 */
static int oled_update(oled_dev_t *dev);
static int oled_update_area(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
static int oled_clear(oled_dev_t *dev);
static int oled_clear_area(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
static int oled_reverse(oled_dev_t *dev);
static int oled_reverse_area(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
static int oled_show_image(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *img);
static int oled_show_char(oled_dev_t *dev, uint8_t x, uint8_t y, char chr, uint8_t size);
static int oled_show_string(oled_dev_t *dev, uint8_t x, uint8_t y, char *str, uint8_t size);
static int oled_show_num(oled_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
static int oled_show_signed_num(oled_dev_t *dev, uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size);
static int oled_show_hex_num(oled_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
static int oled_show_bin_num(oled_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
static int oled_show_float_num(oled_dev_t *dev, uint8_t x, uint8_t y, double num, uint8_t int_len, uint8_t fra_len, uint8_t size);
static int oled_show_chinese(oled_dev_t *dev, uint8_t x, uint8_t y, char *chinese);
static int oled_printf(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t size, char *format, ...);
static int oled_draw_point(oled_dev_t *dev, uint8_t x, uint8_t y);
static int oled_get_point(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t *point);
static int oled_draw_line(oled_dev_t *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
static int oled_draw_rectangle(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t is_filled);
static int oled_draw_triangle(oled_dev_t *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t is_filled);
static int oled_draw_circle(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t is_filled);
static int oled_draw_ellipse(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t is_filled);
static int oled_draw_arc(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t start_angle, int16_t end_angle, uint8_t is_filled);
static int oled_drv_deinit(oled_dev_t *dev);

/**
 * @brief   初始化 OLED 设备
 * @param[in,out] dev oled_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int oled_drv_init(oled_dev_t *dev)
{
	if (!dev)
		return -1;
	
	/* 初始化前加入适量延时，待供电稳定 */
	for (uint32_t i = 0; i < 1000; i ++)
		for (uint32_t j = 0; j < 1000; j ++);
	
	dev->priv_data = (oled_priv_data_t *)malloc(sizeof(oled_priv_data_t));
	if (!dev->priv_data)
		return -1;
	
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	
	priv_data->i2c.config.scl_port = dev->config.scl_port;
	priv_data->i2c.config.scl_pin = dev->config.scl_pin;
	priv_data->i2c.config.sda_port = dev->config.sda_port;
	priv_data->i2c.config.sda_pin = dev->config.sda_pin;
	priv_data->index = g_index++;
	
	i2c_soft_drv_init(&priv_data->i2c);
	
	dev->update = oled_update;
	dev->update_area = oled_update_area;
	dev->clear = oled_clear;
	dev->clear_area = oled_clear_area;
	dev->reverse = oled_reverse;
	dev->reverse_area = oled_reverse_area;
	dev->show_image = oled_show_image;
	dev->show_char = oled_show_char;
	dev->show_string = oled_show_string;
	dev->show_num = oled_show_num;
	dev->show_signed_num = oled_show_signed_num;
	dev->show_hex_num = oled_show_hex_num;
	dev->show_bin_num = oled_show_bin_num;
	dev->show_float_num = oled_show_float_num;
	dev->show_chinese = oled_show_chinese;
	dev->printf = oled_printf;
	dev->draw_point = oled_draw_point;
	dev->get_point = oled_get_point;
	dev->draw_line = oled_draw_line;
	dev->draw_rectangle = oled_draw_rectangle;
	dev->draw_triangle = oled_draw_triangle;
	dev->draw_circle = oled_draw_circle;
	dev->draw_ellipse = oled_draw_ellipse;
	dev->draw_arc = oled_draw_arc;
	dev->deinit = oled_drv_deinit;
	
	dev->init_flag = true;
	
	/* 配置 OLED */
	oled_write_command(dev, 0xAE);	// 设置显示开启/关闭，0xAE关闭，0xAF开启
	
	oled_write_command(dev, 0xD5);	// 设置显示时钟分频比/振荡器频率
	oled_write_command(dev, 0x80);	// 0x00~0xFF
	
	oled_write_command(dev, 0xA8);	// 设置多路复用率
	oled_write_command(dev, 0x3F);	// 0x0E~0x3F
	
	oled_write_command(dev, 0xD3);	// 设置显示偏移
	oled_write_command(dev, 0x00);	// 0x00~0x7F
	
	oled_write_command(dev, 0x40);	// 设置显示开始行，0x40~0x7F
	
	if (!OLED_DIRECTION) {
		oled_write_command(dev, 0xA1);	// 设置左右方向，0xA1正常，0xA0左右反置
		oled_write_command(dev, 0xC8);	// 设置上下方向，0xC8正常，0xC0上下反置
	} else {
		oled_write_command(dev, 0xA0);	// 设置左右方向，0xA1正常，0xA0左右反置
		oled_write_command(dev, 0xC0);	// 设置上下方向，0xC8正常，0xC0上下反置
	}
	
	oled_write_command(dev, 0xDA);	// 设置COM引脚硬件配置
	oled_write_command(dev, 0x12);
	
	oled_write_command(dev, 0x81);	// 设置对比度
	oled_write_command(dev, 0xCF);	// 0x00~0xFF
	
	oled_write_command(dev, 0xD9);	// 设置预充电周期
	oled_write_command(dev, 0xF1);
	
	oled_write_command(dev, 0xDB);	// 设置VCOMH取消选择级别
	oled_write_command(dev, 0x30);
	
	oled_write_command(dev, 0xA4);	// 设置整个显示打开/关闭
	
	oled_write_command(dev, 0xA6);	// 设置正常/反色显示，0xA6正常，0xA7反色
	
	oled_write_command(dev, 0x8D);	// 设置充电泵
	oled_write_command(dev, 0x14);

	oled_write_command(dev, 0xAF);	// 开启显示
	
	oled_clear(dev);				// 清空显存数组
	oled_update(dev);				// 更新显示，清屏，防止初始化后未显示内容时花屏

	return 0;
}

/* 通信协议 *********************************************************************/

/**
 * @brief   OLED 写命令
 * @param[in] dev oled_dev_t 结构体指针
 * @param[in] cmd 要写入的命令值，范围：0x00~0xFF
 * @return	0 表示成功，其他值表示失败
 */
static int oled_write_command(oled_dev_t *dev, uint8_t cmd)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t ack;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	priv_data->i2c.start(&priv_data->i2c);
	priv_data->i2c.send_byte(&priv_data->i2c, 0x78);	// 发送OLED的I2C从机地址
	priv_data->i2c.recv_ack(&priv_data->i2c, &ack);
	priv_data->i2c.send_byte(&priv_data->i2c, 0x00);	// 控制字节，给0x00，表示即将写命令
	priv_data->i2c.recv_ack(&priv_data->i2c, &ack);
	priv_data->i2c.send_byte(&priv_data->i2c, cmd);		// 写入指定的命令
	priv_data->i2c.recv_ack(&priv_data->i2c, &ack);
	priv_data->i2c.stop(&priv_data->i2c);
	
	return 0;
}

/**
 * @brief   OLED 写数据
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] data 要写入数据的起始地址
 * @param[in] cnt  要写入数据的数量
 * @return	0 表示成功，其他值表示失败
 */
static int oled_write_data(oled_dev_t *dev, uint8_t *data, uint8_t cnt)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t i;
	uint8_t ack;

	if (!dev || !dev->init_flag)
		return -1;
	
	priv_data->i2c.start(&priv_data->i2c);
	priv_data->i2c.send_byte(&priv_data->i2c, 0x78);		// 发送OLED的I2C从机地址
	priv_data->i2c.recv_ack(&priv_data->i2c, &ack);
	priv_data->i2c.send_byte(&priv_data->i2c, 0x40);		// 控制字节，给0x40，表示即将写数量
	priv_data->i2c.recv_ack(&priv_data->i2c, &ack);
			
	/* 循环count次，进行连续的数据写入 */
	for (i = 0; i < cnt; i ++) {
		priv_data->i2c.send_byte(&priv_data->i2c, data[i]);	// 依次发送data的每一个数据
		priv_data->i2c.recv_ack(&priv_data->i2c, &ack);
	}
	priv_data->i2c.stop(&priv_data->i2c);
	
	return 0;
}

/********************************************************************** 通信协议 */

/* 硬件配置 **********************************************************************/

/**
 * @brief   OLED 设置显示光标位置
 * @details OLED 默认的y轴，只能8Bit为一组写入，即1页等于8个y轴坐标
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] page 指定光标所在的页，范围：0~7
 * @param[in] x    指定光标所在的x轴坐标，范围：0~127
 * @return	0 表示成功，其他值表示失败
 */
static int oled_set_cursor(oled_dev_t *dev, uint8_t page, uint8_t x)
{	
	if (!dev || !dev->init_flag)
		return -1;
	
	/*
	 * 如果使用此程序驱动1.3寸的OLED显示屏，则需要解除此注释
	 * 因为1.3寸的OLED驱动芯片（SH1106）有132列
	 * 屏幕的起始列接在了第2列，而不是第0列
	 * 所以需要将x加2，才能正常显示
	 */
//	x += 2;
	
	/* 通过指令设置页地址和列地址 */
	oled_write_command(dev, 0xB0 | page);				// 设置页位置
	oled_write_command(dev, 0x10 | ((x & 0xF0) >> 4));	// 设置x位置高4位
	oled_write_command(dev, 0x00 | (x & 0x0F));			// 设置x位置低4位
	
	return 0;
}

/********************************************************************** 硬件配置 */

/* 工具函数 **********************************************************************/

/**
 * @brief   次方函数
 * @param[in] x 底数
 * @param[in] y 指数
 * @return	x^y
 */
static uint32_t oled_pow(uint32_t x, uint32_t y)
{
	uint32_t result = 1;	// 结果默认为1

	while (y--)				// 累乘y次
		result *= x;		// 每次把x累乘到结果上

	return result;
}

/**
 * @brief   判断指定点是否在指定多边形内部
 * @param[in] nvert  多边形的顶点数
 * @param[in] vertx  包含多边形顶点的x坐标的数组
 * @param[in] verty  包含多边形顶点的y坐标的数组
 * @param[in] testx  测试点的x坐标
 * @param[in] testy  测试点的y坐标
 * @return	指定点是否在指定多边形内部，1 在内部，0 不在内部
 */
static uint8_t oled_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
	int16_t i, j, c = 0;
	
	/*
	 * 此算法由W. Randolph Franklin提出
	 * 参考链接：https://wrfranklin.org/Research/Short_Notes/pnpoly.html
	 */
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
			c = !c;

	return c;
}

/**
 * @brief   判断指定点是否在指定角度内部
 * @param[in] x           指定点的x坐标
 * @param[in] y           指定点的y坐标
 * @param[in] start_angle 起始角度，范围：-180~180
 * @param[in] end_angle   终止角度，范围：-180~180
 * 						  水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
 * @param[in] testx  测试点的x坐标
 * @param[in] testy  测试点的y坐标
 * @return	指定点是否在指定角度内部，1 在内部，0 不在内部
 */
static uint8_t oled_is_in_angle(int16_t x, int16_t y, int16_t start_angle, int16_t end_angle)
{
	int16_t point_angle;
	point_angle = atan2(y, x) / 3.14 * 180;	// 计算指定点的弧度，并转换为角度表示

	if (start_angle < end_angle) {
		/* 如果指定角度在起始终止角度之间，则判定指定点在指定角度 */
		if (point_angle >= start_angle && point_angle <= end_angle)
			return 1;
	} else {
		/* 如果指定角度大于起始角度或者小于终止角度，则判定指定点在指定角度 */
		if (point_angle >= start_angle || point_angle <= end_angle)
			return 1;
	}

	return 0;	// 不满足以上条件，则判断判定指定点不在指定角度
}

/********************************************************************** 工具函数 */

/* 功能函数 **********************************************************************/

/**
 * @brief   将 OLED 显存数组更新到 OLED 屏幕
 * @param[in] dev  oled_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int oled_update(oled_dev_t *dev)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t j;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 遍历每一页 */
	for (j = 0; j < 8; j ++) {
		/* 设置光标位置为每一页的第一列 */
		oled_set_cursor(dev, j, 0);
		/* 连续写入128个数据，将显存数组的数据写入到OLED硬件 */
		oled_write_data(dev, g_oled_diaplay_buf[priv_data->index][j], 128);
	}
	
	return 0;
}

/**
 * @brief   将 OLED 显存数组更新到 OLED 屏幕
 * @details 此函数会至少更新参数指定的区域，如果更新区域y轴只包含部分页，
 * 			则同一页的剩余部分会跟随一起更新
 * @param[in] dev    oled_dev_t 结构体指针
 * @param[in] x      指定区域左上角的横坐标，范围：0~127
 * @param[in] y	     指定区域左上角的纵坐标，范围：0~63
 * @param[in] width  指定区域的宽度，范围：0~128
 * @param[in] height 指定区域的高度，范围：0~64
 * @return	0 表示成功，其他值表示失败
 */
static int oled_update_area(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t j;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (x > 127 || y > 63)
		return -1;
	if (x + width > 128)
		width = 128 - x;
	if (y + height > 64)
		height = 64 - y;
	
	/* 遍历指定区域涉及的相关页 */
	/* (y + height - 1) / 8 + 1的目的是(y + height) / 8并向上取整 */
	for (j = y / 8; j < (y + height - 1) / 8 + 1; j ++) {
		/* 设置光标位置为相关页的指定列 */
		oled_set_cursor(dev, j, x);
		/* 连续写入width个数据，将显存数组的数据写入到OLED硬件 */
		oled_write_data(dev, &g_oled_diaplay_buf[priv_data->index][j][x], width);
	}
	
	return 0;
}

/**
 * @brief   将 OLED 显存数组全部清零
 * @param[in] dev    oled_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int oled_clear(oled_dev_t *dev)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t i, j;

	if (!dev || !dev->init_flag)
		return -1;
	
	for (j = 0; j < 8; j ++)				// 遍历8页
		for (i = 0; i < 128; i ++)			// 遍历128列
			g_oled_diaplay_buf[priv_data->index][j][i] = 0x00;	// 将显存数组数据全部清零
	
	return 0;
}

/**
 * @brief   将 OLED 显存数组部分清零
 * @param[in] dev    oled_dev_t 结构体指针
 * @param[in] x      指定区域左上角的横坐标，范围：0~127
 * @param[in] y	     指定区域左上角的纵坐标，范围：0~63
 * @param[in] width  指定区域的宽度，范围：0~128
 * @param[in] height 指定区域的高度，范围：0~64
 * @return	0 表示成功，其他值表示失败
 */
static int oled_clear_area(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t i, j;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (x > 127 || y > 63)
		return -1;
	if (x + width > 128)
		width = 128 - x;
	if (y + height > 64)
		height = 64 - y;
	
	for (j = y; j < y + height; j ++)		// 遍历指定页
		for (i = x; i < x + width; i ++)	// 遍历指定列
			g_oled_diaplay_buf[priv_data->index][j / 8][i] &= ~(0x01 << (j % 8));	// 将显存数组指定数据清零
	
	return 0;
}

/**
 * @brief   将 OLED 显存数组全部取反
 * @param[in] dev    oled_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int oled_reverse(oled_dev_t *dev)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t i, j;

	if (!dev || !dev->init_flag)
		return -1;
	
	for (j = 0; j < 8; j ++)				// 遍历8页
		for (i = 0; i < 128; i ++)			// 遍历128列
			g_oled_diaplay_buf[priv_data->index][j][i] ^= 0xFF;	// 将显存数组数据全部取反
	
	return 0;
}

/**
 * @brief   将 OLED 显存数组部分取反
 * @param[in] dev    oled_dev_t 结构体指针
 * @param[in] x      指定区域左上角的横坐标，范围：0~127
 * @param[in] y	     指定区域左上角的纵坐标，范围：0~63
 * @param[in] width  指定区域的宽度，范围：0~128
 * @param[in] height 指定区域的高度，范围：0~64
 * @return	0 表示成功，其他值表示失败
 */
static int oled_reverse_area(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t i, j;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (x > 127 || y > 63)
		return -1;
	if (x + width > 128)
		width = 128 - x;
	if (y + height > 64)
		height = 64 - y;
	
	for (j = y; j < y + height; j ++)		// 遍历指定页
		for (i = x; i < x + width; i ++)	// 遍历指定列
			g_oled_diaplay_buf[priv_data->index][j / 8][i] ^= 0x01 << (j % 8);	// 将显存数组指定数据取反
	
	return 0;
}

/**
 * @brief   OLED 显示图像
 * @param[in] dev    oled_dev_t 结构体指针
 * @param[in] x      指定区域左上角的横坐标，范围：0~127
 * @param[in] y	     指定区域左上角的纵坐标，范围：0~63
 * @param[in] width  指定区域的宽度，范围：0~128
 * @param[in] height 指定区域的高度，范围：0~64
 * @param[in] img    指定要显示的图像
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_image(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *img)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	uint8_t i, j;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 参数检查，保证指定区域不会超出屏幕范围 */
	if (x > 127 || y > 63)
		return -1;
	
	oled_clear_area(dev, x, y, width, height);	/* 将图像所在区域清空 */
	
	/* 
	 * 遍历指定图像涉及的相关页
	 * (height - 1) / 8 + 1的目的是height / 8并向上取整 
	 */
	for (j = 0; j < (height - 1) / 8 + 1; j ++) {
		/* 遍历指定图像涉及的相关列 */
		for (i = 0; i < width; i ++) {
			/* 超出边界，则跳过显示 */
			if (x + i > 127)
				break;
			if (y / 8 + j > 7)
				return -1;
			
			/* 显示图像在当前页的内容 */
			g_oled_diaplay_buf[priv_data->index][y / 8 + j][x + i] |= img[j * width + i] << (y % 8);
			
			/* 
			 * 超出边界，则跳过显示
			 * 使用continue的目的是，下一页超出边界时，上一页的后续内容还需要继续显示 
			 */
			if (y / 8 + j + 1 > 7)
				continue;
			
			/* 显示图像在下一页的内容 */
			g_oled_diaplay_buf[priv_data->index][y / 8 + j + 1][x + i] |= img[j * width + i] >> (8 - y % 8);
		}
	}
	
	return 0;
}

/**
 * @brief   OLED 显示一个字符
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] x    指定字符左上角的横坐标，范围：0~127
 * @param[in] y	   指定字符左上角的纵坐标，范围：0~63
 * @param[in] chr  指定要显示的字符，范围：ASCII码可见字符
 * @param[in] size 指定字体大小: OLED_8X16, OLED_6X8
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_char(oled_dev_t *dev, uint8_t x, uint8_t y, char chr, uint8_t size)
{	
	if (!dev || !dev->init_flag)
		return -1;
	
	if (size == OLED_8X16)
		oled_show_image(dev, x, y, 8, 16, oled_f8x16[chr - ' ']);
	else if(size == OLED_6X8)
		oled_show_image(dev, x, y, 6, 8, oled_f6x8[chr - ' ']);
	
	return 0;
}

/**
 * @brief   OLED 显示字符串
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] x    指定字符串左上角的横坐标，范围：0~127
 * @param[in] y	   指定字符串左上角的纵坐标，范围：0~63
 * @param[in] str  指定要显示的字符串，范围：ASCII码可见字符组成的字符串
 * @param[in] size 指定字体大小: OLED_8X16, OLED_6X8
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_string(oled_dev_t *dev, uint8_t x, uint8_t y, char *str, uint8_t size)
{	
	uint8_t i;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 遍历字符串的每个字符 */
	for (i = 0; str[i] != '\0'; i++)
		oled_show_char(dev, x + i * size, y, str[i], size);
	
	return 0;
}

/**
 * @brief   OLED 显示数字（十进制，正整数）
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] x    指定数字左上角的横坐标，范围：0~127
 * @param[in] y	   指定数字左上角的纵坐标，范围：0~63
 * @param[in] num  指定要显示的数字，范围：0~4294967295
 * @param[in] len  指定数字的长度，范围：0~10
 * @param[in] size 指定字体大小: OLED_8X16, OLED_6X8
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_num(oled_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{	
	uint8_t i;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 
	 * 遍历数字的每一位
	 * num / oled_pow(10, len - i - 1) % 10 可以十进制提取数字的每一位
	 * + '0' 可将数字转换为字符格式
	 */
	for (i = 0; i < len; i++)
		oled_show_char(dev, x + i * size, y, num / oled_pow(10, len - i - 1) % 10 + '0', size);
	
	return 0;
}

/**
 * @brief   OLED 显示有符号数字（十进制，整数）
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] x    指定数字左上角的横坐标，范围：0~127
 * @param[in] y	   指定数字左上角的纵坐标，范围：0~63
 * @param[in] num  指定要显示的数字，范围：-2147483648~2147483647
 * @param[in] len  指定数字的长度，范围：0~10
 * @param[in] size 指定字体大小: OLED_8X16, OLED_6X8
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_signed_num(oled_dev_t *dev, uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size)
{
	uint8_t i;
	uint32_t num1;

	if (!dev || !dev->init_flag)
		return -1;
	
	if (num >= 0) {
		oled_show_char(dev, x, y, '+', size);	// 显示+号
		num1 = num;								// num1直接等于num
	} else {
		oled_show_char(dev, x, y, '-', size);	// 显示-号
		num1 = -num;							// num1等于num取负
	}

	/* 
	 * 遍历数字的每一位
	 * num1 / oled_pow(10, len - i - 1) % 10 可以十进制提取数字的每一位
	 * + '0' 可将数字转换为字符格式
	 */
	for (i = 0; i < len; i++)
		oled_show_char(dev, x + (i + 1) * size, y, num1 / oled_pow(10, len - i - 1) % 10 + '0', size);
	
	return 0;
}

/**
 * @brief   OLED 显示十六进制数字（十六进制，正整数）
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] x    指定数字左上角的横坐标，范围：0~127
 * @param[in] y	   指定数字左上角的纵坐标，范围：0~63
 * @param[in] num  指定要显示的数字，范围：0x00000000~0xFFFFFFFF
 * @param[in] len  指定数字的长度，范围：0~8
 * @param[in] size 指定字体大小: OLED_8X16, OLED_6X8
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_hex_num(oled_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{	
	uint8_t i, single_num;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 遍历数字的每一位 */
	for (i = 0; i < len; i++) {
		/* 以十六进制提取数字的每一位 */
		single_num = num / oled_pow(16, len - i - 1) % 16;
		
		if (single_num < 10) {
			/* + '0' 可将数字转换为字符格式 */
			oled_show_char(dev, x + i * size, y, single_num + '0', size);
		} else {
			/* + 'A' 可将数字转换为从A开始的十六进制字符 */
			oled_show_char(dev, x + i * size, y, single_num - 10 + 'A', size);
		}
	}
	
	return 0;
}

/**
 * @brief   OLED 显示二进制数字（二进制，正整数）
 * @param[in] dev  oled_dev_t 结构体指针
 * @param[in] x    指定数字左上角的横坐标，范围：0~127
 * @param[in] y	   指定数字左上角的纵坐标，范围：0~63
 * @param[in] num  指定要显示的数字，范围：0x00000000~0xFFFFFFFF
 * @param[in] len  指定数字的长度，范围：0~16
 * @param[in] size 指定字体大小: OLED_8X16, OLED_6X8
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_bin_num(oled_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{	
	uint8_t i;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 
	 * 遍历数字的每一位
	 * num / oled_pow(2, len - i - 1) % 2 可以二进制提取数字的每一位
	 * + '0' 可将数字转换为字符格式 
	 */
	for (i = 0; i < len; i++)
		oled_show_char(dev, x + i * size, y, num / oled_pow(2, len - i - 1) % 2 + '0', size);
	
	return 0;
}

/**
 * @brief   OLED 显示浮点数
 * @param[in] dev     oled_dev_t 结构体指针
 * @param[in] x       指定数字左上角的横坐标，范围：0~127
 * @param[in] y	      指定数字左上角的纵坐标，范围：0~63
 * @param[in] num     指定要显示的数字，范围：-4294967295.0~4294967295.0
 * @param[in] int_len 指定数字的整数位长度，范围：0~10
 * @param[in] fra_len 指定数字的小数位长度，范围：0~9，小数进行四舍五入显示
 * @param[in] size    指定字体大小: OLED_8X16, OLED_6X8
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_float_num(oled_dev_t *dev, uint8_t x, uint8_t y, double num, uint8_t int_len, uint8_t fra_len, uint8_t size)
{	
	uint32_t pow_num, int_num, fra_num;

	if (!dev || !dev->init_flag)
		return -1;
	
	if (num >= 0) {
		oled_show_char(dev, x, y, '+', size);	// 显示+号
	} else {
		oled_show_char(dev, x, y, '-', size);	// 显示-号
		num = -num;								// num取负
	}
	
	/* 提取整数部分和小数部分 */
	int_num = num;						// 直接赋值给整型变量，提取整数
	num -= int_num;						// 将num的整数减掉，防止之后将小数乘到整数时因数过大造成错误
	pow_num = oled_pow(10, fra_len);	// 根据指定小数的位数，确定乘数
	fra_num = round(num * pow_num);		// 将小数乘到整数，同时四舍五入，避免显示误差
	int_num += fra_num / pow_num;		// 若四舍五入造成了进位，则需要再加给整数
	
	oled_show_num(dev, x + size, y, int_num, int_len, size);					/* 显示整数部分 */
	oled_show_char(dev, x + (int_len + 1) * size, y, '.', size);				/* 显示小数点 */
	oled_show_num(dev, x + (int_len + 2) * size, y, fra_num, fra_len, size);	/* 显示小数部分 */
	
	return 0;
}

/**
 * @brief   OLED 显示汉字串
 * @param[in] dev     oled_dev_t 结构体指针
 * @param[in] x       指定汉字串左上角的横坐标，范围：0~127
 * @param[in] y	      指定汉字串左上角的纵坐标，范围：0~63
 * @param[in] chinese 指定要显示的汉字串，范围：必须全部为汉字或者全角字符，不要加入任何半角字符
 * 					  显示的汉字需要在OLED_data.c里的oled_cf16x16数组定义
 *					  未找到指定汉字时，会显示默认图形（一个方框，内部一个问号）
 * @return	0 表示成功，其他值表示失败
 */
static int oled_show_chinese(oled_dev_t *dev, uint8_t x, uint8_t y, char *chinese)
{	
	uint8_t p_chinese = 0;
	uint8_t p_index;
	uint8_t i;
	char single_chinese[OLED_CHN_CHAR_WIDTH + 1] = {0};

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 遍历汉字串 */
	for (i = 0; chinese[i] != '\0'; i ++) {
		single_chinese[p_chinese] = chinese[i];	// 提取汉字串数据到单个汉字数组
		p_chinese++;							// 计次自增
		
		/* 当提取次数到达OLED_CHN_CHAR_WIDTH时，即代表提取到了一个完整的汉字 */
		if (p_chinese >= OLED_CHN_CHAR_WIDTH) {
			p_chinese = 0;		//计次归零
			
			/* 
			 * 遍历整个汉字字模库，寻找匹配的汉字
			 * 如果找到最后一个汉字（定义为空字符串），则表示汉字未在字模库定义，停止寻找 
			 */
			for (p_index = 0; strcmp(oled_cf16x16[p_index].index, "") != 0; p_index++) {
				/* 找到匹配的汉字 */
				if (strcmp(oled_cf16x16[p_index].index, single_chinese) == 0)
					break;		// 跳出循环，此时p_index的值为指定汉字的索引
			}
			
			/* 将汉字字模库oled_cf16x16的指定数据以16*16的图像格式显示 */
			oled_show_image(dev, x + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 16, y, 16, 16, 
							oled_cf16x16[p_index].data);
		}
	}
	
	return 0;
}

/**
 * @brief   OLED 使用 printf 函数打印格式化字符串
 * @param[in] dev    oled_dev_t 结构体指针
 * @param[in] x      指定格式化字符串左上角的横坐标，范围：0~127
 * @param[in] y	     指定格式化字符串左上角的纵坐标，范围：0~63
 * @param[in] size   指定字体大小: OLED_8X16, OLED_6X8
 * @param[in] format 指定要显示的格式化字符串，范围：ASCII码可见字符组成的字符串
 * @return	0 表示成功，其他值表示失败
 */
static int oled_printf(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t size, char *format, ...)
{	
	char str[30];							// 定义字符数组
	va_list arg;							// 定义可变参数列表数据类型的变量arg

	if (!dev || !dev->init_flag)
		return -1;
	
	va_start(arg, format);					// 从format开始，接收参数列表到arg变量
	vsprintf(str, format, arg);				// 使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);							// 结束变量arg
	oled_show_string(dev, x, y, str, size);	// OLED显示字符数组（字符串）
	
	return 0;
}

/**
 * @brief   OLED 在指定位置画一个点
 * @param[in] dev oled_dev_t 结构体指针
 * @param[in] x   指定点的横坐标，范围：0~127
 * @param[in] y   指定点的纵坐标，范围：0~63
 * @return	0 表示成功，其他值表示失败
 */
static int oled_draw_point(oled_dev_t *dev, uint8_t x, uint8_t y)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	/* 参数检查，保证指定位置不会超出屏幕范围 */
	if (x > 127 || y > 63)
		return -1;
	
	/* 将显存数组指定位置的一个Bit数据置1 */
	g_oled_diaplay_buf[priv_data->index][y / 8][x] |= 0x01 << (y % 8);
	
	return 0;
}

/**
 * @brief   OLED 获取指定位置点的值
 * @param[in]  dev   oled_dev_t 结构体指针
 * @param[in]  x     指定点的横坐标，范围：0~127
 * @param[in]  y     指定点的纵坐标，范围：0~63
 * @param[out] point 指定位置点是否处于点亮状态，1 点亮，0 熄灭
 * @return	0 表示成功，其他值表示失败
 */
static int oled_get_point(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t *point)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 参数检查，保证指定位置不会超出屏幕范围 */
	if (x > 127 || y > 63)
		return -1;
	
	if (g_oled_diaplay_buf[priv_data->index][y / 8][x] & 0x01 << (y % 8))
		*point = 1;
	else
		*point = 0;

	return 0;
}

/**
 * @brief   OLED 画线
 * @param[in] dev oled_dev_t 结构体指针
 * @param[in] x0  指定一个端点的横坐标，范围：0~127
 * @param[in] y0  指定一个端点的纵坐标，范围：0~63
 * @param[in] x1  指定另一个端点的横坐标，范围：0~127
 * @param[in] y1  指定另一个端点的纵坐标，范围：0~63
 * @return	0 表示成功，其他值表示失败
 */
static int oled_draw_line(oled_dev_t *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{	
	int16_t x, y, dx, dy, d, incr_e, incr_ne, temp;
	uint8_t yflag = 0, xyflag = 0;

	if (!dev || !dev->init_flag)
		return -1;
	
	/* 横线单独处理 */
	if (y0 == y1) {
		/* 0号点X坐标大于1号点X坐标，则交换两点X坐标 */
		if (x0 > x1) {temp = x0; x0 = x1; x1 = temp;}
		
		/* 遍历X坐标 */
		for (x = x0; x <= x1; x ++)
			oled_draw_point(dev, x, y0);	// 依次画点

	/* 竖线单独处理 */
	} else if (x0 == x1) {
		/* 0号点Y坐标大于1号点Y坐标，则交换两点Y坐标 */
		if (y0 > y1) {temp = y0; y0 = y1; y1 = temp;}
		
		/* 遍历Y坐标 */
		for (y = y0; y <= y1; y ++)
			oled_draw_point(dev, x0, y);	// 依次画点
	
	/* 斜线 */
	} else {
		/* 
		 * 使用Bresenham算法画直线，可以避免耗时的浮点运算，效率更高
		 * 参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf
		 * 参考教程：https://www.bilibili.com/video/BV1364y1d7Lo 
		 */
		
		/* 0号点X坐标大于1号点X坐标 */
		if (x0 > x1) {
			/* 
			 * 交换两点坐标
			 * 交换后不影响画线，但是画线方向由第一、二、三、四象限变为第一、四象限 
			 */
			temp = x0; x0 = x1; x1 = temp;
			temp = y0; y0 = y1; y1 = temp;
		}
		
		/* 0号点Y坐标大于1号点Y坐标 */
		if (y0 > y1) {
			/* 
			 * 将Y坐标取负
			 * 取负后影响画线，但是画线方向由第一、四象限变为第一象限 
			 */
			y0 = -y0;
			y1 = -y1;
			
			/* 置标志位yflag，记住当前变换，在后续实际画线时，再将坐标换回来 */
			yflag = 1;
		}
		
		/* 画线斜率大于1 */
		if (y1 - y0 > x1 - x0) {
			/* 
			 * 将X坐标与Y坐标互换
			 * 互换后影响画线，但是画线方向由第一象限0~90度范围变为第一象限0~45度范围 
			 */
			temp = x0; x0 = y0; y0 = temp;
			temp = x1; x1 = y1; y1 = temp;
			
			/* 置标志位xyflag，记住当前变换，在后续实际画线时，再将坐标换回来 */
			xyflag = 1;
		}
		
		/* 以下为Bresenham算法画直线，算法要求，画线方向必须为第一象限0~45度范围 */
		dx = x1 - x0;
		dy = y1 - y0;
		incr_e = 2 * dy;
		incr_ne = 2 * (dy - dx);
		d = 2 * dy - dx;
		x = x0;
		y = y0;
		
		/* 画起始点，同时判断标志位，将坐标换回来 */
		if (yflag && xyflag)
			oled_draw_point(dev, y, -x);
		else if (yflag)
			oled_draw_point(dev, x, -y);
		else if (xyflag)
			oled_draw_point(dev, y, x);
		else
			oled_draw_point(dev, x, y);
		
		/* 遍历X轴的每个点 */
		while (x < x1) {
			x ++;
			if (d < 0) {
				/* 下一个点在当前点东方 */
				d += incr_e;
			} else {
				/* 下一个点在当前点东北方 */
				y ++;
				d += incr_ne;
			}
			
			/* 画每一个点，同时判断标志位，将坐标换回来 */
			if (yflag && xyflag)
				oled_draw_point(dev, y, -x);
			else if (yflag)
				oled_draw_point(dev, x, -y);
			else if (xyflag)
				oled_draw_point(dev, y, x);
			else
				oled_draw_point(dev, x, y);
		}	
	}
	
	return 0;
}

/**
 * @brief   OLED 画矩形
 * @param[in] dev       oled_dev_t 结构体指针
 * @param[in] x         指定矩形左上角的横坐标，范围：0~127
 * @param[in] y         指定矩形左上角的纵坐标，范围：0~63
 * @param[in] width     指定矩形的宽度，范围：0~128
 * @param[in] height    指定矩形的高度，范围：0~64
 * @param[in] is_filled 指定矩形是否填充
 * @return	0 表示成功，其他值表示失败
 */
static int oled_draw_rectangle(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t is_filled)
{
	uint8_t i, j;

	if (!dev || !dev->init_flag)
		return -1;
	
	if (!is_filled) {
		/* 遍历上下x坐标，画矩形上下两条线 */
		for (i = x; i < x + width; i ++) {
			oled_draw_point(dev, i, y);
			oled_draw_point(dev, i, y + height - 1);
		}
		/* 遍历左右y坐标，画矩形左右两条线 */
		for (i = y; i < y + height; i ++) {
			oled_draw_point(dev, x, i);
			oled_draw_point(dev, x + width - 1, i);
		}
	} else {
		for (i = x; i < x + width; i ++)
			for (j = y; j < y + height; j ++)
				oled_draw_point(dev, i, j);
	}
	
	return 0;
}

/**
 * @brief   OLED 画三角形
 * @param[in] dev       oled_dev_t 结构体指针
 * @param[in] x0        指定第一个端点的横坐标，范围：0~127
 * @param[in] y0        指定第一个端点的纵坐标，范围：0~63
 * @param[in] x1        指定第二个端点的横坐标，范围：0~127
 * @param[in] y1        指定第二个端点的纵坐标，范围：0~63
 * @param[in] x2        指定第三个端点的横坐标，范围：0~127
 * @param[in] y2        指定第三个端点的纵坐标，范围：0~63
 * @param[in] is_filled 指定三角形是否填充
 * @return	0 表示成功，其他值表示失败
 */
static int oled_draw_triangle(oled_dev_t *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t is_filled)
{
	uint8_t minx = x0, miny = y0, maxx = x0, maxy = y0;
	uint8_t i, j;
	int16_t vx[] = {x0, x1, x2};
	int16_t vy[] = {y0, y1, y2};

	if (!dev || !dev->init_flag)
		return -1;
	
	if (!is_filled) {
		/* 调用画线函数，将三个点用直线连接 */
		oled_draw_line(dev, x0, y0, x1, y1);
		oled_draw_line(dev, x0, y0, x2, y2);
		oled_draw_line(dev, x1, y1, x2, y2);
	} else {
		/* 找到三个点最小的X、Y坐标 */
		if (x1 < minx) minx = x1;
		if (x2 < minx) minx = x2;
		if (y1 < miny) miny = y1;
		if (y2 < miny) miny = y2;
		
		/* 找到三个点最大的X、Y坐标 */
		if (x1 > maxx) maxx = x1;
		if (x2 > maxx) maxx = x2;
		if (y1 > maxy) maxy = y1;
		if (y2 > maxy) maxy = y2;
		
		/*
		 * 最小最大坐标之间的矩形为可能需要填充的区域
		 * 遍历此区域中所有的点
		 * 遍历X坐标 
		 */		
		for (i = minx; i <= maxx; i ++) {
			/* 遍历Y坐标 */	
			for (j = miny; j <= maxy; j ++) {
				/*
				 * 调用oled_pnpoly，判断指定点是否在指定三角形之中
				 * 如果在，则画点，如果不在，则不做处理
				 */
				if (oled_pnpoly(3, vx, vy, i, j))
					oled_draw_point(dev, i, j);
			}
		}
	}
	
	return 0;
}

/**
 * @brief   OLED 画圆
 * @param[in] dev       oled_dev_t 结构体指针
 * @param[in] x         指定圆的圆心横坐标，范围：0~127
 * @param[in] y         指定圆的圆心纵坐标，范围：0~63
 * @param[in] radius    指定圆的半径，范围：0~255
 * @param[in] is_filled 指定圆是否填充
 * @return	0 表示成功，其他值表示失败
 */
static int oled_draw_circle(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t is_filled)
{	
	/*
	 * 使用Bresenham算法画圆，可以避免耗时的浮点运算，效率更高
	 * 参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf
	 * 参考教程：https://www.bilibili.com/video/BV1VM4y1u7wJ 
	 */
	int16_t x1, y1, d, j;

	if (!dev || !dev->init_flag)
		return -1;
	
	d = 1 - radius;
	x1 = 0;
	y1 = radius;
	
	/* 画每个八分之一圆弧的起始点 */
	oled_draw_point(dev, x + x1, y + y1);
	oled_draw_point(dev, x - x1, y - y1);
	oled_draw_point(dev, x + y1, y + x1);
	oled_draw_point(dev, x - y1, y - x1);
	
	if (is_filled) {
		/* 遍历起始点Y坐标 */
		for (j = -y1; j < y1; j ++) {
			/* 在指定区域画点，填充部分圆 */
			oled_draw_point(dev, x, y + j);
		}
	}
	
	/* 遍历X轴的每个点 */
	while (x1 < y1) {
		x1 ++;
		if (d < 0) {
			/* 下一个点在当前点东方 */
			d += 2 * x1 + 1;
		} else {
			/* 下一个点在当前点东南方 */
			y1 --;
			d += 2 * (x1 - y1) + 1;
		}
		
		/* 画每个八分之一圆弧的点 */
		oled_draw_point(dev, x + x1, y + y1);
		oled_draw_point(dev, x + y1, y + x1);
		oled_draw_point(dev, x - x1, y - y1);
		oled_draw_point(dev, x - y1, y - x1);
		oled_draw_point(dev, x + x1, y - y1);
		oled_draw_point(dev, x + y1, y - x1);
		oled_draw_point(dev, x - x1, y + y1);
		oled_draw_point(dev, x - y1, y + x1);
		
		if (is_filled) {
			/* 遍历中间部分 */
			for (j = -y1; j < y1; j ++) {
				/* 在指定区域画点，填充部分圆 */
				oled_draw_point(dev, x + x1, y + j);
				oled_draw_point(dev, x - x1, y + j);
			}
			
			/* 遍历两侧部分 */
			for (j = -x1; j < x1; j ++) {
				/* 在指定区域画点，填充部分圆 */
				oled_draw_point(dev, x - y1, y + j);
				oled_draw_point(dev, x + y1, y + j);
			}
		}
	}
	
	return 0;
}

/**
 * @brief   OLED 画椭圆
 * @param[in] dev       oled_dev_t 结构体指针
 * @param[in] x         指定椭圆的圆心横坐标，范围：0~127
 * @param[in] y         指定椭圆的圆心纵坐标，范围：0~63
 * @param[in] a         指定椭圆的横向半轴长度，范围：0~255
 * @param[in] b         指定椭圆的纵向半轴长度，范围：0~255
 * @param[in] is_filled 指定椭圆是否填充
 * @return	0 表示成功，其他值表示失败
 */
static int oled_draw_ellipse(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, uint8_t is_filled)
{	
	/* 
	 * 使用Bresenham算法画椭圆，可以避免部分耗时的浮点运算，效率更高
	 * 参考链接：https://blog.csdn.net/myf_666/article/details/128167392 
	 */
	int16_t x1, y1, j;
	float d1, d2;

	if (!dev || !dev->init_flag)
		return -1;
	
	x1 = 0;
	y1 = b;
	d1 = b * b + a * a * (-b + 0.5);
	
	if (is_filled) {
		/* 遍历起始点Y坐标 */
		for (j = -y1; j < y1; j ++)  {
			/* 在指定区域画点，填充部分椭圆 */
			oled_draw_point(dev, x, y + j);
			oled_draw_point(dev, x, y + j);
		}
	}
	
	/* 画椭圆弧的起始点 */
	oled_draw_point(dev, x + x1, y + y1);
	oled_draw_point(dev, x - x1, y - y1);
	oled_draw_point(dev, x - x1, y + y1);
	oled_draw_point(dev, x + x1, y - y1);
	
	/* 画椭圆中间部分 */
	while (b * b * (x1 + 1) < a * a * (y1 - 0.5)) {
		if (d1 <= 0) {
			/* 下一个点在当前点东方 */
			d1 += b * b * (2 * x1 + 3);
		} else {
			/* 下一个点在当前点东南方 */
			d1 += b * b * (2 * x1 + 3) + a * a * (-2 * y1 + 2);
			y1 --;
		}
		x1 ++;
		
		if (is_filled) {
			/* 遍历中间部分 */
			for (j = -y1; j < y1; j ++) {
				/* 在指定区域画点，填充部分椭圆 */
				oled_draw_point(dev, x + x1, y + j);
				oled_draw_point(dev, x - x1, y + j);
			}
		}
		
		/* 画椭圆中间部分圆弧 */
		oled_draw_point(dev, x + x1, y + y1);
		oled_draw_point(dev, x - x1, y - y1);
		oled_draw_point(dev, x - x1, y + y1);
		oled_draw_point(dev, x + x1, y - y1);
	}                               
	
	/* 画椭圆两侧部分 */
	d2 = b * b * (x1 + 0.5) * (x1 + 0.5) + a * a * (y1 - 1) * (y1 - 1) - a * a * b * b;
	
	while (y1 > 0) {
		if (d2 <= 0) {
			/* 下一个点在当前点东方 */
			d2 += b * b * (2 * x1 + 2) + a * a * (-2 * y1 + 3);
			x1 ++;
			
		} else {
			/* 下一个点在当前点东南方 */
			d2 += a * a * (-2 * y1 + 3);
		}
		y1 --;
		
		if (is_filled) {
			/* 遍历两侧部分 */
			for (j = -y1; j < y1; j ++) {
				/* 在指定区域画点，填充部分椭圆 */
				oled_draw_point(dev, x + x1, y + j);
				oled_draw_point(dev, x - x1, y + j);
			}
		}
		
		/* 画椭圆两侧部分圆弧 */
		oled_draw_point(dev, x + x1, y + y1);
		oled_draw_point(dev, x - x1, y - y1);
		oled_draw_point(dev, x - x1, y + y1);
		oled_draw_point(dev, x + x1, y - y1);
	}
	
	return 0;
}

/**
 * @brief   OLED 画圆弧
 * @param[in] dev         oled_dev_t 结构体指针
 * @param[in] x           指定圆弧的圆心横坐标，范围：0~127
 * @param[in] y           指定圆弧的圆心纵坐标，范围：0~63
 * @param[in] radius      指定圆弧的半径，范围：0~255
 * @param[in] start_angle 指定圆弧的起始角度，范围：-180~180
 * 						  水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
 * @param[in] end_angle   指定圆弧的终止角度，范围：-180~180
 * 						  水平向右为0度，水平向左为180度或-180度，下方为正数，上方为负数，顺时针旋转
 * @param[in] is_filled  指定圆弧是否填充，填充后为扇形
 * @return	0 表示成功，其他值表示失败
 */
static int oled_draw_arc(oled_dev_t *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t start_angle, int16_t end_angle, uint8_t is_filled)
{
	/* 此函数借用Bresenham算法画圆的方法 */
	int16_t x1, y1, d, j;
	
	if (!dev || !dev->init_flag)
		return -1;
	
	d = 1 - radius;
	x1 = 0;
	y1 = radius;
	
	/* 在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
	if (oled_is_in_angle(x1, y1, start_angle, end_angle))
		oled_draw_point(dev, x + x1, y + y1);
	if (oled_is_in_angle(-x1, -y1, start_angle, end_angle))
		oled_draw_point(dev, x - x1, y - y1);
	if (oled_is_in_angle(y1, x1, start_angle, end_angle))
		oled_draw_point(dev, x + y1, y + x1);
	if (oled_is_in_angle(-y1, -x1, start_angle, end_angle))
		oled_draw_point(dev, x - y1, y - x1);
	
	if (is_filled) {
		/* 遍历起始点Y坐标 */
		for (j = -y1; j < y1; j ++) {
			/* 在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
			if (oled_is_in_angle(0, j, start_angle, end_angle)) {oled_draw_point(dev, x, y + j);}
		}
	}
	
	/* 遍历X轴的每个点 */
	while (x1 < y1) {
		x1 ++;
		if (d < 0) {
			/* 下一个点在当前点东方 */
			d += 2 * x1 + 1;
		} else {
			/* 下一个点在当前点东南方 */
			y1 --;
			d += 2 * (x1 - y1) + 1;
		}
		
		/* 在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
		if (oled_is_in_angle(x1, y1, start_angle, end_angle))
			oled_draw_point(dev, x + x1, y + y1);
		if (oled_is_in_angle(y1, x1, start_angle, end_angle))
			oled_draw_point(dev, x + y1, y + x1);
		if (oled_is_in_angle(-x1, -y1, start_angle, end_angle))
			oled_draw_point(dev, x - x1, y - y1);
		if (oled_is_in_angle(-y1, -x1, start_angle, end_angle))
			oled_draw_point(dev, x - y1, y - x1);
		if (oled_is_in_angle(x1, -y1, start_angle, end_angle))
			oled_draw_point(dev, x + x1, y - y1);
		if (oled_is_in_angle(y1, -x1, start_angle, end_angle))
			oled_draw_point(dev, x + y1, y - x1);
		if (oled_is_in_angle(-x1, y1, start_angle, end_angle))
			oled_draw_point(dev, x - x1, y + y1);
		if (oled_is_in_angle(-y1, x1, start_angle, end_angle))
			oled_draw_point(dev, x - y1, y + x1);
		
		if (is_filled) {
			/* 遍历中间部分 */
			for (j = -y1; j < y1; j ++) {
				/* 在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
				if (oled_is_in_angle(x1, j, start_angle, end_angle))
					oled_draw_point(dev, x + x1, y + j);
				if (oled_is_in_angle(-x1, j, start_angle, end_angle))
					oled_draw_point(dev, x - x1, y + j);
			}
			
			/* 遍历两侧部分 */
			for (j = -x1; j < x1; j ++) {
				/* 在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理 */
				if (oled_is_in_angle(-y1, j, start_angle, end_angle))
					oled_draw_point(dev, x - y1, y + j);
				if (oled_is_in_angle(y1, j, start_angle, end_angle))
					oled_draw_point(dev, x + y1, y + j);
			}
		}
	}
	
	return 0;
}

/**
 * @brief   去初始化 OLED 设备
 * @param[in,out] dev oled_dev_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
static int oled_drv_deinit(oled_dev_t *dev)
{
	oled_priv_data_t *priv_data = (oled_priv_data_t *)dev->priv_data;

    if (!dev || !dev->init_flag)
        return -1;
	
	priv_data->i2c.deinit(&priv_data->i2c);
	free(dev->priv_data);
	dev->priv_data = NULL;
	dev->init_flag = false;
    
    return 0;
}

/********************************************************************** 功能函数 */
