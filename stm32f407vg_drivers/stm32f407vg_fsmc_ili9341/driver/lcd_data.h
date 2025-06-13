#ifndef __LCD_DATA_H
#define __LCD_DATA_H

#include <stdint.h>

/*中文字符字节宽度*/
#define LCD_CHN_CHAR_WIDTH	3		//UTF-8编码格式给3，GB2312编码格式给2

/*字模基本单元*/
typedef struct 
{
	uint8_t Index[LCD_CHN_CHAR_WIDTH + 1];	//汉字索引
	uint8_t Msk[24];						//字模数据
}ChineseCell12_t;

typedef struct 
{
	uint8_t Index[LCD_CHN_CHAR_WIDTH + 1];	//汉字索引
	uint8_t Msk[32];						//字模数据
}ChineseCell16_t;

typedef struct 
{
	uint8_t Index[LCD_CHN_CHAR_WIDTH + 1];	//汉字索引
	uint8_t Msk[72];						//字模数据
}ChineseCell24_t;

typedef struct 
{
	uint8_t Index[LCD_CHN_CHAR_WIDTH + 1];	//汉字索引
	uint8_t Msk[128];						//字模数据
}ChineseCell32_t;

/*ASCII字模数据声明*/
extern const uint8_t LCD_F6x12[][12];
extern const uint8_t LCD_F8x16[][16];
extern const uint8_t LCD_F12x24[][48];
extern const uint8_t LCD_F16x32[][64];

/*汉字字模数据声明*/
extern const ChineseCell12_t LCD_CF12x12[];
extern const ChineseCell16_t LCD_CF16x16[];
extern const ChineseCell24_t LCD_CF24x24[];
extern const ChineseCell32_t LCD_CF32x32[];

/*获取字数*/
int get_Chinese_num(uint8_t font_size);

/*图像数据声明*/
extern const uint8_t image_test[];

#endif
