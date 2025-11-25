#ifndef DRV_LCD_DATA_H
#define DRV_LCD_DATA_H

#include <stdint.h>

/* 中文字符字节宽度 */
#define LCD_CHN_CHAR_WIDTH	3		// UTF-8编码格式给3，GB2312编码格式给2

/* 字模基本单元 */
typedef struct 
{
	uint8_t index[LCD_CHN_CHAR_WIDTH + 1];	// 汉字索引
	uint8_t msk[24];						// 字模数据
} chinese_cell_12_t;

typedef struct 
{
	uint8_t index[LCD_CHN_CHAR_WIDTH + 1];	// 汉字索引
	uint8_t msk[32];						// 字模数据
} chinese_cell_16_t;

typedef struct 
{
	uint8_t index[LCD_CHN_CHAR_WIDTH + 1];	// 汉字索引
	uint8_t msk[72];						// 字模数据
} chinese_cell_24_t;

typedef struct 
{
	uint8_t index[LCD_CHN_CHAR_WIDTH + 1];	// 汉字索引
	uint8_t msk[128];						// 字模数据
} chinese_cell_32_t;

/* ASCII字模数据声明 */
extern const uint8_t lcd_f6x12[][12];
extern const uint8_t lcd_f8x6[][16];
extern const uint8_t lcd_f12x24[][48];
extern const uint8_t lcd_f16x32[][64];

/* 汉字字模数据声明 */
extern const chinese_cell_12_t lcd_f12x12[];
extern const chinese_cell_16_t lcd_f16x16[];
extern const chinese_cell_24_t lcd_f24x24[];
extern const chinese_cell_32_t lcd_f32x32[];

/* 获取字数 */
int get_chinese_num(uint8_t size);

/* 图像数据声明 */
extern const uint8_t image_test[];

#endif
