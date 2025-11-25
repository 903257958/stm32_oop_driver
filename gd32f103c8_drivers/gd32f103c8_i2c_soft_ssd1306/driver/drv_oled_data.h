#ifndef DRV_OLED_DATA_H
#define DRV_OLED_DATA_H

#include <stdint.h>

/* 中文字符字节宽度 */
#define OLED_CHN_CHAR_WIDTH	3	// UTF-8编码格式给3，GB2312编码格式给2

/* 字模基本单元 */
typedef struct {
	char index[OLED_CHN_CHAR_WIDTH + 1];	// 汉字索引
	uint8_t data[32];						// 字模数据
} chinese_cell_t;

/* ASCII字模数据声明 */
extern const uint8_t oled_f8x16[][16];
extern const uint8_t oled_f6x8[][6];

/* 汉字字模数据声明 */
extern const chinese_cell_t oled_cf16x16[];

/* 图像数据声明 */
extern const uint8_t diode[];

#endif
