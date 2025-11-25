#ifndef DRV_SSD1306_H
#define DRV_SSD1306_H

#include <stdint.h>
#include <stdbool.h>

/* ----------------------- 用户配置，可根据实际硬件修改 ----------------------- */
#ifndef MAX_SSD1306_NUM
#define MAX_SSD1306_NUM	1
#endif
/* -------------------------------------------------------------------------- */

/* 
 * size 参数取值枚举
 * 此参数值不仅用于判断，而且用于计算横向字符偏移，默认值为字体像素宽度 
 */
typedef enum {
	OLED_8X16 = 8,
	OLED_6X8 = 6
} ssd1306_font_size_t;

/* I2C 操作接口结构体 */
typedef struct {
	int (*write_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
	int (*write_regs)(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data);
} ssd1306_i2c_ops_t;

/* 配置结构体 */
typedef struct {
	const ssd1306_i2c_ops_t *i2c_ops;
	bool is_forward;  // true: 正向显示；false: 反向显示
} ssd1306_cfg_t;

typedef struct ssd1306_dev ssd1306_dev_t;

/* 操作接口结构体 */
typedef struct {
	int (*update)(ssd1306_dev_t *dev);
	int (*update_area)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*clear)(ssd1306_dev_t *dev);
	int (*clear_area)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*reverse)(ssd1306_dev_t *dev);
	int (*reverse_area)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
	int (*show_image)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *img);
	int (*show_char)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, char chr, ssd1306_font_size_t size);
	int (*show_str)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, char *str, ssd1306_font_size_t size);
	int (*show_num)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, ssd1306_font_size_t size);
	int (*show_signed_num)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, int32_t num, uint8_t len, ssd1306_font_size_t size);
	int (*show_hex_num)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, ssd1306_font_size_t size);
	int (*show_bin_num)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint32_t num, uint8_t len, ssd1306_font_size_t size);
	int (*show_float_num)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, double num, uint8_t int_len, uint8_t fra_len, ssd1306_font_size_t size);
	int (*show_chinese)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, char *chinese);
	int (*printf)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, ssd1306_font_size_t size, char *format, ...);
	int (*draw_point)(ssd1306_dev_t *dev, uint8_t x, uint8_t y);
	int (*get_point)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t *point);
	int (*draw_line)(ssd1306_dev_t *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
	int (*draw_rectangle)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool is_filled);
	int (*draw_triangle)(ssd1306_dev_t *dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool is_filled);
	int (*draw_circle)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t radius, bool is_filled);
	int (*draw_ellipse)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t a, uint8_t b, bool is_filled);
	int (*draw_arc)(ssd1306_dev_t *dev, uint8_t x, uint8_t y, uint8_t radius, int16_t start_angle, int16_t end_angle, bool is_filled);
	int (*deinit)(ssd1306_dev_t *dev);
} ssd1306_ops_t;

/* 设备结构体 */
struct ssd1306_dev {
	void *priv;
	ssd1306_cfg_t cfg;
	const ssd1306_ops_t *ops;
};

/**
 * @brief   初始化 SSD1306 驱动
 * @param[out] dev ssd1306_dev_t 结构体指针
 * @param[in]  cfg ssd1306_cfg_t 结构体指针
 * @return	0 表示成功，其他值表示失败
 */
int drv_ssd1306_init(ssd1306_dev_t *dev, const ssd1306_cfg_t *cfg);

#endif
