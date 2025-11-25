#include "drv_delay.h"
#include "drv_i2c_soft.h"
#include "drv_ssd1306.h"
#include "drv_oled_data.h"

static i2c_soft_dev_t i2c_soft;
static const i2c_soft_cfg_t i2c_soft_cfg = {
    .scl_port     = GPIOA, 
    .scl_pin      = GPIO_PIN_7, 
    .sda_port     = GPIOA, 
    .sda_pin      = GPIO_PIN_8,
    .delay_us     = delay_us, 
    .bit_delay_us = 1
};

int i2c_soft_write_reg_adapter(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return i2c_soft.ops->write_reg(&i2c_soft, dev_addr, reg_addr, data);
}

int i2c_soft_write_regs_adapter(uint8_t dev_addr, uint8_t reg_addr, uint16_t num, uint8_t *data)
{
    return i2c_soft.ops->write_regs(&i2c_soft, dev_addr, reg_addr, num, data);
}

static ssd1306_i2c_ops_t oled_i2c_ops = {
    .write_reg  = i2c_soft_write_reg_adapter, 
    .write_regs = i2c_soft_write_regs_adapter
};

static ssd1306_dev_t oled;
static const ssd1306_cfg_t oled_cfg = {
    .i2c_ops    = &oled_i2c_ops, 
    .is_forward = true
};

int main(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    drv_i2c_soft_init(&i2c_soft, &i2c_soft_cfg);
    drv_ssd1306_init(&oled, &oled_cfg);

    while (1) {
        /* ================== 文本显示 ================== */
        oled.ops->clear(&oled);
        oled.ops->show_str(&oled, 0, 0, "Hello, world!", OLED_6X8);
        oled.ops->show_str(&oled, 0, 10, "OLED Test", OLED_8X16);
        oled.ops->show_char(&oled, 0, 30, 'A', OLED_6X8);
        oled.ops->show_char(&oled, 10, 30, 'B', OLED_6X8);
		oled.ops->show_chinese(&oled, 80, 0, "你");
        oled.ops->show_chinese(&oled, 96, 0, "好");
        oled.ops->update(&oled);
        delay_ms(1000);

        /* ================== 数字 ================== */
        oled.ops->clear(&oled);
        oled.ops->show_num(&oled, 0, 0, 12345, 5, OLED_8X16);
        oled.ops->show_signed_num(&oled, 0, 16, -678, 4, OLED_6X8);
        oled.ops->show_hex_num(&oled, 0, 28, 0xABCD, 4, OLED_6X8);
        oled.ops->show_bin_num(&oled, 00, 40, 0x5A, 8, OLED_6X8);
        oled.ops->show_float_num(&oled, 0, 52, 3.14159, 1, 4, OLED_6X8);
        oled.ops->update(&oled);
        delay_ms(1000);

        /* ================== 基本图形 ================== */
        oled.ops->clear(&oled);
        oled.ops->draw_point(&oled, 0, 0);
        oled.ops->draw_line(&oled, 0, 0, 127, 63);
        oled.ops->draw_rectangle(&oled, 10, 20, 50, 30, 0);   // 空心矩形
        oled.ops->draw_rectangle(&oled, 70, 20, 40, 20, 1);   // 实心矩形
        oled.ops->draw_triangle(&oled, 0, 50, 20, 50, 10, 63, 0);
        oled.ops->draw_triangle(&oled, 30, 50, 50, 50, 40, 63, 1);
        oled.ops->draw_circle(&oled, 100, 40, 10, 0);          // 空心圆
        oled.ops->draw_circle(&oled, 115, 50, 8, 1);           // 实心圆
        oled.ops->update(&oled);
        delay_ms(1000);

        /* ================== 清屏与反转 ================== */
        oled.ops->clear(&oled);
        oled.ops->show_str(&oled, 0, 0, "Clear & Reverse", OLED_6X8);
        oled.ops->update(&oled);
        delay_ms(500);

        oled.ops->clear_area(&oled, 0, 0, 64, 32);
        oled.ops->update(&oled);
        delay_ms(500);

        oled.ops->reverse(&oled);
        oled.ops->update(&oled);
        delay_ms(500);

        oled.ops->reverse_area(&oled, 0, 0, 64, 32);
        oled.ops->update(&oled);
        delay_ms(500);

        /* ================== 显示图片和 printf ================== */
        oled.ops->clear(&oled);
        oled.ops->show_image(&oled, 0, 0, 16, 16, diode);
        oled.ops->printf(&oled, 0, 40, OLED_6X8, "Printf: %d %.2f", 123, 3.14);
        oled.ops->update(&oled);
        delay_ms(1000);
    }
}
