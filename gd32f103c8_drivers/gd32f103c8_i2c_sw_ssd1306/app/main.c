#include "bsp.h"

int main(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    bsp_init();

    while (1) {
        /* ================== 文本显示 ================== */
        oled.clear(&oled);
        oled.show_string(&oled, 0, 0, "Hello, world!", OLED_6X8);
        oled.show_string(&oled, 0, 10, "OLED Test", OLED_8X16);
        oled.show_char(&oled, 0, 30, 'A', OLED_6X8);
        oled.show_char(&oled, 10, 30, 'B', OLED_6X8);
		oled.show_chinese(&oled, 80, 0, "你");
        oled.show_chinese(&oled, 96, 0, "好");
        oled.update(&oled);
        delay_ms(1000);

        /* ================== 数字 ================== */
        oled.clear(&oled);
        oled.show_num(&oled, 0, 0, 12345, 5, OLED_8X16);
        oled.show_signed_num(&oled, 0, 16, -678, 4, OLED_6X8);
        oled.show_hex_num(&oled, 0, 28, 0xABCD, 4, OLED_6X8);
        oled.show_bin_num(&oled, 00, 40, 0x5A, 8, OLED_6X8);
        oled.show_float_num(&oled, 0, 52, 3.14159, 1, 4, OLED_6X8);
        oled.update(&oled);
        delay_ms(1000);

        /* ================== 基本图形 ================== */
        oled.clear(&oled);
        oled.draw_point(&oled, 0, 0);
        oled.draw_line(&oled, 0, 0, 127, 63);
        oled.draw_rectangle(&oled, 10, 20, 50, 30, 0);   // 空心矩形
        oled.draw_rectangle(&oled, 70, 20, 40, 20, 1);   // 实心矩形
        oled.draw_triangle(&oled, 0, 50, 20, 50, 10, 63, 0);
        oled.draw_triangle(&oled, 30, 50, 50, 50, 40, 63, 1);
        oled.draw_circle(&oled, 100, 40, 10, 0);          // 空心圆
        oled.draw_circle(&oled, 115, 50, 8, 1);           // 实心圆
        oled.update(&oled);
        delay_ms(1000);

        /* ================== 清屏与反转 ================== */
        oled.clear(&oled);
        oled.show_string(&oled, 0, 0, "Clear & Reverse", OLED_6X8);
        oled.update(&oled);
        delay_ms(500);

        oled.clear_area(&oled, 0, 0, 64, 32);
        oled.update(&oled);
        delay_ms(500);

        oled.reverse(&oled);
        oled.update(&oled);
        delay_ms(500);

        oled.reverse_area(&oled, 0, 0, 64, 32);
        oled.update(&oled);
        delay_ms(500);

        /* ================== 显示图片和 printf ================== */
        oled.clear(&oled);
        oled.show_image(&oled, 0, 0, 16, 16, diode);
        oled.printf(&oled, 0, 40, OLED_6X8, "Printf: %d %.2f", 123, 3.14);
        oled.update(&oled);
        delay_ms(1000);
    }
}
