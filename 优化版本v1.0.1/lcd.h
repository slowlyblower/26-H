/*
 * lcd.h - ST7789 1.9寸 170x320 SPI LCD
 *
 * 引脚(天猛星 FPC):
 *   PB9→SCL  PB8→SDA  PB10→RES  PB11→DC  PB14→CS  PB26→BLK
 */
#ifndef LCD_H
#define LCD_H
#include <stdint.h>

void lcd_init(void);
void lcd_clear(uint16_t color);

/* 小字体 8x16: 30字/行 × 15行 */
void lcd_set_cursor(uint16_t row, uint16_t col);
void lcd_putc(char c);
void lcd_puts(const char *s);
void lcd_print_int(int32_t n);

/* 大字体 16x32 (2x缩放): 15字/行 × 7行 */
void lcd_set_cursor_big(uint16_t row, uint16_t col);
void lcd_putc_big(char c);
void lcd_puts_big(const char *s);
void lcd_print_int_big(int32_t n);

void lcd_set_color(uint16_t fg, uint16_t bg);

#define LCD_BLACK   0x0000
#define LCD_WHITE   0xFFFF
#define LCD_RED     0xF800
#define LCD_GREEN   0x07E0
#define LCD_BLUE    0x001F
#define LCD_YELLOW  0xFFE0

#endif
