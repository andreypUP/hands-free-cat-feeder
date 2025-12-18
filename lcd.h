#ifndef LCD_H
#define LCD_H

#include <stdint.h>

// Common I2C LCD addresses (try 0x27 first, then 0x3F)
#define LCD_ADDR 0x27

// LCD commands
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02

void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char* str);
void lcd_print_num(uint16_t num);

#endif