#include "lcd.h"
#include "i2c.h"
#include <util/delay.h>
#include <stdio.h>

// LCD control bits
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01

static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = nibble | mode | LCD_BACKLIGHT;
    
    i2c_start();
    i2c_write(LCD_ADDR << 1);  // Address with write bit
    
    // Send with E high
    i2c_write(data | LCD_ENABLE);
    _delay_us(1);
    
    // Send with E low
    i2c_write(data & ~LCD_ENABLE);
    _delay_us(50);
    
    i2c_stop();
}

static void lcd_send_byte(uint8_t data, uint8_t mode) {
    lcd_send_nibble(data & 0xF0, mode);       // High nibble
    lcd_send_nibble((data << 4) & 0xF0, mode); // Low nibble
}

void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, 0);  // RS=0 for command
    _delay_ms(2);
}

void lcd_data(uint8_t data) {
    lcd_send_byte(data, LCD_RS);  // RS=1 for data
    _delay_ms(2);
}

void lcd_init(void) {
    i2c_init();
    _delay_ms(50);  // Wait for LCD to power up
    
    // Initialize in 4-bit mode
    lcd_send_nibble(0x30, 0);
    _delay_ms(5);
    lcd_send_nibble(0x30, 0);
    _delay_us(150);
    lcd_send_nibble(0x30, 0);
    _delay_ms(5);
    lcd_send_nibble(0x20, 0);  // Set 4-bit mode
    _delay_ms(5);
    
    // Function set: 4-bit, 2 lines, 5x8 dots
    lcd_command(0x28);
    
    // Display on, cursor off, blink off
    lcd_command(0x0C);
    
    // Clear display
    lcd_clear();
    
    // Entry mode: increment cursor, no shift
    lcd_command(0x06);
}

void lcd_clear(void) {
    lcd_command(LCD_CLEAR);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_command(0x80 | (col + row_offsets[row]));
}

void lcd_print(const char* str) {
    while(*str) {
        lcd_data(*str++);
    }
}

void lcd_print_num(uint16_t num) {
    char buffer[6];
    snprintf(buffer, 6, "%u", num);
    lcd_print(buffer);
}