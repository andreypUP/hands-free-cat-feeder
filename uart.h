// uart.h
#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdint.h>

// Initialize UART with specified baud rate
void uart_init(uint32_t baud);

// Transmit a single character
void uart_transmit(char data);

// Transmit a string
void uart_print(const char* str);

// Transmit a number
void uart_print_num(uint16_t num);

// Transmit newline
void uart_println(const char* str);

#endif