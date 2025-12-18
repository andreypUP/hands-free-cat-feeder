#include "uart.h"
#include <avr/io.h>

void uart_init(uint32_t baud){
    uint16_t ubrr = (F_CPU/16/baud)-1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1<<TXEN0);  // enable TX only
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8N1
}

void uart_transmit(char data){
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str){
    while(*str) uart_transmit(*str++);
}

void uart_print_num(uint16_t num){
    char buf[6];
    snprintf(buf,6,"%u",num);
    uart_print(buf);
}

void uart_println(const char* str){
    uart_print(str);
    uart_transmit('\r');
    uart_transmit('\n');
}
