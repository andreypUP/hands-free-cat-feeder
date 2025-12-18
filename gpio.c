#include "gpio.h"

void gpio_set_direction(GpioPort_t port, uint8_t pin_num, GpioDirection_t direction) {
    switch(port) {
        case GPIO_PORT_B:
            if(direction == GPIO_PIN_OUTPUT) DDRB |= (1 << pin_num);
            else DDRB &= ~(1 << pin_num);
            break;
        case GPIO_PORT_C:
            if(direction == GPIO_PIN_OUTPUT) DDRC |= (1 << pin_num);
            else DDRC &= ~(1 << pin_num);
            break;
        case GPIO_PORT_D:
            if(direction == GPIO_PIN_OUTPUT) DDRD |= (1 << pin_num);
            else DDRD &= ~(1 << pin_num);
            break;
    }
}

void gpio_set_pullup(GpioPort_t port, uint8_t pin_num, uint8_t enable) {
    switch(port) {
        case GPIO_PORT_B:
            if(enable) PORTB |= (1 << pin_num);
            else PORTB &= ~(1 << pin_num);
            break;
        case GPIO_PORT_C:
            if(enable) PORTC |= (1 << pin_num);
            else PORTC &= ~(1 << pin_num);
            break;
        case GPIO_PORT_D:
            if(enable) PORTD |= (1 << pin_num);
            else PORTD &= ~(1 << pin_num);
            break;
    }
}

void gpio_write(GpioPort_t port, uint8_t pin_num, GpioValue_t value) {
    switch(port) {
        case GPIO_PORT_B:
            if(value == GPIO_PIN_HIGH) PORTB |= (1 << pin_num);
            else PORTB &= ~(1 << pin_num);
            break;
        case GPIO_PORT_C:
            if(value == GPIO_PIN_HIGH) PORTC |= (1 << pin_num);
            else PORTC &= ~(1 << pin_num);
            break;
        case GPIO_PORT_D:
            if(value == GPIO_PIN_HIGH) PORTD |= (1 << pin_num);
            else PORTD &= ~(1 << pin_num);
            break;
    }
}

void gpio_toggle(GpioPort_t port, uint8_t pin_num) {
    switch(port) {
        case GPIO_PORT_B: PORTB ^= (1 << pin_num); break;
        case GPIO_PORT_C: PORTC ^= (1 << pin_num); break;
        case GPIO_PORT_D: PORTD ^= (1 << pin_num); break;
    }
}

GpioValue_t gpio_read(GpioPort_t port, uint8_t pin_num) {
    switch(port) {
        case GPIO_PORT_B: return (PINB & (1 << pin_num)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
        case GPIO_PORT_C: return (PINC & (1 << pin_num)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
        case GPIO_PORT_D: return (PIND & (1 << pin_num)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
    }
    return GPIO_PIN_LOW;
}
