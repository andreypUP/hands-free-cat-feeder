#include "i2c.h"
#include <avr/io.h>
#include <util/twi.h>

void i2c_init(void) {
    // Set SCL frequency: F_SCL = F_CPU / (16 + 2*TWBR*Prescaler)
    // For 100kHz: TWBR = ((F_CPU / I2C_FREQ) - 16) / 2
    TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;
    TWSR = 0;  // Prescaler = 1
    TWCR = (1 << TWEN);  // Enable I2C
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));  // Wait for start condition
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));  // Wait for transmission
}

uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while(!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t i2c_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(!(TWCR & (1 << TWINT)));
    return TWDR;
}