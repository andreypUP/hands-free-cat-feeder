#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "gpio.h"
#include "sensor.h"

#define SENSOR_TRIG_PORT GPIO_PORT_D
#define SENSOR_TRIG_PIN  7
#define SENSOR_ECHO_PORT GPIO_PORT_D
#define SENSOR_ECHO_PIN  6

volatile uint16_t pulse_width = 0;
volatile uint8_t measurement_complete = 0;

void sensor_init(void) {
    gpio_set_direction(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_OUTPUT);
    gpio_set_direction(SENSOR_ECHO_PORT, SENSOR_ECHO_PIN, GPIO_PIN_INPUT);
    gpio_write(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_LOW);
    
    // Setup Timer1 for microsecond counting
    TCCR1A = 0;
    TCCR1B = (1 << CS11); // Prescaler 8: each tick = 0.5us at 16MHz
    TCNT1 = 0;
}

void sensor_trigger(void) {
    measurement_complete = 0;
    pulse_width = 0;
    
    // Send 10us trigger pulse
    gpio_write(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_HIGH);
    _delay_us(10);
    gpio_write(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_LOW);
    
    // Wait for echo to go HIGH (with timeout)
    uint16_t timeout = 0;
    while(gpio_read(SENSOR_ECHO_PORT, SENSOR_ECHO_PIN) == GPIO_PIN_LOW) {
        _delay_us(2);
        timeout++;
        if(timeout > 5000) { // ~10ms timeout
            return; // No echo pulse started
        }
    }
    
    // Echo is HIGH, start timing
    TCNT1 = 0; // Reset timer
    uint16_t start_time = TCNT1;
    
    // Wait for echo to go LOW (with timeout)
    timeout = 0;
    while(gpio_read(SENSOR_ECHO_PORT, SENSOR_ECHO_PIN) == GPIO_PIN_HIGH) {
        timeout++;
        if(timeout > 30000) { // ~60ms timeout (max range)
            return; // Timeout
        }
        _delay_us(2);
    }
    
    // Echo is LOW, calculate pulse width
    uint16_t end_time = TCNT1;
    
    if(end_time >= start_time) {
        pulse_width = end_time - start_time;
    } else {
        pulse_width = (0xFFFF - start_time) + end_time;
    }
    
    measurement_complete = 1;
}

uint8_t sensor_is_measurement_complete(void) {
    return measurement_complete;
}

uint16_t sensor_get_pulse_width(void) {
    return pulse_width;
}