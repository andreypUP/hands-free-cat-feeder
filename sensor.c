#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "gpio.h"
#include "sensor.h"

#define SENSOR_TRIG_PORT GPIO_PORT_D
#define SENSOR_TRIG_PIN  7
#define SENSOR_ECHO_PORT GPIO_PORT_D
#define SENSOR_ECHO_PIN  6

// State machine for sensor measurement
typedef enum {
    SENSOR_IDLE,
    SENSOR_WAITING_RISING,
    SENSOR_MEASURING,
    SENSOR_COMPLETE
} SensorState;

volatile SensorState sensor_state = SENSOR_IDLE;
volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_end = 0;
volatile uint16_t pulse_width = 0;
volatile uint8_t measurement_complete = 0;

// Pin Change Interrupt for Echo pin (PD6 - PCINT22)
ISR(PCINT2_vect) {
    uint8_t echo_state = gpio_read(SENSOR_ECHO_PORT, SENSOR_ECHO_PIN);
    
    if (sensor_state == SENSOR_WAITING_RISING && echo_state == GPIO_PIN_HIGH) {
        // Rising edge detected - start measurement
        pulse_start = TCNT1;
        sensor_state = SENSOR_MEASURING;
    }
    else if (sensor_state == SENSOR_MEASURING && echo_state == GPIO_PIN_LOW) {
        // Falling edge detected - measurement complete
        pulse_end = TCNT1;
        
        // Calculate pulse width (handle overflow)
        if (pulse_end >= pulse_start) {
            pulse_width = pulse_end - pulse_start;
        } else {
            pulse_width = (0xFFFF - pulse_start) + pulse_end;
        }
        
        sensor_state = SENSOR_COMPLETE;
        measurement_complete = 1;
        
        // Disable pin change interrupt until next trigger
        PCICR &= ~(1 << PCIE2);
    }
}

// Timer1 overflow - timeout protection
ISR(TIMER1_OVF_vect) {
    if (sensor_state == SENSOR_WAITING_RISING || sensor_state == SENSOR_MEASURING) {
        // Measurement timeout
        sensor_state = SENSOR_IDLE;
        pulse_width = 0;
        measurement_complete = 1;
        PCICR &= ~(1 << PCIE2);
    }
}

void sensor_init(void) {
    // Configure pins
    gpio_set_direction(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_OUTPUT);
    gpio_set_direction(SENSOR_ECHO_PORT, SENSOR_ECHO_PIN, GPIO_PIN_INPUT);
    gpio_write(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_LOW);
    
    // Setup Timer1 for microsecond counting
    TCCR1A = 0;
    TCCR1B = (1 << CS11); // Prescaler 8: 0.5us per tick at 16MHz
    TCNT1 = 0;
    TIMSK1 = (1 << TOIE1); // Enable overflow interrupt for timeout
    
    // Setup Pin Change Interrupt for Echo pin (PD6 = PCINT22)
    PCMSK2 = (1 << PCINT22); // Enable PCINT22
    // PCICR will be enabled when we trigger
}

void sensor_trigger(void) {
    if (sensor_state != SENSOR_IDLE) {
        return; // Measurement already in progress
    }
    
    measurement_complete = 0;
    pulse_width = 0;
    sensor_state = SENSOR_WAITING_RISING;
    
    // Reset and enable timer
    TCNT1 = 0;
    
    // Enable pin change interrupt
    PCIFR = (1 << PCIF2); // Clear any pending interrupt
    PCICR |= (1 << PCIE2); // Enable PCINT[23:16]
    
    // Send trigger pulse (10us)
    gpio_write(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_HIGH);
    _delay_us(10);
    gpio_write(SENSOR_TRIG_PORT, SENSOR_TRIG_PIN, GPIO_PIN_LOW);
}

uint8_t sensor_is_measurement_complete(void) {
    return measurement_complete;
}

uint16_t sensor_get_pulse_width(void) {
    return pulse_width;
}

uint16_t sensor_get_distance_cm(void) {
    // Each timer tick is 0.5us
    // Distance (cm) = (pulse_width * 0.5us * 34300cm/s) / 2
    // Distance (cm) = pulse_width / 116
    if (pulse_width == 0) return 0;
    return pulse_width / 116;
}

void sensor_reset(void) {
    // Reset sensor state to IDLE so it can be triggered again
    sensor_state = SENSOR_IDLE;
    measurement_complete = 0;
}