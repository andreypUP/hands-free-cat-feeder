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

// External callback function (defined in main.c)
extern void sensor_measurement_complete_callback(uint16_t distance_cm);

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
        
        // Calculate distance: pulse_width / 116 (each tick is 0.5us)
        uint16_t distance_cm = pulse_width / 116;
        
        sensor_state = SENSOR_IDLE;  // Ready for next measurement
        measurement_complete = 1;
        
        // Disable pin change interrupt until next trigger
        PCICR &= ~(1 << PCIE2);
        
        // Notify main.c via callback
        sensor_measurement_complete_callback(distance_cm);
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
        
        // Notify with 0 distance (timeout/error)
        sensor_measurement_complete_callback(0);
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
    PCMSK2 = (1 << PCINT22);
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
    PCIFR = (1 << PCIF2);
    PCICR |= (1 << PCIE2);
    
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
    if (pulse_width == 0) return 0;
    return pulse_width / 116;
}

void sensor_reset(void) {
    sensor_state = SENSOR_IDLE;
    measurement_complete = 0;
}