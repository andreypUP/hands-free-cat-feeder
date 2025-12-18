#include "motor.h"
#include "gpio.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Pin definitions
#define IN1 0
#define IN2 1
#define IN3 2
#define IN4 3

// Full step sequence
static const uint8_t step_sequence[4][4] = {
    {1, 0, 0, 1},
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1}
};

volatile uint8_t current_step = 0;
volatile uint16_t remaining_time_ms = 0;
volatile uint8_t motor_running = 0;

// Timer2 Compare Match A - triggers every 2ms for motor stepping
ISR(TIMER2_COMPA_vect) {
    if (motor_running && remaining_time_ms > 0) {
        // Apply current step pattern
        gpio_write(GPIO_PORT_B, IN1, step_sequence[current_step][0] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN2, step_sequence[current_step][1] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN3, step_sequence[current_step][2] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN4, step_sequence[current_step][3] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        
        // Move to next step
        current_step = (current_step + 1) % 4;
        
        // Decrement remaining time (ISR fires every 2ms)
        if (remaining_time_ms >= 2) {
            remaining_time_ms -= 2;
        } else {
            remaining_time_ms = 0;
        }
        
        // Stop motor when time expires
        if (remaining_time_ms == 0) {
            motor_stop();
        }
    }
}

void motor_init(void) {
    // Configure motor pins as outputs
    gpio_set_direction(GPIO_PORT_B, IN1, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN2, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN3, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN4, GPIO_PIN_OUTPUT);
    
    // Stop motor initially
    motor_stop();
    
    // Setup Timer2 for motor stepping
    // CTC mode, prescaler 1024
    // OCR2A = 31 gives ~2ms per interrupt at 16MHz
    // (16MHz / 1024) / 31 ≈ 500Hz ≈ 2ms period
    TCCR2A = (1 << WGM21); // CTC mode
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
    OCR2A = 31; // Compare value for ~2ms
    TIMSK2 = (1 << OCIE2A); // Enable compare match interrupt
}

void motor_start_dispensing(uint16_t duration_ms) {
    current_step = 0;
    remaining_time_ms = duration_ms;
    motor_running = 1;
    // Timer2 interrupt will handle the stepping
}

void motor_step_forward(void) {
    // This function keeps the motor running
    // The actual stepping is handled by the Timer2 ISR
    // Just ensure motor_running flag is set
    if (!motor_running) {
        motor_running = 1;
    }
}

void motor_stop(void) {
    motor_running = 0;
    remaining_time_ms = 0;
    
    // Turn off all motor coils
    gpio_write(GPIO_PORT_B, IN1, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN2, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN3, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN4, GPIO_PIN_LOW);
}

uint8_t motor_is_running(void) {
    return motor_running;
}

uint16_t motor_get_remaining_time(void) {
    return remaining_time_ms;
}