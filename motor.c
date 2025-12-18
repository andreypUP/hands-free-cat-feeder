#include "motor.h"         // Motor function declarations
#include "gpio.h"          // GPIO abstraction (read/write pins)
#include <avr/io.h>        // AVR register definitions
#include <avr/interrupt.h> // Interrupt handling (ISR)

// Pin definitions (connected to stepper motor driver inputs)
// PB0–PB3 are used to control the four motor coils
#define IN1 0 // PB0
#define IN2 1 // PB1
#define IN3 2 // PB2
#define IN4 3 // PB3

// Full-step sequence for stepper motor rotation
// Each row represents one step
// Each column corresponds to one motor coil (IN1–IN4)
// 1 = coil energized, 0 = coil off
static const uint8_t step_sequence[4][4] = {
    {1, 0, 0, 1}, // Step 1
    {1, 1, 0, 0}, // Step 2
    {0, 1, 1, 0}, // Step 3
    {0, 0, 1, 1}  // Step 4
};

// Shared variables used inside interrupt service routine (ISR)
// volatile ensures correct behavior since ISR modifies them
volatile uint8_t current_step = 0;       // Tracks current step index (0–3)
volatile uint16_t remaining_time_ms = 0; // Remaining motor run time in milliseconds
volatile uint8_t motor_running = 0;      // Motor state flag (1 = running)

// External callback function implemented in main.c
// Called when motor dispensing is finished
extern void motor_dispense_complete_callback(void);

// Timer2 Compare Match A Interrupt
// This ISR is triggered every ~2ms
// Responsible for stepping the motor and timing the operation
ISR(TIMER2_COMPA_vect)
{

    // Only run motor if enabled and time remains
    if (motor_running && remaining_time_ms > 0)
    {

        // Apply the current step pattern to the motor coils
        // Each gpio_write sets one coil HIGH or LOW
        gpio_write(GPIO_PORT_B, IN1, step_sequence[current_step][0] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN2, step_sequence[current_step][1] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN3, step_sequence[current_step][2] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN4, step_sequence[current_step][3] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);

        // Advance to the next step in the sequence
        // Modulo 4 ensures the sequence repeats continuously
        current_step = (current_step + 1) % 4;

        // Reduce remaining time
        // Since ISR runs every 2ms, subtract 2ms each interrupt
        if (remaining_time_ms >= 2)
        {
            remaining_time_ms -= 2;
        }
        else
        {
            remaining_time_ms = 0;
        }

        // If the allotted time has expired, stop the motor
        if (remaining_time_ms == 0)
        {
            motor_stop();                       // Disable motor and coils
            motor_dispense_complete_callback(); // Notify main application
        }
    }
}

// Initializes motor control and timer interrupt
void motor_init(void)
{

    // Configure motor control pins as outputs
    gpio_set_direction(GPIO_PORT_B, IN1, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN2, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN3, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN4, GPIO_PIN_OUTPUT);

    // Ensure motor is off during initialization
    motor_stop();

    // Configure Timer2 for periodic interrupts
    // WGM21 = 1 → CTC (Clear Timer on Compare Match) mode
    //     What is CTC mode (simple terms)
    // The timer counts up,
    // stops automatically when it reaches OCR2A,
    // resets to zero,
    // and fires an interrupt.
    // Output Compare Register 2 A
    // OCR2A stores the compare value that the timer counts up to.

    // Timer/Counter Control Register 2 A
    TCCR2A = (1 << WGM21); // Waveform Generation Mode // Timer2

    // How Timer2 uses OCR2A (step-by-step)
    // Timer2 starts counting from 0
    // Each tick comes from the prescaled system clock
    // When the counter reaches OCR2A
    // Timer resets to 0
    // Interrupt fires
    // Motor takes one step

    // Prescaler = 1024 (CS22 | CS21 | CS20)
    // Slows down timer clock for manageable interrupt rate
    // Timer/Counter Control Register 2 B
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);

    // Compare value
    // At 16 MHz with prescaler 1024, OCR2A = 31 gives ~2ms interval
    OCR2A = 31;

    // Enable Timer2 Compare Match A interrupt
    // Whenever Timer2 reaches the value in OCR2A, fire the ISR TIMER2_COMPA_vect
    // Timer/Counter2 Interrupt Mask Register
    TIMSK2 = (1 << OCIE2A); // Output Compare A Match Interrupt Enable
}

// Starts motor operation for a specified duration (in milliseconds)
void motor_start_dispensing(uint16_t duration_ms)
{
    current_step = 0;                // Reset step sequence
    remaining_time_ms = duration_ms; // Set how long motor should run
    motor_running = 1;               // Enable motor stepping
}

// Enables motor stepping without setting a time
// Used when external logic controls stopping
void motor_step_forward(void)
{
    if (!motor_running)
    {
        motor_running = 1;
    }
}

// Stops motor operation immediately
void motor_stop(void)
{
    motor_running = 0;     // Disable motor stepping
    remaining_time_ms = 0; // Clear remaining time

    // Turn off all motor coils to save power and prevent overheating
    gpio_write(GPIO_PORT_B, IN1, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN2, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN3, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN4, GPIO_PIN_LOW);
}

// Returns current motor state
uint8_t motor_is_running(void)
{
    return motor_running;
}

// Returns remaining motor run time in milliseconds
uint16_t motor_get_remaining_time(void)
{
    return remaining_time_ms;
}
