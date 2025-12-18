#include "motor.h"
#include "gpio.h"
#include <util/delay.h>

// Pin definitions
#define IN1 0
#define IN2 1
#define IN3 2
#define IN4 3

// Choose mode
#define USE_FULL_STEP

#ifdef USE_FULL_STEP
static const uint8_t step_sequence[4][4] = {
    {1, 0, 0, 1},
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1}};
#define NUM_STEPS 4
#endif

// Variable delay helper
static void delay_ms_variable(uint8_t ms)
{
    while (ms--)
    {
        _delay_ms(1);
    }
}

void motor_init(void)
{
    gpio_set_direction(GPIO_PORT_B, IN1, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN2, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN3, GPIO_PIN_OUTPUT);
    gpio_set_direction(GPIO_PORT_B, IN4, GPIO_PIN_OUTPUT);
    motor_stop();
}

void motor_step_forward(void)
{
    for (uint8_t step = 0; step < NUM_STEPS; step++)
    {
        gpio_write(GPIO_PORT_B, IN1, step_sequence[step][0] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN2, step_sequence[step][1] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN3, step_sequence[step][2] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN4, step_sequence[step][3] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        _delay_ms(2);
    }
}

void motor_step_backward(void)
{
    for (int8_t step = NUM_STEPS - 1; step >= 0; step--)
    {
        gpio_write(GPIO_PORT_B, IN1, step_sequence[step][0] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN2, step_sequence[step][1] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN3, step_sequence[step][2] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        gpio_write(GPIO_PORT_B, IN4, step_sequence[step][3] ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
        _delay_ms(2);
    }
}

void motor_forward(uint16_t steps)
{
    for (uint16_t i = 0; i < steps; i++)
    {
        motor_step_forward();
    }
}

void motor_backward(uint16_t steps)
{
    for (uint16_t i = 0; i < steps; i++)
    {
        motor_step_backward();
    }
}

void motor_stop(void)
{
    gpio_write(GPIO_PORT_B, IN1, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN2, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN3, GPIO_PIN_LOW);
    gpio_write(GPIO_PORT_B, IN4, GPIO_PIN_LOW);
}

void motor_rotate_degrees(int16_t degrees)
{
    int16_t steps = (degrees * 512) / 360;

    if (steps > 0)
    {
        motor_forward(steps);
    }
    else
    {
        motor_backward(-steps);
    }
}

void motor_forward_smooth(uint16_t steps)
{
    uint16_t accel_steps = 50;
    uint16_t decel_steps = 50;

    for (uint16_t i = 0; i < steps; i++)
    {
        if (i < accel_steps)
        {
            uint8_t delay_val = 10 - (i * 8 / accel_steps);
            motor_step_forward();
            delay_ms_variable(delay_val);
        }
        else if (i > steps - decel_steps)
        {
            uint8_t delay_val = 2 + ((i - (steps - decel_steps)) * 8 / decel_steps);
            motor_step_forward();
            delay_ms_variable(delay_val);
        }
        else
        {
            motor_step_forward();
        }
    }
}
// Run motor for specified duration in milliseconds
void motor_run_for_duration(uint16_t duration_ms) {
    uint16_t elapsed = 0;
    
    while(elapsed < duration_ms) {
        motor_step_forward();
        elapsed += 8;  // Each step cycle takes ~8ms (4 steps * 2ms each)
    }
}