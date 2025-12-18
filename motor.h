#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_step_forward(void);
void motor_step_backward(void);
void motor_forward(uint16_t steps);
void motor_backward(uint16_t steps);
void motor_stop(void);
void motor_hold(void);
void motor_rotate_degrees(int16_t degrees);
void motor_forward_smooth(uint16_t steps);
void motor_run_for_duration(uint16_t duration_ms);
#endif