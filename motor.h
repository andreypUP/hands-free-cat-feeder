#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_start_dispensing(uint16_t duration_ms);
void motor_step_forward(void);  // Added missing function
void motor_stop(void);
uint8_t motor_is_running(void);
uint16_t motor_get_remaining_time(void);

#endif