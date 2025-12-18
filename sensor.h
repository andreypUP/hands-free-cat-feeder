#ifndef SENSOR_H
#define SENSOR_H
#include <stdint.h>
void sensor_init(void);
void sensor_trigger(void);
uint8_t sensor_is_measurement_complete(void);
uint16_t sensor_get_pulse_width(void);
#endif