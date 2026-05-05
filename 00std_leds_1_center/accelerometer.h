#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>

void accel_init(void);
int16_t accel_read_x(void);

#endif