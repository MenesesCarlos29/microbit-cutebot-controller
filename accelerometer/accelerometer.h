#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>

// LSM303AGR I2C address (micro:bit V2)
#define ACCEL_I2C_ADDR 0x19

// LSM303AGR registers
#define ACCEL_CTRL_REG1 0x20
#define ACCEL_OUT_X_L   0x28
#define ACCEL_OUT_X_H   0x29

void accelerometer_init(void);
int16_t accelerometer_read_x(void);
int16_t accelerometer_read_y(void);

#endif