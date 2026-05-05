#ifndef I2C_H
#define I2C_H

#include "nrf52833.h"
#include <stdint.h>

// I2C pins for micro:bit V2
#define I2C_SCL_PIN 8
#define I2C_SDA_PIN 16

void i2c_init(void);
void i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
uint8_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr);

#endif