#include "accelerometer.h"
#include "i2c.h"

// Simple delay function
static void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __NOP();
    }
}

void accelerometer_init(void) {
    // Configure accelerometer:
    // ODR = 10Hz, Normal mode, X,Y,Z enabled
    // CTRL_REG1: 0x27 = 0010 0111
    // Bit 7-4: ODR = 0010 (10 Hz)
    // Bit 3: Low power mode = 0 (normal mode)
    // Bit 2-0: Z,Y,X enable = 111
    i2c_write_byte(ACCEL_I2C_ADDR, ACCEL_CTRL_REG1, 0x27);
    delay_ms(10);
}

int16_t accelerometer_read_x(void) {
    uint8_t low = i2c_read_byte(ACCEL_I2C_ADDR, ACCEL_OUT_X_L);
    uint8_t high = i2c_read_byte(ACCEL_I2C_ADDR, ACCEL_OUT_X_H);
    
    // Combine into 16-bit signed value
    int16_t x = (int16_t)((high << 8) | low);
    return x;
}

int16_t accelerometer_read_y(void) {
    uint8_t low = i2c_read_byte(ACCEL_I2C_ADDR, 0x2A);  // OUT_Y_L
    uint8_t high = i2c_read_byte(ACCEL_I2C_ADDR, 0x2B); // OUT_Y_H
    
    int16_t y = (int16_t)((high << 8) | low);
    return y;
}