#include "accelerometer.h"
#include "twi.h"

#define ACC_ADDR 0x19

void accel_init(void)
{
    uint8_t cmd[2];

    // CTRL_REG1_A (0x20): 100 Hz, enable XYZ
    cmd[0] = 0x20;
    cmd[1] = 0x57;

    twi_write(ACC_ADDR, cmd, 2);
}

int16_t accel_read_x(void)
{
    uint8_t reg = 0x28 | 0x80; // auto increment
    uint8_t data[2];

    // write register address
    twi_write(ACC_ADDR, &reg, 1);

    // read 2 bytes
    twi_read(ACC_ADDR, data, 2);

    return (int16_t)(data[1] << 8 | data[0]);
}