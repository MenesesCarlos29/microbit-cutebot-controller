#include <stdint.h>
#include "twi.h"
#include "accelerometer.h"
#include "led_matrix.h"

#define THRESHOLD 8000

int main(void)
{
    twi_init();
    accel_init();
    led_matrix_init();

    while (1)
    {
        int16_t x = accel_read_x();

        led_matrix_clear();

        if (x > THRESHOLD)
        {
            led_matrix_set_pixel(2, 4, 1); // right
        }
        else if (x < -THRESHOLD)
        {
            led_matrix_set_pixel(2, 0, 1); // left
        }

        // refresh display continuously
        for (int i = 0; i < 500; i++)
        {
            led_matrix_refresh();
        }
    }
}