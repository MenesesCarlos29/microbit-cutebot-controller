#include "nrf52833.h"
#include "i2c.h"
#include "accelerometer.h"
#include "led_matrix.h"

// Simple delay function
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 4000; i++) {
        __NOP();
    }
}

// Tilt threshold (adjust for sensitivity)
#define TILT_THRESHOLD 20

int main(void) {
    // Initialize all peripherals
    i2c_init();
    accelerometer_init();
    led_matrix_init();
    
    uint32_t refresh_counter = 0;
    
    while (1) {
        // Read X-axis acceleration
        int16_t x_accel = accelerometer_read_x();
        
        // Clear LED matrix
        led_matrix_clear();
        
        // Determine tilt direction and light LEDs
        if (x_accel > TILT_THRESHOLD) {
            // Tilted right - light rightmost column (column 4)
            led_matrix_set_column(4, 1);
        } 
        else if (x_accel < -TILT_THRESHOLD) {
            // Tilted left - light leftmost column (column 0)
            led_matrix_set_column(0, 1);
        } 
        else {
            // Level - light center column (column 2)
            led_matrix_set_column(2, 1);
        }
        
        // Refresh LED matrix multiple times for brightness
        // (multiplexing needs to happen fast enough)
        for (int i = 0; i < 100; i++) {
            led_matrix_refresh();
            delay_ms(1);
        }
    }
    
    return 0;
}