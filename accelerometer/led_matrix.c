#include "led_matrix.h"
#include "nrf52833.h"

// micro:bit V2 LED matrix pins
static const uint8_t row_pins[5] = {21, 22, 15, 24, 19};
static const uint8_t col_pins[5] = {28, 11, 31, 30, 37};

// LED state buffer (5x5 matrix)
static uint8_t led_state[5][5] = {0};
static uint8_t current_row = 0;

void led_matrix_init(void) {
    // Configure row pins as outputs
    // DIR=1 (Output), INPUT=1 (Disconnect), PULL=0 (Disabled), DRIVE=0 (S0S1)
    for (int i = 0; i < 5; i++) {
        NRF_P0->PIN_CNF[row_pins[i]] = 0x00000003;  // Output, Disconnect, No pull, Standard drive
        NRF_P0->OUTCLR = (1 << row_pins[i]);  // Set low initially
    }
    
    // Configure column pins as outputs
    for (int i = 0; i < 5; i++) {
        // Handle both P0 and P1 ports
        if (col_pins[i] < 32) {
            NRF_P0->PIN_CNF[col_pins[i]] = 0x00000003;
            NRF_P0->OUTSET = (1 << col_pins[i]);  // Set high initially (LED off)
        } else {
            uint8_t pin = col_pins[i] - 32;
            NRF_P1->PIN_CNF[pin] = 0x00000003;
            NRF_P1->OUTSET = (1 << pin);  // Set high initially (LED off)
        }
    }
}

void led_matrix_clear(void) {
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
            led_state[r][c] = 0;
        }
    }
}

void led_matrix_set_pixel(uint8_t row, uint8_t col, uint8_t on) {
    if (row < 5 && col < 5) {
        led_state[row][col] = on ? 1 : 0;
    }
}

void led_matrix_set_column(uint8_t col, uint8_t on) {
    if (col < 5) {
        for (int row = 0; row < 5; row++) {
            led_state[row][col] = on ? 1 : 0;
        }
    }
}

void led_matrix_refresh(void) {
    // Turn off all rows first
    for (int i = 0; i < 5; i++) {
        NRF_P0->OUTCLR = (1 << row_pins[i]);
    }
    
    // Set column states for current row
    for (int col = 0; col < 5; col++) {
        if (col_pins[col] < 32) {
            if (led_state[current_row][col]) {
                NRF_P0->OUTCLR = (1 << col_pins[col]);  // Active low
            } else {
                NRF_P0->OUTSET = (1 << col_pins[col]);
            }
        } else {
            uint8_t pin = col_pins[col] - 32;
            if (led_state[current_row][col]) {
                NRF_P1->OUTCLR = (1 << pin);
            } else {
                NRF_P1->OUTSET = (1 << pin);
            }
        }
    }
    
    // Turn on current row
    NRF_P0->OUTSET = (1 << row_pins[current_row]);
    
    // Move to next row
    current_row = (current_row + 1) % 5;
}