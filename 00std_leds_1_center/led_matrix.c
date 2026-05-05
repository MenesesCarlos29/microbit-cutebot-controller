#include "led_matrix.h"
#include "nrf_gpio.h"
#include <string.h>

// micro:bit v2 pin mapping
static const uint8_t rows[5] = {21, 22, 15, 24, 19};
static const uint8_t cols[5] = {28, 11, 31, 5, 30};

static uint8_t display[5][5];

void led_matrix_init(void)
{
    for (int i = 0; i < 5; i++)
    {
        nrf_gpio_cfg_output(rows[i]);
        nrf_gpio_cfg_output(cols[i]);
    }
}

void led_matrix_set_pixel(int row, int col, uint8_t val)
{
    if (row < 5 && col < 5)
        display[row][col] = val;
}

void led_matrix_clear(void)
{
    memset(display, 0, sizeof(display));
}

void led_matrix_refresh(void)
{
    static int current_row = 0;

    // disable all rows
    for (int i = 0; i < 5; i++)
        nrf_gpio_pin_clear(rows[i]);

    // set columns
    for (int c = 0; c < 5; c++)
    {
        if (display[current_row][c])
            nrf_gpio_pin_clear(cols[c]); // ON (active low)
        else
            nrf_gpio_pin_set(cols[c]);   // OFF
    }

    // enable current row
    nrf_gpio_pin_set(rows[current_row]);

    current_row = (current_row + 1) % 5;
}