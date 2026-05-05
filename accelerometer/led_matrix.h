#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>

void led_matrix_init(void);
void led_matrix_clear(void);
void led_matrix_set_pixel(uint8_t row, uint8_t col, uint8_t on);
void led_matrix_set_column(uint8_t col, uint8_t on);
void led_matrix_refresh(void);

#endif