#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>

void led_matrix_init(void);
void led_matrix_refresh(void);
void led_matrix_set_pixel(int row, int col, uint8_t val);
void led_matrix_clear(void);

#endif