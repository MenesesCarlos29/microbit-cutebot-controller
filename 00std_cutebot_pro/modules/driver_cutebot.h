// driver_cutebot.h
//
// Hardware drivers used by the robot (cutebot) micro:bit:
//   - Cutebot Pro motors and headlights over the external I2C bus (TWI0)
//   - NRF_RADIO in BLE long-range mode for reception
//   - HC-SR04 ultrasonic sensor (TIMER0 used for 1us timing)
//   - Onboard speaker (P0.00)
//   - 5x5 LED matrix
//
// The application file (ex8_tilt_rx.c) only sees these high-level functions.

#ifndef DRIVER_CUTEBOT_H
#define DRIVER_CUTEBOT_H

#include <stdint.h>


// === Motors and headlights (I2C TWI0 to Cutebot Pro) =======================

void cutebot_motors_init(void);

// Drive both wheels. Speeds are signed in [-100, +100].
//   positive -> forward, negative -> backward, 0 -> stop.
void update_motors(int left, int right);

// Set both headlight RGB LEDs to the given color. (0,0,0) turns them off.
void set_headlights(uint8_t r, uint8_t g, uint8_t b);

// Set only the left / only the right headlight (the other is left untouched).
void set_left_headlight(uint8_t r, uint8_t g, uint8_t b);
void set_right_headlight(uint8_t r, uint8_t g, uint8_t b);


// === Radio RX (NRF_RADIO) ==================================================

void radio_rx_init(void);

// Latest received values, updated by RADIO_IRQHandler in driver_cutebot.c.
extern volatile int8_t   rx_acc_x;
extern volatile int8_t   rx_acc_y;
extern volatile uint32_t rx_count;     // increments on each valid packet
extern volatile uint32_t rx_bad_crc;   // increments on each CRC failure

extern volatile uint8_t rx_btn;

// === Ultrasonic sensor (HC-SR04 + TIMER0) ==================================

void     ultrasonic_init(void);

// Microsecond counter (1us tick), useful for any timing.
uint32_t now_us(void);

// Returns the distance in cm, or 999 if no echo / out of range.
int      ultrasonic_distance_cm(void);


// === Speaker (onboard P0.00) ===============================================

// Blocking square-wave beep. Returns when the duration has elapsed.
void beep(uint32_t freq_hz, uint32_t duration_ms);


// === 5x5 LED matrix ========================================================

void leds_all_off(void);

// Lights one LED. row and col are in 1..5 (LED11 = top-left).
void led_on(int row, int col);

#endif
