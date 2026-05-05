// driver_manette.h
//
// Hardware drivers used by the controller (manette) micro:bit:
//   - LSM303AGR accelerometer over the internal I2C bus (TWI1)
//   - NRF_RADIO in BLE long-range mode for transmission
//
// The application file (ex8_tilt_tx.c) only sees these high-level functions.

#ifndef DRIVER_MANETTE_H
#define DRIVER_MANETTE_H

#include <stdint.h>

// === Accelerometer (LSM303AGR) ===

// Configures TWI1 on the internal I2C pins (P0.08, P0.16) and wakes
// the chip up at 100 Hz, normal mode, X/Y/Z enabled.
void accel_init(void);

// Returns the WHO_AM_I register value. Should be 0x33 if the link works.
uint8_t accel_who_am_i(void);

// Reads the latest 16-bit signed acceleration values (raw, ~±16000 for 1g).
void accel_read_xyz(int16_t *x, int16_t *y, int16_t *z);


// === Radio TX ===

// Configures NRF_RADIO for transmission (BLE LR125Kbit, frequency 14,
// address 0xAAAAAAAA, 4-byte packet).
void radio_tx_init(void);

// Sends a packet [header=0, length=2, acc_x, acc_y]. Blocking.
void radio_tx_send(int8_t acc_x, int8_t acc_y);

#endif
