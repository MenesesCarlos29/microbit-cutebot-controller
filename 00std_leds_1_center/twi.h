#ifndef TWI_H
#define TWI_H

#include <stdint.h>

void twi_init(void);
void twi_write(uint8_t addr, uint8_t *data, uint8_t len);
void twi_read(uint8_t addr, uint8_t *data, uint8_t len);

#endif