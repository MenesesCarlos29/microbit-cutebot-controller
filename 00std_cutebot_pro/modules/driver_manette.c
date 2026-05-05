// driver_manette.c — see driver_manette.h for the public API.

#include "driver_manette.h"
#include <nrf52833.h>


// ===========================================================================
// LSM303AGR accelerometer (internal I2C bus, TWI1)
// ===========================================================================

#define ACCEL_ADDR     0x19
#define REG_WHO_AM_I   0x0F
#define REG_CTRL_REG1  0x20
#define REG_OUT_X_L    0x28

#define PIN_SCL_INT    8     // P0.08
#define PIN_SDA_INT    16    // P0.16


static void accel_write_reg(uint8_t reg, uint8_t value) {
    NRF_TWI1->EVENTS_TXDSENT = 0;
    NRF_TWI1->EVENTS_STOPPED = 0;
    NRF_TWI1->TASKS_STARTTX  = 1;

    NRF_TWI1->TXD = reg;
    while (NRF_TWI1->EVENTS_TXDSENT == 0);
    NRF_TWI1->EVENTS_TXDSENT = 0;

    NRF_TWI1->TXD = value;
    while (NRF_TWI1->EVENTS_TXDSENT == 0);
    NRF_TWI1->EVENTS_TXDSENT = 0;

    NRF_TWI1->TASKS_STOP = 1;
    while (NRF_TWI1->EVENTS_STOPPED == 0);
    NRF_TWI1->EVENTS_STOPPED = 0;
}

static uint8_t accel_read_reg(uint8_t reg) {
    uint8_t value;

    NRF_TWI1->EVENTS_TXDSENT = 0;
    NRF_TWI1->EVENTS_STOPPED = 0;
    NRF_TWI1->TASKS_STARTTX  = 1;
    NRF_TWI1->TXD            = reg;
    while (NRF_TWI1->EVENTS_TXDSENT == 0);
    NRF_TWI1->EVENTS_TXDSENT = 0;

    // STOP automatically when the byte arrives (BB_STOP shortcut, bit 1)
    NRF_TWI1->SHORTS = 0x00000002;

    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->TASKS_STARTRX   = 1;
    while (NRF_TWI1->EVENTS_RXDREADY == 0);
    value = NRF_TWI1->RXD;

    while (NRF_TWI1->EVENTS_STOPPED == 0);
    NRF_TWI1->EVENTS_STOPPED  = 0;
    NRF_TWI1->EVENTS_RXDREADY = 0;
    NRF_TWI1->SHORTS          = 0;

    return value;
}

void accel_init(void) {
    NRF_P0->PIN_CNF[PIN_SCL_INT] = 0x00000602;
    NRF_P0->PIN_CNF[PIN_SDA_INT] = 0x00000602;

    NRF_TWI1->PSEL.SCL  = PIN_SCL_INT;
    NRF_TWI1->PSEL.SDA  = PIN_SDA_INT;
    NRF_TWI1->FREQUENCY = 0x01980000;     // 100 kbps
    NRF_TWI1->ADDRESS   = ACCEL_ADDR;
    NRF_TWI1->ENABLE    = 0x00000005;

    // wake up: 100 Hz, normal mode, X/Y/Z enabled
    accel_write_reg(REG_CTRL_REG1, 0x57);
}

uint8_t accel_who_am_i(void) {
    return accel_read_reg(REG_WHO_AM_I);
}

void accel_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t xl = accel_read_reg(REG_OUT_X_L + 0);
    uint8_t xh = accel_read_reg(REG_OUT_X_L + 1);
    uint8_t yl = accel_read_reg(REG_OUT_X_L + 2);
    uint8_t yh = accel_read_reg(REG_OUT_X_L + 3);
    uint8_t zl = accel_read_reg(REG_OUT_X_L + 4);
    uint8_t zh = accel_read_reg(REG_OUT_X_L + 5);

    *x = (int16_t)((xh << 8) | xl);
    *y = (int16_t)((yh << 8) | yl);
    *z = (int16_t)((zh << 8) | zl);
}


// ===========================================================================
// Radio TX (NRF_RADIO)
// ===========================================================================

// 4-byte payload: [header=0, length=2, acc_x, acc_y]
static uint8_t pdu[] = { 0x00, 2, 0, 0 };

void radio_tx_init(void) {
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->MODE        = 5;             // Ble_LR125Kbit
    NRF_RADIO->TXPOWER     = 8;             // +8 dBm
    NRF_RADIO->PCNF0       = (8U << 0)  | (1U << 8)  | (0U << 16)
                           | (2U << 22) | (3U << 24) | (3U << 29);
    NRF_RADIO->PCNF1       = (sizeof(pdu) << 0) | (3U << 16);
    NRF_RADIO->BASE0       = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->TIFS        = 1000;
    NRF_RADIO->CRCCNF      = (3U << 0) | (1U << 8);
    NRF_RADIO->CRCINIT     = 0xFFFFUL;
    NRF_RADIO->CRCPOLY     = 0x00065b;
    NRF_RADIO->FREQUENCY   = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;

    // SHORTS: READY -> START and END -> DISABLE
    NRF_RADIO->SHORTS      = (1U << 0) | (1U << 1);
}

void radio_tx_send(int8_t acc_x, int8_t acc_y) {
    pdu[2] = (uint8_t)acc_x;
    pdu[3] = (uint8_t)acc_y;

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_TXEN      = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
}
