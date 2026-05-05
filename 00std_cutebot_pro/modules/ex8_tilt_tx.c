// ex8_tilt_tx.c
//
// Step 2: read the LSM303AGR accelerometer and broadcast (acc_x, acc_y)
// over the radio. The values are scaled to the [-100, +100] range, the
// same range used for the Cutebot wheel speeds.
//
// Radio packet (4 bytes total):
//   pdu[0] = 0       header (S0)
//   pdu[1] = 2       length of payload
//   pdu[2] = acc_x   signed (-100..+100)
//   pdu[3] = acc_y   signed (-100..+100)
//
// Accelerometer is on the INTERNAL I2C bus (TWI1):
//   SCL = P0.08, SDA = P0.16

#include "ex8_tilt_tx.h"
#include <nrf.h>
#include <nrf52833.h>
#include <stdint.h>
#include <stdio.h>

// LSM303AGR accelerometer
#define ACCEL_ADDR     0x19
#define REG_WHO_AM_I   0x0F
#define REG_CTRL_REG1  0x20
#define REG_OUT_X_L    0x28

// Internal I2C pins on micro:bit v2
#define PIN_SCL_INT    8
#define PIN_SDA_INT    16

// Radio payload buffer
static uint8_t pdu[] = {
    0x00,   // header
    2,      // payload length
    0,      // acc_x  (filled before each transmission)
    0,      // acc_y
};

volatile uint8_t who_am_i;
volatile int16_t accel_x;
volatile int16_t accel_y;
volatile int16_t accel_z;


// ----- I2C helpers (TWI1) --------------------------------------------------

static void accel_i2c_init(void) {
    NRF_P0->PIN_CNF[PIN_SCL_INT] = 0x00000602;
    NRF_P0->PIN_CNF[PIN_SDA_INT] = 0x00000602;

    NRF_TWI1->PSEL.SCL  = PIN_SCL_INT;
    NRF_TWI1->PSEL.SDA  = PIN_SDA_INT;
    NRF_TWI1->FREQUENCY = 0x01980000;      // 100 kbps
    NRF_TWI1->ADDRESS   = ACCEL_ADDR;
    NRF_TWI1->ENABLE    = 0x00000005;
}

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

    NRF_TWI1->SHORTS = 0x00000002;     // BB -> STOP

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


// ----- Radio TX ------------------------------------------------------------
// Same configuration as 00std_wireless_tx.c so it stays compatible.

static void radio_tx_init(void) {
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->MODE        = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER     = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0       = (8 << RADIO_PCNF0_LFLEN_Pos)   |
                             (1 << RADIO_PCNF0_S0LEN_Pos)   |
                             (0 << RADIO_PCNF0_S1LEN_Pos)   |
                             (2 << RADIO_PCNF0_CILEN_Pos)   |
                             (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) |
                             (3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1       = (sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos)  |
                             (0           << RADIO_PCNF1_STATLEN_Pos) |
                             (3           << RADIO_PCNF1_BALEN_Pos)   |
                             (RADIO_PCNF1_ENDIAN_Little    << RADIO_PCNF1_ENDIAN_Pos) |
                             (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0       = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS        = 1000;
    NRF_RADIO->CRCCNF      = (RADIO_CRCCNF_LEN_Three      << RADIO_CRCCNF_LEN_Pos) |
                             (RADIO_CRCCNF_SKIPADDR_Skip  << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT     = 0xFFFFUL;
    NRF_RADIO->CRCPOLY     = 0x00065b;
    NRF_RADIO->FREQUENCY   = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;

    // shortcuts: TXEN -> ramp up -> START, then on END -> DISABLE
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);
}

// blocking send: triggers TX and waits until the radio is fully disabled
static void radio_send(void) {
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_TXEN      = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
}


// ----- helpers -------------------------------------------------------------

static int clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Map raw accelerometer value (~ ±16000 for 1g) to a signed value
// in [-100, +100]. The negative divisor flips the axis sign so a tilt
// "forward" becomes positive (matches the MakeCode "/-7" convention).
// Saturates around ~70% of 1g, like MakeCode does.
static int8_t scale_accel(int16_t raw) {
    return (int8_t)clamp((int)raw / -112, -100, 100);
}


// ----- application ---------------------------------------------------------

void run_tilt_tx(void) {
    accel_i2c_init();

    who_am_i = accel_read_reg(REG_WHO_AM_I);
    printf("WHO_AM_I = 0x%02x (expected 0x33)\n", who_am_i);
    if (who_am_i != 0x33) {
        while (1);
    }

    accel_write_reg(REG_CTRL_REG1, 0x57);  // wake up: 100 Hz, normal, X/Y/Z

    radio_tx_init();

    while (1) {
        uint8_t xl = accel_read_reg(REG_OUT_X_L + 0);
        uint8_t xh = accel_read_reg(REG_OUT_X_L + 1);
        uint8_t yl = accel_read_reg(REG_OUT_X_L + 2);
        uint8_t yh = accel_read_reg(REG_OUT_X_L + 3);
        uint8_t zl = accel_read_reg(REG_OUT_X_L + 4);
        uint8_t zh = accel_read_reg(REG_OUT_X_L + 5);

        accel_x = (int16_t)((xh << 8) | xl);
        accel_y = (int16_t)((yh << 8) | yl);
        accel_z = (int16_t)((zh << 8) | zl);

        int8_t ax = scale_accel(accel_x);
        int8_t ay = scale_accel(accel_y);

        pdu[2] = (uint8_t)ax;
        pdu[3] = (uint8_t)ay;
        radio_send();

        printf("tx: acc_x=%4d  acc_y=%4d\n", ax, ay);

        for (volatile int i = 0; i < 200000; i++);
    }
}
