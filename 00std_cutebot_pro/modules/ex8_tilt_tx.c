// ex8_tilt_tx.c
//
// Step 2: read the LSM303AGR accelerometer and broadcast (acc_x, acc_y)
// over the radio. The values are scaled to the [-100, +100] range.
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
// Same configuration as 00std_wireless_tx.c so the link is compatible.
// Values written as literals (the SDK macros are not always picked up
// by SES's indexer/compiler in this project layout).

static void radio_tx_init(void) {
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->MODE        = 5;             // Ble_LR125Kbit (long range)
    NRF_RADIO->TXPOWER     = 8;             // +8 dBm

    // PCNF0: LFLEN=8 (bit0), S0LEN=1 (bit8), S1LEN=0 (bit16),
    //        CILEN=2 (bit22), PLEN=LongRange=3 (bit24), TERMLEN=3 (bit29)
    NRF_RADIO->PCNF0       = (8U << 0) | (1U << 8) | (0U << 16)
                           | (2U << 22) | (3U << 24) | (3U << 29);

    // PCNF1: MAXLEN=sizeof(pdu) (bit0), STATLEN=0 (bit8), BALEN=3 (bit16),
    //        ENDIAN=Little=0 (bit24), WHITEEN=Disabled=0 (bit25)
    NRF_RADIO->PCNF1       = (sizeof(pdu) << 0) | (3U << 16);

    NRF_RADIO->BASE0       = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = 1;             // ADDR0 enabled
    NRF_RADIO->TIFS        = 1000;

    // CRCCNF: LEN=3 bytes (bit0), SKIPADDR=Skip=1 (bit8)
    NRF_RADIO->CRCCNF      = (3U << 0) | (1U << 8);
    NRF_RADIO->CRCINIT     = 0xFFFFUL;
    NRF_RADIO->CRCPOLY     = 0x00065b;
    NRF_RADIO->FREQUENCY   = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;

    // SHORTS: READY_START (bit0) + END_DISABLE (bit1)
    NRF_RADIO->SHORTS      = (1U << 0) | (1U << 1);
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

// SES's minimal printf doesn't handle %d, so we build the output by hand.
static void print_str(const char *s) {
    while (*s) putchar(*s++);
}

static void print_int(int v) {
    char buf[8];
    int  i = 0;
    if (v < 0) { putchar('-'); v = -v; }
    if (v == 0) { putchar('0'); return; }
    while (v > 0) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    while (i > 0) putchar(buf[--i]);
}

// Map raw accelerometer value (~ ±16000 for 1g) to a signed value
// in [-100, +100]. The chip orientation on this board makes "tilt forward"
// produce a NEGATIVE raw value, so we use a positive divisor to flip it.
// Saturates around ~70% of 1g.
static int8_t scale_accel(int16_t raw) {
    return (int8_t)clamp((int)raw / 112, -100, 100);
}


// ----- LED matrix helpers --------------------------------------------------
// Pins (micro:bit v2): row HIGH, column LOW lights one LED.
//   ROW1=P0.21  ROW2=P0.22  ROW3=P0.15  ROW4=P0.24  ROW5=P0.19
//   COL1=P0.28  COL2=P0.11  COL3=P0.31  COL4=P1.05  COL5=P0.30

static void leds_all_off(void) {
    NRF_P0->PIN_CNF[21] = 0x00000002;
    NRF_P0->PIN_CNF[22] = 0x00000002;
    NRF_P0->PIN_CNF[15] = 0x00000002;
    NRF_P0->PIN_CNF[24] = 0x00000002;
    NRF_P0->PIN_CNF[19] = 0x00000002;
    NRF_P0->PIN_CNF[28] = 0x00000002;
    NRF_P0->PIN_CNF[11] = 0x00000002;
    NRF_P0->PIN_CNF[31] = 0x00000002;
    NRF_P1->PIN_CNF[5]  = 0x00000002;
    NRF_P0->PIN_CNF[30] = 0x00000002;
}

static void led_on(int row, int col) {
    leds_all_off();

    int row_pin = (row == 1) ? 21 :
                  (row == 2) ? 22 :
                  (row == 3) ? 15 :
                  (row == 4) ? 24 : 19;
    NRF_P0->PIN_CNF[row_pin] = 0x00000003;
    NRF_P0->OUTSET = (1U << row_pin);

    if (col == 4) {
        NRF_P1->PIN_CNF[5] = 0x00000003;
        NRF_P1->OUTCLR = (1U << 5);
    } else {
        int col_pin = (col == 1) ? 28 :
                      (col == 2) ? 11 :
                      (col == 3) ? 31 : 30;
        NRF_P0->PIN_CNF[col_pin] = 0x00000003;
        NRF_P0->OUTCLR = (1U << col_pin);
    }
}


// ----- application ---------------------------------------------------------

void run_tilt_tx(void) {
    accel_i2c_init();

    who_am_i = accel_read_reg(REG_WHO_AM_I);
    print_str("WHO_AM_I = 0x");
    print_int(who_am_i);
    print_str(" (expected 51)\n");
    if (who_am_i != 0x33) {
        while (1);
    }

    accel_write_reg(REG_CTRL_REG1, 0x57);  // wake up: 100 Hz, normal, X/Y/Z

    radio_tx_init();

    int tx_blink = 0;       // alternates between two LEDs to show TX activity

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

        // visual proof that we are transmitting: alternate two LEDs
        tx_blink ^= 1;
        if (tx_blink) led_on(1, 1);    // top-left
        else          led_on(5, 5);    // bottom-right

        print_str("tx: acc_x=");
        print_int(ax);
        print_str("  acc_y=");
        print_int(ay);
        putchar('\n');

        for (volatile int i = 0; i < 200000; i++);
    }
}
