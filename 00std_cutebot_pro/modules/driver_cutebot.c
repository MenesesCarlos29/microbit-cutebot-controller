// driver_cutebot.c — see driver_cutebot.h for the public API.

#include "driver_cutebot.h"
#include <nrf52833.h>


// ===========================================================================
// Cutebot Pro motors & headlights (external I2C bus, TWI0)
// ===========================================================================

// Sends 'len' bytes to the Cutebot. Bails out on NACK so we don't hang
// when the cutebot is not powered.
static void cutebot_i2c_send(uint8_t *buf, uint8_t len) {
    uint8_t i = 0;
    NRF_TWI0->EVENTS_TXDSENT = 0;
    NRF_TWI0->EVENTS_ERROR   = 0;
    NRF_TWI0->TXD            = buf[i];
    NRF_TWI0->TASKS_STARTTX  = 1;
    i++;
    while (i < len) {
        while (NRF_TWI0->EVENTS_TXDSENT == 0 && NRF_TWI0->EVENTS_ERROR == 0);
        if (NRF_TWI0->EVENTS_ERROR) {
            NRF_TWI0->EVENTS_ERROR = 0;
            NRF_TWI0->TASKS_STOP   = 1;
            return;
        }
        NRF_TWI0->EVENTS_TXDSENT = 0;
        NRF_TWI0->TXD            = buf[i];
        i++;
    }
    while (NRF_TWI0->EVENTS_TXDSENT == 0 && NRF_TWI0->EVENTS_ERROR == 0);
    NRF_TWI0->EVENTS_ERROR = 0;
    NRF_TWI0->TASKS_STOP   = 1;
}

// Sends one motor command. motor_id: 0x01 or 0x02.
// (On this Cutebot Pro, 0x02 drives the LEFT wheel and 0x01 the RIGHT one.)
static void send_motor(uint8_t motor_id, int speed) {
    uint8_t direction = (speed >= 0) ? 0x01 : 0x00;
    int     mag       = (speed >= 0) ? speed : -speed;
    if (mag > 100) mag = 100;
    uint8_t buf[7] = { 0x99, 0x01, motor_id, direction, (uint8_t)mag, 0x00, 0x88 };
    cutebot_i2c_send(buf, sizeof(buf));
}

void cutebot_motors_init(void) {
    NRF_P0->PIN_CNF[26] = 0x00000602;     // SCL
    NRF_P1->PIN_CNF[0]  = 0x00000602;     // SDA

    NRF_TWI0->ENABLE    = 0x00000005;
    NRF_TWI0->PSEL.SCL  = 0x0000001a;     // P0.26
    NRF_TWI0->PSEL.SDA  = 0x00000020;     // P1.00
    NRF_TWI0->FREQUENCY = 0x01980000;     // 100 kbps
    NRF_TWI0->ADDRESS   = 0x10;
}

void update_motors(int left, int right) {
    send_motor(0x02, left);    // 0x02 -> left wheel
    send_motor(0x01, right);   // 0x01 -> right wheel
}

void set_headlights(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t buf[7] = { 0x99, 0x0f, 0x03, r, g, b, 0x88 };
    cutebot_i2c_send(buf, sizeof(buf));
}


// ===========================================================================
// Radio RX (NRF_RADIO)
// ===========================================================================

static uint8_t pdu[8] = { 0 };

volatile int8_t   rx_acc_x;
volatile int8_t   rx_acc_y;
volatile uint32_t rx_count;
volatile uint32_t rx_bad_crc;

void radio_rx_init(void) {
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->MODE        = 5;             // Ble_LR125Kbit
    NRF_RADIO->TXPOWER     = 8;
    NRF_RADIO->PCNF0       = (8U << 0)  | (1U << 8)  | (0U << 16)
                           | (2U << 22) | (3U << 24) | (3U << 29);
    NRF_RADIO->PCNF1       = (sizeof(pdu) << 0) | (3U << 16);
    NRF_RADIO->BASE0       = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = 1;
    NRF_RADIO->TIFS        = 0;
    NRF_RADIO->CRCCNF      = (3U << 0) | (1U << 8);
    NRF_RADIO->CRCINIT     = 0xFFFFUL;
    NRF_RADIO->CRCPOLY     = 0x00065b;
    NRF_RADIO->FREQUENCY   = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;

    // READY_START | END_DISABLE | DISABLED_RXEN: continuous reception
    NRF_RADIO->SHORTS = (1U << 0) | (1U << 1) | (1U << 3);

    NRF_RADIO->INTENCLR = 0xffffffff;
    NRF_RADIO->INTENSET = (1U << 4);        // DISABLED interrupt
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->TASKS_RXEN = 1;
}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;

        if (NRF_RADIO->CRCSTATUS != 1) {
            rx_bad_crc++;
        } else if (pdu[1] == 2) {
            rx_acc_x = (int8_t)pdu[2];
            rx_acc_y = (int8_t)pdu[3];
            rx_count++;
        }
    }
}


// ===========================================================================
// Ultrasonic sensor (HC-SR04, TIMER0 at 1 MHz for 1us timing)
// ===========================================================================

#define US_TRIG_PIN  10        // P0.10 = micro:bit edge connector P8
#define US_ECHO_PIN  12        // P0.12 = micro:bit edge connector P12

void ultrasonic_init(void) {
    NRF_P0->PIN_CNF[US_TRIG_PIN] = 0x00000003;   // output
    NRF_P0->OUTCLR = (1U << US_TRIG_PIN);
    NRF_P0->PIN_CNF[US_ECHO_PIN] = 0x00000000;   // input

    NRF_TIMER0->TASKS_STOP  = 1;
    NRF_TIMER0->MODE        = 0;     // timer
    NRF_TIMER0->BITMODE     = 3;     // 32-bit
    NRF_TIMER0->PRESCALER   = 4;     // 16 MHz / 2^4 = 1 MHz
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;
}

uint32_t now_us(void) {
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    return NRF_TIMER0->CC[0];
}

int ultrasonic_distance_cm(void) {
    // 10 us trigger pulse
    NRF_P0->OUTSET = (1U << US_TRIG_PIN);
    uint32_t t = now_us();
    while (now_us() - t < 10);
    NRF_P0->OUTCLR = (1U << US_TRIG_PIN);

    // wait for echo HIGH (timeout 5 ms)
    t = now_us();
    while ((NRF_P0->IN & (1U << US_ECHO_PIN)) == 0) {
        if (now_us() - t > 5000) return 999;
    }

    uint32_t echo_start = now_us();

    // wait for echo LOW (timeout 25 ms ~ 430 cm)
    while (NRF_P0->IN & (1U << US_ECHO_PIN)) {
        if (now_us() - echo_start > 25000) return 999;
    }

    uint32_t echo_us = now_us() - echo_start;
    return (int)(echo_us / 58);
}


// ===========================================================================
// Speaker (P0.00)
// ===========================================================================

void beep(uint32_t freq_hz, uint32_t duration_ms) {
    NRF_P0->PIN_CNF[0] = 0x00000003;
    uint32_t half_us = 500000U / freq_hz;
    uint32_t start   = now_us();
    while (now_us() - start < duration_ms * 1000U) {
        NRF_P0->OUTSET = (1U << 0);
        uint32_t t = now_us();
        while (now_us() - t < half_us);
        NRF_P0->OUTCLR = (1U << 0);
        t = now_us();
        while (now_us() - t < half_us);
    }
    NRF_P0->OUTCLR = (1U << 0);
}


// ===========================================================================
// 5x5 LED matrix
// ===========================================================================
// Pins (micro:bit v2): row HIGH, col LOW lights one LED.
//   ROW1=P0.21  ROW2=P0.22  ROW3=P0.15  ROW4=P0.24  ROW5=P0.19
//   COL1=P0.28  COL2=P0.11  COL3=P0.31  COL4=P1.05  COL5=P0.30

void leds_all_off(void) {
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

void led_on(int row, int col) {
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
                      (col == 3) ? 31 : 30;     // col 5
        NRF_P0->PIN_CNF[col_pin] = 0x00000003;
        NRF_P0->OUTCLR = (1U << col_pin);
    }
}
