// ex8_tilt_rx.c
//
// Step 3 receiver: listens on the radio, parses (acc_x, acc_y), turns
// them into left/right wheel speeds and drives the Cutebot Pro motors
// over I2C (TWI0).
//
// Packet layout (matches ex8_tilt_tx.c):
//   pdu[0] = 0       header
//   pdu[1] = 2       payload length
//   pdu[2] = acc_x   signed (-100..+100)
//   pdu[3] = acc_y   signed (-100..+100)
//
// Differential drive (same math as MakeCode):
//   right = (acc_y + acc_x) / 2
//   left  = (acc_y - acc_x) / 2
// Both clamped to [-100, +100]. Sign -> direction, |speed| -> speed byte.

#include "ex8_tilt_rx.h"
#include <nrf52833.h>
#include <stdint.h>
#include <stdio.h>

#define DEADZONE 5    // |speed| below this is treated as 0 (kills jitter)

// Ultrasonic sensor on the Cutebot Pro front (HC-SR04).
// Most variants wire it to P8 (trigger) and P12 (echo) of the micro:bit
// edge connector, which map to P0.10 and P0.12. If your board is wired
// differently, just change these two values.
#define US_TRIG_PIN     10
#define US_ECHO_PIN     12
#define OBSTACLE_CM     30        // trigger evade if distance below this
#define SAFE_CM         45        // exit evade once distance is above this
#define EVADE_SPEED    -40        // backward speed during evade
#define EVADE_TIMEOUT_US 3000000  // safety: stop evade after 3 s no matter what


// Buffer big enough for header + length + a few payload bytes.
static uint8_t pdu[8] = { 0 };

// Latest received values (filled in the IRQ, printed in main).
static volatile int8_t   rx_acc_x;
static volatile int8_t   rx_acc_y;
static volatile uint32_t rx_count;
static volatile uint32_t rx_bad_crc;


// ----- minimal print helpers (SES's printf doesn't handle %d) --------------

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


// ----- Cutebot I2C (TWI0) --------------------------------------------------
// External I2C bus on micro:bit edge connector (SCL=P0.26, SDA=P1.00),
// Cutebot Pro slave address = 0x10. Same setup as ex3_speed_up.c.

static void cutebot_i2c_init(void) {
    NRF_P0->PIN_CNF[26] = 0x00000602;     // SCL
    NRF_P1->PIN_CNF[0]  = 0x00000602;     // SDA

    NRF_TWI0->ENABLE    = 0x00000005;
    NRF_TWI0->PSEL.SCL  = 0x0000001a;     // Pin 26 on port 0
    NRF_TWI0->PSEL.SDA  = 0x00000020;     // Pin  0 on port 1
    NRF_TWI0->FREQUENCY = 0x01980000;     // 100 kbps
    NRF_TWI0->ADDRESS   = 0x10;
}

// Sends 'len' bytes to the Cutebot. If the slave doesn't ack (cutebot off
// or not connected), EVENTS_ERROR fires and we bail out instead of hanging.
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

// Send the speed for ONE motor.
//   motor_id : 0x01 = left, 0x02 = right
//   speed    : signed, [-100, +100]; sign goes into the direction byte
static void send_motor(uint8_t motor_id, int speed) {
    uint8_t direction = (speed >= 0) ? 0x01 : 0x00;
    int     mag       = (speed >= 0) ? speed : -speed;
    if (mag > 100) mag = 100;

    uint8_t buf[7] = { 0x99, 0x01, motor_id, direction, (uint8_t)mag, 0x00, 0x88 };
    cutebot_i2c_send(buf, sizeof(buf));
}

// Note on motor IDs: on this Cutebot Pro, 0x02 drives the LEFT wheel and
// 0x01 drives the RIGHT one (opposite of the comment in 00std_cutebot_pro.c
// which was written for the original Cutebot).
static void update_motors(int left, int right) {
    send_motor(0x02, left);    // left wheel
    send_motor(0x01, right);   // right wheel
}

// Headlights: same I2C protocol, motor_id replaced by LED select.
//   led: 0x01 = right, 0x02 = left, 0x03 = both
static void set_headlights(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t buf[7] = { 0x99, 0x0f, 0x03, r, g, b, 0x88 };
    cutebot_i2c_send(buf, sizeof(buf));
}

static int clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


// ----- Ultrasonic (HC-SR04) ------------------------------------------------
// Uses TIMER0 in 1 MHz mode so each tick == 1 us. Returns distance in cm,
// or 999 if no echo (no obstacle in range).

static void ultrasonic_init(void) {
    // trig: output, drive low
    NRF_P0->PIN_CNF[US_TRIG_PIN] = 0x00000003;
    NRF_P0->OUTCLR = (1U << US_TRIG_PIN);

    // echo: input, no pull
    NRF_P0->PIN_CNF[US_ECHO_PIN] = 0x00000000;

    // free-running TIMER0 at 1 MHz (32-bit)
    NRF_TIMER0->TASKS_STOP  = 1;
    NRF_TIMER0->MODE        = 0;     // timer mode
    NRF_TIMER0->BITMODE     = 3;     // 32-bit
    NRF_TIMER0->PRESCALER   = 4;     // 16 MHz / 2^4 = 1 MHz
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;
}

static uint32_t now_us(void) {
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    return NRF_TIMER0->CC[0];
}

static int ultrasonic_distance_cm(void) {
    // 10 us trigger pulse
    NRF_P0->OUTSET = (1U << US_TRIG_PIN);
    uint32_t t = now_us();
    while (now_us() - t < 10);
    NRF_P0->OUTCLR = (1U << US_TRIG_PIN);

    // wait for echo to go HIGH (timeout 5 ms)
    t = now_us();
    while ((NRF_P0->IN & (1U << US_ECHO_PIN)) == 0) {
        if (now_us() - t > 5000) return 999;
    }

    uint32_t echo_start = now_us();

    // wait for echo to go LOW (timeout 25 ms ~ 430 cm)
    while (NRF_P0->IN & (1U << US_ECHO_PIN)) {
        if (now_us() - echo_start > 25000) return 999;
    }

    uint32_t echo_us = now_us() - echo_start;
    return (int)(echo_us / 58);
}


// ----- speaker -------------------------------------------------------------
// Plays a square wave on P0.00 (the onboard micro:bit v2 speaker).
// Blocking: returns once the duration has elapsed.

static void beep(uint32_t freq_hz, uint32_t duration_ms) {
    NRF_P0->PIN_CNF[0] = 0x00000003;     // output

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


// ----- Radio RX setup ------------------------------------------------------
// Same configuration as 00std_wireless_rx.c so the link is compatible.
// Values written as literals to dodge SES indexer / macro-resolution issues.

static void radio_rx_init(void) {
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    NRF_RADIO->MODE        = 5;             // Ble_LR125Kbit
    NRF_RADIO->TXPOWER     = 8;             // +8 dBm

    // PCNF0: LFLEN=8, S0LEN=1, S1LEN=0, CILEN=2, PLEN=LongRange (3), TERMLEN=3
    NRF_RADIO->PCNF0       = (8U << 0) | (1U << 8) | (0U << 16)
                           | (2U << 22) | (3U << 24) | (3U << 29);

    // PCNF1: MAXLEN=sizeof(pdu), STATLEN=0, BALEN=3, ENDIAN=Little=0, WHITEEN=Disabled=0
    NRF_RADIO->PCNF1       = (sizeof(pdu) << 0) | (3U << 16);

    NRF_RADIO->BASE0       = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = 1;             // ADDR0 enabled
    NRF_RADIO->TIFS        = 0;

    NRF_RADIO->CRCCNF      = (3U << 0) | (1U << 8);   // LEN=3 bytes, SKIPADDR=Skip
    NRF_RADIO->CRCINIT     = 0xFFFFUL;
    NRF_RADIO->CRCPOLY     = 0x00065b;
    NRF_RADIO->FREQUENCY   = 14;
    NRF_RADIO->PACKETPTR   = (uint32_t)pdu;

    // SHORTS: READY_START (bit0) | END_DISABLE (bit1) | DISABLED_RXEN (bit3)
    // -> the radio listens continuously, no need to retrigger from main
    NRF_RADIO->SHORTS = (1U << 0) | (1U << 1) | (1U << 3);

    NRF_RADIO->INTENCLR = 0xffffffff;
    NRF_RADIO->INTENSET = (1U << 4);        // enable DISABLED interrupt
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->TASKS_RXEN = 1;
}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;

        // CRCSTATUS == 1 means CRC OK
        if (NRF_RADIO->CRCSTATUS != 1) {
            rx_bad_crc++;
        } else if (pdu[1] == 2) {
            rx_acc_x = (int8_t)pdu[2];
            rx_acc_y = (int8_t)pdu[3];
            rx_count++;
        }
    }
}


// ----- application ---------------------------------------------------------

void run_tilt_rx(void) {
    led_on(1, 1);                  // immediate "I'm alive" before any I/O

    cutebot_i2c_init();
    update_motors(0, 0);           // stop motors (silently fails if no cutebot)
    ultrasonic_init();

    radio_rx_init();

    uint32_t last_count     = 0;
    uint32_t last_bad       = 0;
    int      rx_blink       = 0;
    int      evading        = 0;
    uint32_t evade_start_us = 0;

    while (1) {
        if (evading) {
            // we're actively backing away from the obstacle.
            // drive backwards until distance is safe again or timeout.
            int dist = ultrasonic_distance_cm();
            int timed_out = (now_us() - evade_start_us) > EVADE_TIMEOUT_US;

            if (dist >= SAFE_CM || timed_out) {
                update_motors(0, 0);
                set_headlights(0, 0, 0);
                beep(500, 150);                // "all clear" tone
                evading = 0;
                led_on(1, 1);
                print_str("rx: evade done (dist=");
                print_int(dist);
                print_str(")\n");
            }
            // small wait so we don't poll the ultrasonic too fast
            for (volatile int i = 0; i < 50000; i++);
            continue;
        }

        // -------- normal mode: act on incoming radio packets ---------------
        if (rx_count != last_count) {
            last_count = rx_count;

            int ax = (int)rx_acc_x;
            int ay = (int)rx_acc_y;

            int right = clamp((ay + ax) / 2, -100, 100);
            int left  = clamp((ay - ax) / 2, -100, 100);

            if (right > -DEADZONE && right < DEADZONE) right = 0;
            if (left  > -DEADZONE && left  < DEADZONE) left  = 0;

            int dist = ultrasonic_distance_cm();
            if (dist < OBSTACLE_CM) {
                // enter evade mode: lights on, alert beep, back away
                update_motors(EVADE_SPEED, EVADE_SPEED);
                set_headlights(0xFF, 0xFF, 0xFF);
                beep(1000, 200);
                led_on(5, 3);
                evading = 1;
                evade_start_us = now_us();
                print_str("rx: obstacle! backing up\n");
            } else {
                update_motors(left, right);
                rx_blink ^= 1;
                if (rx_blink) led_on(3, 3);
                else          led_on(5, 5);
            }

            print_str("rx: ax=");
            print_int(ax);
            print_str(" ay=");
            print_int(ay);
            print_str(" L=");
            print_int(left);
            print_str(" R=");
            print_int(right);
            print_str(" dist=");
            print_int(dist);
            putchar('\n');
        }
        if (rx_bad_crc != last_bad) {
            last_bad = rx_bad_crc;
            print_str("rx: bad CRC\n");
        }
        __WFE();
    }
}
