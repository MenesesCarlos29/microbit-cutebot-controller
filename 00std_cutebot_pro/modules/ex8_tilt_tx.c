// ex8_tilt_tx.c — TX application (manette).
//
// Reads the accelerometer, scales the raw values to [-100, +100] and
// broadcasts (acc_x, acc_y) over the radio in a loop.
//
// Hardware access goes through driver_manette.h.

#include "ex8_tilt_tx.h"
#include "driver_manette.h"
#include <stdint.h>
#include <stdio.h>


// ----- helpers (only used by this application) -----------------------------

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

static int clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Map raw accelerometer value (~ ±16000 for 1g) to a signed value in
// [-100, +100]. The divisor sign matches the chip orientation on this
// board (positive on tilt-right and tilt-forward).
static int8_t scale_accel(int16_t raw) {
    return (int8_t)clamp((int)raw / 112, -100, 100);
}


// ----- application ---------------------------------------------------------

void run_tilt_tx(void) {
    accel_init();

    uint8_t id = accel_who_am_i();
    print_str("WHO_AM_I = 0x");
    print_int(id);
    print_str(" (expected 51)\n");
    if (id != 0x33) {
        while (1);
    }

    radio_tx_init();

    while (1) {
        int16_t rx, ry, rz;
        accel_read_xyz(&rx, &ry, &rz);

        int8_t ax = scale_accel(rx);
        int8_t ay = scale_accel(ry);

        radio_tx_send(ax, ay);

        print_str("tx: acc_x=");
        print_int(ax);
        print_str("  acc_y=");
        print_int(ay);
        putchar('\n');

        for (volatile int i = 0; i < 200000; i++);
    }
}
