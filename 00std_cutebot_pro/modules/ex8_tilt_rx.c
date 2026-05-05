// ex8_tilt_rx.c — RX application (cutebot).
//
// Receives (acc_x, acc_y) from the controller, computes left/right wheel
// speeds with the differential drive formula, and drives the cutebot.
// Stops + backs away from obstacles closer than OBSTACLE_CM, using a
// simple state machine (NORMAL / EVADING).
//
// Hardware access goes through driver_cutebot.h.

#include "ex8_tilt_rx.h"
#include "driver_cutebot.h"
#include <nrf52833.h>          // for __WFE()
#include <stdint.h>
#include <stdio.h>


// ----- tuning constants ----------------------------------------------------

#define DEADZONE          5         // |speed| below this is treated as 0
#define OBSTACLE_CM       30        // enter evade if distance below this
#define SAFE_CM           45        // exit evade once distance is above this
#define EVADE_SPEED      -40        // backward speed during evade
#define EVADE_TIMEOUT_US  3000000   // safety: stop evade after 3 s no matter what


// ----- helpers (only used by this application) -----------------------------

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


// ----- application ---------------------------------------------------------

void run_tilt_rx(void) {
    led_on(1, 1);                  // immediate "I'm alive" before any I/O

    cutebot_motors_init();
    update_motors(0, 0);           // make sure motors are stopped at boot
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
            int dist = ultrasonic_distance_cm();
            int timed_out = (now_us() - evade_start_us) > EVADE_TIMEOUT_US;

            if (dist >= SAFE_CM || timed_out) {
                update_motors(0, 0);
                set_headlights(0, 0, 0);
                beep(500, 150);
                evading = 0;
                led_on(1, 1);
                print_str("rx: evade done (dist=");
                print_int(dist);
                print_str(")\n");
            }
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
                // enter evade mode
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
