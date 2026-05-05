// ex8_tilt_rx.c - RX application (cutebot).
//
// Receives (acc_x, acc_y, button) from the controller, computes left/right
// wheel speeds with the differential drive formula, and drives the cutebot.
// Also runs non-blocking behavior macros and obstacle evasion.
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

#define MODE_NORMAL       0
#define MODE_EVADE        1
#define MODE_STUNT_A      2
#define MODE_STUNT_B      3


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

    uint32_t last_count      = 0;
    uint32_t last_bad        = 0;
    uint32_t last_rx_count   = rx_count;
    uint32_t evade_start_us  = 0;
    uint32_t stunt_start_us  = 0;
    int      mode            = MODE_NORMAL;
    int      stunt_phase     = -1;
    int      rx_blink        = 0;

    while (1) {
        if (rx_count != last_rx_count) {
            last_rx_count = rx_count;

            if (rx_btn == 1 && mode == MODE_NORMAL) {
                mode = MODE_STUNT_A;
                stunt_start_us = now_us();
                stunt_phase = -1;
                last_count = rx_count;      // consume the button packet
            }
            else if (rx_btn == 2 && mode == MODE_NORMAL) {
                mode = MODE_STUNT_B;
                stunt_start_us = now_us();
                stunt_phase = -1;
                last_count = rx_count;      // consume the button packet
            }
        }

        uint32_t current_time = now_us();

        if (mode == MODE_STUNT_A) {
            uint32_t elapsed = current_time - stunt_start_us;
            int phase = (elapsed < 500000)  ? 0 :
                        (elapsed < 1000000) ? 1 :
                        (elapsed < 1500000) ? 2 : 3;

            if (phase != stunt_phase) {
                stunt_phase = phase;

                if (phase == 0) {
                    update_motors(80, -80);
                    set_headlights(255, 0, 0);
                }
                else if (phase == 1) {
                    update_motors(-80, 80);
                    set_headlights(0, 0, 255);
                }
                else if (phase == 2) {
                    update_motors(80, 80);
                    set_headlights(0, 255, 0);
                    beep(800, 50);
                }
                else {
                    update_motors(0, 0);
                    set_headlights(0, 0, 0);
                    mode = MODE_NORMAL;
                }
            }
        }
        else if (mode == MODE_STUNT_B) {
            uint32_t elapsed = current_time - stunt_start_us;
            int phase = (elapsed < 400000)  ? 0 :
                        (elapsed < 800000)  ? 1 :
                        (elapsed < 1200000) ? 2 : 3;

            if (phase != stunt_phase) {
                stunt_phase = phase;

                if (phase == 0) {
                    update_motors(-80, -20);
                    set_headlights(255, 255, 0);
                }
                else if (phase == 1) {
                    update_motors(-20, -80);
                    set_headlights(0, 255, 255);
                }
                else if (phase == 2) {
                    update_motors(-80, -20);
                    set_headlights(255, 0, 255);
                }
                else {
                    update_motors(0, 0);
                    set_headlights(0, 0, 0);
                    mode = MODE_NORMAL;
                }
            }
        }
        else if (mode == MODE_NORMAL) {
            if (rx_count != last_count) {
                last_count = rx_count;

                if (rx_btn == 0) {
                    int ax = (int)rx_acc_x;
                    int ay = (int)rx_acc_y;

                    int left  = clamp((ay + ax) / 2, -100, 100);
                    int right = clamp((ay - ax) / 2, -100, 100);

                    if (right > -DEADZONE && right < DEADZONE) right = 0;
                    if (left  > -DEADZONE && left  < DEADZONE) left  = 0;

                    int dist = ultrasonic_distance_cm();
                    if (dist < OBSTACLE_CM) {
                        update_motors(EVADE_SPEED, EVADE_SPEED);
                        set_headlights(0xFF, 0xFF, 0xFF);
                        beep(1000, 200);
                        led_on(5, 3);
                        evade_start_us = now_us();
                        mode = MODE_EVADE;
                        print_str("rx: obstacle! backing up\n");
                    }
                    else {
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
            }

            if (rx_bad_crc != last_bad) {
                last_bad = rx_bad_crc;
                print_str("rx: bad CRC\n");
            }

            __WFE();
        }
        else if (mode == MODE_EVADE) {
            int dist = ultrasonic_distance_cm();
            int timed_out = (now_us() - evade_start_us) > EVADE_TIMEOUT_US;

            if (dist >= SAFE_CM || timed_out) {
                update_motors(0, 0);
                set_headlights(0, 0, 0);
                beep(500, 150);
                led_on(1, 1);
                mode = MODE_NORMAL;
                print_str("rx: evade done (dist=");
                print_int(dist);
                print_str(")\n");
            }

            for (volatile int i = 0; i < 50000; i++);
        }
    }
}
