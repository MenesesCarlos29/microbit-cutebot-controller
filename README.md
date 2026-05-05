# micro:bit Cutebot Pro — Tilt-controlled robot in C

A micro:bit v2 used as a tilt controller wirelessly drives a Cutebot Pro
robot fitted with a second micro:bit. Equivalent of MakeCode's
"Example 8 — Remote Control with Accelerometer" lab, ported to bare-metal
C and built with Segger Embedded Studio.

The robot also has obstacle avoidance: if it detects something closer
than 30 cm in front, it stops, beeps, lights its headlights white and
automatically backs away until the path is clear.

---

## Architecture

Two firmwares share **one** Segger project (`00std_cutebot_pro`). Before
each flash, [main.c](00std_cutebot_pro/main.c) selects which one to run:

```c
run_tilt_tx();    // controller (manette)
//run_tilt_rx();  // robot (cutebot)
```

### Layout

```
00std_cutebot_pro/
├── main.c                      <- entry point: selects TX or RX
└── modules/
    ├── driver_manette.{c,h}    <- HW drivers used by the controller
    ├── driver_cutebot.{c,h}    <- HW drivers used by the robot
    ├── ex8_tilt_tx.{c,h}       <- TX application logic
    └── ex8_tilt_rx.{c,h}       <- RX application logic
```

The split is deliberate: drivers expose a small, hardware-aware API.
Application files only contain the *what to do* — never raw register
writes.

### What lives where

#### Controller (`driver_manette` + `ex8_tilt_tx`)

| Responsibility               | Function(s)                                      | File              |
|------------------------------|--------------------------------------------------|-------------------|
| LSM303AGR over TWI1          | `accel_init`, `accel_who_am_i`, `accel_read_xyz` | driver_manette.c  |
| NRF_RADIO transmit           | `radio_tx_init`, `radio_tx_send`                 | driver_manette.c  |
| Raw → ±100 scaling           | `scale_accel`                                    | ex8_tilt_tx.c     |
| Print helpers, `clamp`       | static in app file                               | ex8_tilt_tx.c     |
| Main loop                    | `run_tilt_tx`                                    | ex8_tilt_tx.c     |

#### Robot (`driver_cutebot` + `ex8_tilt_rx`)

| Responsibility               | Function(s)                                                  | File              |
|------------------------------|--------------------------------------------------------------|-------------------|
| Cutebot Pro motors over TWI0 | `cutebot_motors_init`, `update_motors`                       | driver_cutebot.c  |
| Cutebot Pro headlights       | `set_headlights`                                             | driver_cutebot.c  |
| NRF_RADIO receive + IRQ      | `radio_rx_init`, `RADIO_IRQHandler`, `rx_acc_x`, `rx_acc_y`  | driver_cutebot.c  |
| HC-SR04 ultrasonic + TIMER0  | `ultrasonic_init`, `now_us`, `ultrasonic_distance_cm`        | driver_cutebot.c  |
| Onboard speaker (P0.00)      | `beep`                                                       | driver_cutebot.c  |
| 5×5 LED matrix               | `leds_all_off`, `led_on`                                     | driver_cutebot.c  |
| State machine NORMAL/EVADING | `run_tilt_rx`                                                | ex8_tilt_rx.c     |

---

## End-to-end data flow

```
    [MANETTE]                                   [ROBOT]
    LSM303AGR                                   HC-SR04
       │                                           │
       ▼  I²C TWI1                                 ▼  GPIO + TIMER0
   accel_read_xyz                       ultrasonic_distance_cm
       │                                           │
       ▼                                  ┌────────┴────────┐
   scale_accel                            ▼                 ▼
       │                              < 30 cm?            normal
       ▼  radio_tx_send                   │                 │
   NRF_RADIO ─── radio (BLE LR) ───→ NRF_RADIO              │
                                          │ IRQ updates     │
                                          ▼ rx_acc_x/y      │
                                  enter EVADING:       (ay ± ax) / 2
                                  back up + beep            │
                                  + headlights              ▼
                                          │           update_motors
                                          ▼                 │
                                  loop until safe           │
                                          │                 │
                                          └─────────┬───────┘
                                                    ▼  I²C TWI0
                                       Cutebot Pro (motors + LEDs)
                                                    +
                                          Speaker on P0.00 for beeps
```

---

## Build & flash

1. Open `microbit.emProject` in Segger Embedded Studio.
2. Pick `00std_cutebot_pro` as active project.
3. Edit [main.c](00std_cutebot_pro/main.c) to select `run_tilt_tx()` or `run_tilt_rx()`.
4. Connect **one** micro:bit at a time over USB and press *Build & Run*.
5. Repeat for the second micro:bit with the other function selected.

Place both boards within radio range (a few meters), power them on, hold
the controller in your hand and tilt to drive the robot.

---

## Tuning knobs

All in [ex8_tilt_rx.c](00std_cutebot_pro/modules/ex8_tilt_rx.c):

| Macro              | Default     | Effect                                   |
|--------------------|-------------|------------------------------------------|
| `DEADZONE`         | `5`         | Below this, the robot ignores tilt input |
| `OBSTACLE_CM`      | `30`        | Triggers the evade behavior              |
| `SAFE_CM`          | `45`        | Exits evade once the path is this clear  |
| `EVADE_SPEED`      | `-40`       | Backward speed during evade              |
| `EVADE_TIMEOUT_US` | `3 000 000` | Safety net: stop after 3 s anyway        |

The accelerometer scaling lives in [ex8_tilt_tx.c](00std_cutebot_pro/modules/ex8_tilt_tx.c)
in `scale_accel`. The `112` divisor maps roughly 70 % of 1 g to ±100;
flipping its sign would invert both axes at once.

---

## RTT debug output

The project is configured with `LIBRARY_IO_TYPE="RTT"`, so `putchar` is
piped to SEGGER RTT. Open the *Debug Terminal* in SES while debugging
to watch lines like:

```
WHO_AM_I = 0x51 (expected 51)         <- TX side, sanity check
tx: acc_x=42  acc_y=-18                <- TX side, each transmission
rx: ax=42 ay=-18 L=12 R=30 dist=180   <- RX side, each received packet
rx: obstacle! backing up               <- RX side, evade triggered
rx: evade done (dist=48)               <- RX side, evade ended
```
