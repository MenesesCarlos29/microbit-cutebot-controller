# micro:bit Cutebot Pro - Tilt-controlled robot in C

A micro:bit v2 used as a tilt controller wirelessly drives a Cutebot Pro
robot fitted with a second micro:bit. The project is the bare-metal C
version of the remote-control lab built in Segger Embedded Studio.

The final version combines wireless driving, obstacle safety, light and
sound feedback, and button-triggered stunt modes. The project name and
runtime configuration stay the same: the code is still launched from the
same Segger project and selected through `00std_cutebot_pro/main.c`.

---

## Implemented features

- Tilt control: the controller reads the LSM303AGR accelerometer, scales
  the values to `[-100, +100]`, and sends them by radio.
- Wireless differential drive: the robot converts `(acc_x, acc_y)` into
  left and right wheel speeds with a differential-drive formula.
- Button actions: button A triggers `MODE_STUNT_A` and button B triggers
  `MODE_STUNT_B`, each with its own short motor and light sequence.
- Obstacle avoidance: if the ultrasonic sensor detects an obstacle closer
  than `30 cm`, the robot enters `MODE_EVADE`, reverses, beeps, and leaves
  evade only when the path is clear or the timeout expires.
- Light feedback: the headlights blink as turn signals while steering and
  turn white during obstacle evasion.
- Sound feedback: the onboard speaker emits a periodic reverse beep and
  short confirmation beeps during behaviors.
- Status feedback: the 5x5 LED matrix shows basic activity states and RTT
  prints help monitor radio and safety events while debugging.
- Modular organization: low-level hardware access is separated from the
  application logic through `driver_manette` and `driver_cutebot`.

---

## Architecture

Two firmwares share **one** Segger project (`00std_cutebot_pro`). Before
each flash, [main.c](00std_cutebot_pro/main.c) selects which one to run.

The repository currently leaves the robot side enabled by default:

```c
//run_tilt_tx();   // controller (manette)
run_tilt_rx();     // robot (cutebot)
```

Before flashing the controller micro:bit, switch the uncommented call to
`run_tilt_tx()`. Before flashing the robot micro:bit, switch it back to
`run_tilt_rx()`.

### Layout

```text
00std_cutebot_pro/
|-- main.c                      <- entry point: selects TX or RX
`-- modules/
    |-- driver_manette.{c,h}    <- HW drivers used by the controller
    |-- driver_cutebot.{c,h}    <- HW drivers used by the robot
    |-- ex8_tilt_tx.{c,h}       <- TX application logic
    `-- ex8_tilt_rx.{c,h}       <- RX application logic
```

The split is deliberate: the driver files contain raw hardware access and
the application files contain the behavior logic.

### What lives where

#### Controller (`driver_manette` + `ex8_tilt_tx`)

| Responsibility             | Function(s)                                      | File             |
|---------------------------|--------------------------------------------------|------------------|
| LSM303AGR over TWI1       | `accel_init`, `accel_who_am_i`, `accel_read_xyz` | `driver_manette.c` |
| NRF_RADIO transmit        | `radio_tx_init`, `radio_tx_send`                 | `driver_manette.c` |
| Button A / B handling     | local button read and packet button field        | `ex8_tilt_tx.c`  |
| Raw -> `[-100, +100]` map | `scale_accel`                                    | `ex8_tilt_tx.c`  |
| Print helpers and clamp   | static helpers                                   | `ex8_tilt_tx.c`  |
| Main TX loop              | `run_tilt_tx`                                    | `ex8_tilt_tx.c`  |

#### Robot (`driver_cutebot` + `ex8_tilt_rx`)

| Responsibility                | Function(s)                                                           | File              |
|------------------------------|-----------------------------------------------------------------------|-------------------|
| Cutebot Pro motors over TWI0 | `cutebot_motors_init`, `update_motors`                                | `driver_cutebot.c` |
| Headlights                   | `set_headlights`, `set_left_headlight`, `set_right_headlight`         | `driver_cutebot.c` |
| NRF_RADIO receive + IRQ      | `radio_rx_init`, `RADIO_IRQHandler`, `rx_acc_x`, `rx_acc_y`, `rx_btn` | `driver_cutebot.c` |
| HC-SR04 ultrasonic + TIMER0  | `ultrasonic_init`, `now_us`, `ultrasonic_distance_cm`                 | `driver_cutebot.c` |
| Onboard speaker              | `beep`                                                                | `driver_cutebot.c` |
| 5x5 LED matrix               | `leds_all_off`, `led_on`                                              | `driver_cutebot.c` |
| Reverse beep                 | `update_reverse_beep`                                                 | `ex8_tilt_rx.c`   |
| Turn signals                 | `update_turn_signals`                                                 | `ex8_tilt_rx.c`   |
| Robot state machine          | `run_tilt_rx` with `MODE_NORMAL`, `MODE_EVADE`, `MODE_STUNT_A`, `MODE_STUNT_B` | `ex8_tilt_rx.c` |

---

## End-to-end data flow

```text
[CONTROLLER]                                   [ROBOT]
LSM303AGR + buttons A/B                        HC-SR04
      |                                           |
      v                                           v
accel_read_xyz + button read                  ultrasonic_distance_cm
      |                                           |
      v                                           |
scale_accel                                       |
      |                                           |
      v                                           |
radio_tx_send  ------------------------------>  RADIO_IRQHandler
                                                  |
                                                  +--> button A/B -> stunt mode
                                                  |
                                                  `--> normal packet
                                                        |
                                                        v
                                                   (ay +/- ax) / 2
                                                        |
                                                   obstacle check
                                                    /         \
                                                   /           \
                                              MODE_EVADE    normal drive
                                              reverse       update_motors
                                              white lights  turn signals
                                              beep          reverse beep
```

---

## Build & flash

1. Open `microbit.emProject` in Segger Embedded Studio.
2. Pick `00std_cutebot_pro` as the active project.
3. Edit [main.c](00std_cutebot_pro/main.c)
   and select `run_tilt_tx()` or `run_tilt_rx()`.
4. Connect **one** micro:bit at a time over USB and press *Build & Run*.
5. Repeat for the second micro:bit with the other function selected.

No project files need to be renamed and no runtime configuration needs to
change beyond selecting the function in `main.c`.

Place both boards within radio range, power them on, hold the controller
in your hand, and tilt to drive the robot.

---

## Tuning knobs

Most of the runtime behavior is tuned in
[ex8_tilt_rx.c](00std_cutebot_pro/modules/ex8_tilt_rx.c).

| Macro                     | Default     | Effect |
|--------------------------|-------------|--------|
| `DEADZONE`               | `5`         | Ignores very small tilt commands |
| `OBSTACLE_CM`            | `30`        | Distance that triggers evade mode |
| `SAFE_CM`                | `45`        | Distance required to exit evade |
| `EVADE_SPEED`            | `-40`       | Reverse speed during evade |
| `EVADE_TIMEOUT_US`       | `3000000`   | Stops evade after 3 seconds if needed |
| `TURN_THRESHOLD`         | `50`        | Minimum steering command to blink turn signals |
| `BLINK_PERIOD_US`        | `250000`    | Turn-signal blink period |
| `REVERSE_BEEP_PERIOD_US` | `500000`    | Delay between reverse beeps |
| `REVERSE_BEEP_FREQ_HZ`   | `800`       | Reverse beep frequency |
| `REVERSE_BEEP_DURATION`  | `80`        | Reverse beep duration in ms |

The accelerometer scaling lives in
[ex8_tilt_tx.c](00std_cutebot_pro/modules/ex8_tilt_tx.c) inside
`scale_accel()`. The divisor `112` maps the raw sensor values to roughly
`[-100, +100]`.

---

## RTT debug output

The project is configured with `LIBRARY_IO_TYPE="RTT"`, so `putchar` is
sent to SEGGER RTT. Open the *Debug Terminal* in SES while debugging to
watch messages like:

```text
WHO_AM_I = 0x51 (expected 51)
tx: acc_x=42  acc_y=-18
rx: obstacle! backing up
rx: evade done (dist=48)
rx: bad CRC
```

Note: the accelerometer ID check compares against decimal `51`, which is
the same value as sensor ID `0x33`.
