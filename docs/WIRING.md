# Wiring

> **Before you connect a battery:** the pin map below is implemented in
> `firmware/src/pins.rs` and the firmware will drive those pins. If your board is
> not an ESP32-WROOM-32 with GPIO16/GPIO17 available, stop and read
> [HARDWARE_ASSUMPTIONS.md](HARDWARE_ASSUMPTIONS.md). Do not power the drivers
> with the motors on the ground until you have done the bench test in
> [TESTING.md](TESTING.md).

## Power tree

```
 12 V pack (+) ──[ fuse 30 A ]──[ E-STOP, NC latching ]──┬── driver A  B+
                                                        └── driver B  B+
 12 V pack (-) ──────────────────────────────────────────┬── driver A  B-
                                                         └── driver B  B-
                                                         └── buck 5 V ── ESP32 5V/VIN
                                                                       └── driver A/B VCC (only if the module has no regulator)
 ESP32 GND ──────────────────────────────────────────────── common with pack (-)
```

* The E-stop breaks the **motor** supply, not the ESP32 supply. That is
  deliberate: the robot goes inert, the Wi-Fi link stays up so you can see the
  fault, and the switch is genuinely independent of the firmware.
* One fuse for the whole motor branch. Size it above the measured stall current
  of one side and below the pack's continuous rating.
* Both drivers share the same `B-` and the ESP32 ground: one star point at the
  battery negative terminal, not a chain through the modules.
* Most IBT-2 boards have an onboard 5 V regulator and `VCC` can stay
  unconnected; check yours with a meter before feeding 5 V into it. `VCC` is the
  **logic** supply, not the motor supply.

## ESP32 pin map

Implemented in `firmware/src/pins.rs`. Nothing else in the firmware chooses pins.

| Signal | GPIO | Direction | Notes |
|--------|------|-----------|-------|
| Driver A `RPWM` (left forward) | 16 | PWM, LEDC ch0 | 10 kHz, 12 bit |
| Driver A `LPWM` (left reverse) | 17 | PWM, LEDC ch1 | |
| Driver A `R_EN` | 18 | output | low at boot = driver disabled |
| Driver A `L_EN` | 19 | output | low at boot |
| Driver B `RPWM` (right forward) | 21 | PWM, LEDC ch2 | |
| Driver B `LPWM` (right reverse) | 22 | PWM, LEDC ch3 | |
| Driver B `R_EN` | 23 | output | low at boot |
| Driver B `L_EN` | 25 | output | low at boot |
| Battery sense | 34 | ADC1 channel 6 | input only, 11 dB attenuation |
| Status LED | 2 | output | on-board LED, active high |

Pins deliberately avoided:

* **6–11** — SPI flash, the board will not boot.
* **0, 2 (button), 5, 12, 15** — boot strapping pins. GPIO2 is used for the LED
  only, and only after boot.
* **1, 3** — USB serial, useful to keep for logs.
* **16, 17 on a WROVER module** — used by PSRAM. Use a WROOM-32.

`GPIO34` has no output driver and no internal pull-up: it is input only, which is
exactly what an ADC tap needs.

## Driver A / B to ESP32

Each IBT-2 / BTS7960 module is one H-bridge driving one side (two motors in
parallel). The module header is labelled `RPWM LPWM R_EN L_EN VCC GND`.

| Module | Module pin | ESP32 pin |
|--------|-----------|-----------|
| A (left) | RPWM | GPIO16 |
| A | LPWM | GPIO17 |
| A | R_EN | GPIO18 |
| A | L_EN | GPIO19 |
| A | GND | common ground |
| A | VCC | 5 V only if the board has no regulator |
| B (right) | RPWM | GPIO21 |
| B | LPWM | GPIO22 |
| B | R_EN | GPIO23 |
| B | L_EN | GPIO25 |
| B | GND | common ground |
| B | VCC | as above |

The 3.3 V ESP32 outputs are enough for the BTS7960 logic inputs (high threshold
is well below 3.3 V), but this is one of the assumptions in
[HARDWARE_ASSUMPTIONS.md](HARDWARE_ASSUMPTIONS.md): confirm the first time you
enable a driver that the output actually switches.

### Motors

```
Driver A  M+ ──┬── left front motor (+)
               └── left rear  motor (+)
Driver A  M- ──┬── left front motor (-)
               └── left rear  motor (-)
Driver B  M+ ── right front (+)   Driver B  M- ── right rear (-)  [same pairing]
```

Two motors in parallel on one module is electrically fine as long as the **sum**
of their currents stays inside the module rating, and mechanically fine only if
both wheels on a side have the same diameter, the same gearing and can spin
freely. Check by pushing the robot: both wheels on one side must start turning
together with the same effort.

If a side runs backwards, swap `M+` and `M-` on that module (or fix it in
software by swapping the two PWM channels in `motors.rs`; the hardware fix is
preferred, because then the wiring diagram and the behaviour always agree).

## Battery divider

```
 pack (+) ──[ 100 kΩ ]──┬── GPIO34 (ADC1_CH6)
                        │
                   [ 100 nF ]
                        │
 pack (-) ──[ 27 kΩ ]───┴── pack (-)
```

* 12.6 V (3S full) → 12.6 × 27 / 127 = **2.68 V** at the pin, inside the 11 dB
  range (~3.1 V full scale).
* 100 nF from the tap to ground: the ESP32 ADC samples a small capacitor and a
  high-impedance divider alone gives noisy readings.
* The divider draws about 100 µA — negligible.
* `core/src/config.rs` defaults encode this as `divider_num = 127`,
  `divider_den = 27`. If you use different resistors, update those two numbers
  (see [CALIBRATION.md](CALIBRATION.md)).

Never connect the pack directly to GPIO34.

## Physical emergency stop

Requirements this satisfies:

* **normally closed, latching** mushroom switch rated for the pack voltage and
  the motor current,
* in series with the motor supply (B+ of both drivers),
* reachable from the driver's position without letting go of the controller,
* the robot must be mechanically unable to move when it is pressed.

Test it **before** every run: with the robot armed and the wheels in the air,
press the E-stop and confirm the wheels stop and stay stopped, then release the
switch and confirm the robot does **not** resume without a new ARM.

The in-app E-STOP is a convenience, not the safety device. It relies on the
firmware, the Wi-Fi link and the phone.

## EMC and mechanical notes

* Keep the motor wires away from GPIO34 and from the ESP32 antenna. Motor wires
  are the noisiest thing in the robot.
* Twist each motor pair; keep the two sides' wires physically separated.
* Mount the drivers on a metal plate or with a heatsink: the BTS7960 has
  over-temperature shutdown, but a hot module means thermal shutdown in the
  middle of a run.
* Strap the battery so it cannot slide into a wheel. Li-ion packs dislike being
  pierced far more than they dislike being discharged.
* Route the E-stop wire so it cannot be pulled out by a wheel.

## Pre-power checklist

1. Continuity: pack (+) to each driver `B+` through the E-stop, no short to `B-`.
2. No continuity between `B+` and `B-` on either module with the switch open.
3. ESP32 powered alone (USB): boots, prints the boot log, LED reacts, motors do
   not move (the enable pins are low until firmware drives them).
4. Measure 5 V (or 3.3 V, per module) at each module's `VCC` before connecting
   the motors.
5. Measure the divider output with the pack connected: expect about 2.5–2.7 V.
6. Only then connect the motors and do the bench test in
   [TESTING.md](TESTING.md).
