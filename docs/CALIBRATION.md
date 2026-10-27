# Calibration

All tunables live in **one place**: `core/src/config.rs`, in the `Default` impls.
The firmware calls `Config::default()` and validates it at boot; a value outside
its allowed range aborts the boot with the motors disabled. Nothing is stored in
flash, so changing a value means editing, rebuilding and reflashing.

After any change:

```powershell
cargo test -p robo-core
cd firmware
cargo build --release --target xtensa-esp32-none-elf
```

The tests cover the safety envelope. If a change breaks a test, the change is
wrong, not the test.

## 1. What the defaults already encode

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `forward` | 800 | both sides forward |
| `reverse` | 600 | both sides backward |
| `turn_inner` | 350 | inner wheel in a forward turn |
| `turn_outer` | 800 | outer wheel in a forward turn |
| `reverse_turn_inner` / `reverse_turn_outer` | 350 / 600 | reverse turn |
| `accel_per_s` | 2000 | units per second (0 → 1000 in 0.5 s) |
| `decel_per_s` | 4000 | 0 → 1000 in 0.25 s |
| `direction_change_coast_ms` | 60 | coast before reversing |
| `min_duty_permille` | 200 | PWM floor (stiction) |
| `max_duty_permille` | 1000 | PWM ceiling |
| `slow/normal/fast_percent` | 50 / 100 / 100 | speed modes |
| `command_timeout_ms` | 250 | moving robot with no new command → fault |
| `link_timeout_ms` | 2000 | silent socket → link lost |
| `control_period_ms` | 10 | 100 Hz loop |
| `cell_count` | 3 | pack is 3S |
| `warn_mv_per_cell` | 3400 | 10.2 V pack |
| `critical_mv_per_cell` | 3300 | 9.9 V pack, 3 samples |
| `divider_num/den` | 127 / 27 | 100 k / 27 k divider |
| `ema_alpha_percent` | 20 | voltage filter |
| `critical_samples` | 3 | debounce (600 ms at 5 Hz) |

`critical_mv()` is `cell_count × critical_mv_per_cell`; the same for `warn_mv()`.

## 2. Battery calibration

The chain is:

```
ADC counts --(ADC_FULL_SCALE_MV)--> millivolts at the pin --(divider)--> pack mV
```

`ADC_FULL_SCALE_MV` is `3100` in `firmware/src/battery.rs` (nominal 11 dB range).
The divider is applied by `adc_to_pack_mv()`:

```
pack_mv = adc_mv × divider_num / divider_den + calibration_offset_mv
```

### Two-point procedure

You need a multimeter. Do this with the robot on blocks, motors disconnected is
fine (the ADC only needs the pack).

1. Flash the firmware and open the serial monitor. The status frame reports pack
   millivolts; for calibration it is easier to read the **measured** voltage with
   the multimeter and compare it with the value the app shows in the header
   (`--.- V`).
2. Discharge or charge the pack to two clearly different voltages, at least 1.5 V
   apart (for example 10.0 V and 12.4 V). Let it rest a minute; Li-ion rebounds.
3. At point 1, record `V1` (multimeter) and `A1` (voltage the app shows).
4. At point 2, record `V2` and `A2`.
5. Compute:

```
scale  = (V2 - V1) / (A2 - A1)          # should be near 4.70 with 127/27
offset = V1 - scale × A1                # millivolts to add
```

6. Encode `scale` as a fraction. With the default hardware the fraction is
   `127/27 = 4.7037`; if your resistors differ, use `num/den` of your divider.
   Keep the divider values as integers and put the residual error in
   `calibration_offset_mv` (a signed millivolt count).
7. Rebuild, and repeat the two measurements. They should now agree within about
   50 mV.

If **both** readings are off by the same percentage and the divider is correct,
the error is in `ADC_FULL_SCALE_MV` (the ADC reference), not in the divider. In
that case scale `ADC_FULL_SCALE_MV` by `measured / reported` and leave the
divider alone.

### Thresholds

* At 9.9 V the supervisor latches a `LowBattery` fault — but **only while armed**.
  While disarmed it just reports `BATTERY_OK = 0`.
* Three consecutive critical samples are required, so a single sag under
  acceleration does not stop the robot.
* Set `cell_count` to match the pack you actually use. A 2S pack with
  `cell_count = 3` will read as permanently critical.

## 3. PWM / duty calibration

Command values are normalised (`-1000 … +1000`), not volts. The firmware maps
the magnitude onto the duty window:

```
duty_permille = min_duty + (max_duty - min_duty) × |command| / 1000
```

So the default `forward = 800` produces `200 + 800 × 800/1000 = 840 ‰` duty, and
a command of `0` disables the driver instead of leaving it enabled at zero duty.

Procedure, wheels **in the air**:

1. Start with `min_duty_permille = 200`. Arm, hold forward, and lower the value
   until the wheels just barely fail to start, then raise it by 20.
2. Set `max_duty_permille` so the robot reaches competition speed without
   wheelspin on the actual surface. Do not exceed 1000.
3. Log the pack current at full duty on the ground. Two motors per side share one
   module; keep the measured current well inside the module's continuous rating
   (see [HARDWARE_ASSUMPTIONS.md](HARDWARE_ASSUMPTIONS.md)).
4. If one side is noticeably faster, lower that side's `max_duty_permille`… there
   is no per-side window in the current configuration, so correct mechanical
   differences (wheel diameter, gearbox drag) mechanically, or add a per-side
   scale before you change anything else.

## 4. Steering calibration

The four buttons produce the table in `core/src/steering.rs`. Verify on blocks:

| Input | Expected command (left, right) |
|-------|-------------------------------|
| Forward | `+800, +800` |
| Backward | `-600, -600` |
| Left | `+350, +800` |
| Right | `+800, +350` |
| Forward + Backward | `0, 0` (stop wins) |
| Left + Right | `0, 0` |
| Stop + anything | `0, 0` |

If the robot turns **away** from the requested direction, the motor polarity of
one side is inverted — fix it at the driver terminals (`M+`/`M-`), not in
software.

Turning radius is set by the ratio `turn_inner / turn_outer`. Lower `turn_inner`
for a tighter turn; a value near 0 spins on the spot but loads the gearboxes.

Speed modes scale the whole command: `slow = 50 %` gives `+400, +400` for
forward, `normal = 100 %` gives the table above. The app defaults to `NORMAL`.

## 5. Ramp calibration

* `accel_per_s = 2000` means a full-scale command ramps in 0.5 s. Lower it if the
  wheels spin at launch; raise it if the robot feels sluggish.
* `decel_per_s = 4000` stops in 0.25 s. Lower it if the robot nose-dives.
* `direction_change_coast_ms = 60` is the dead time between a forward and a
  reverse command. Too short and the current spike trips the driver; too long
  and the robot feels unresponsive in a slalom.

Tune on the surface you will compete on, with a fresh pack. A pack at 10 V feels
noticeably weaker than a pack at 12.4 V.

## 6. Record sheet

Keep a copy per robot, per event.

| Date | Pack | Divider num/den | Offset | ADC full scale | min/max duty | Accel/decel | Notes |
|------|------|-----------------|--------|----------------|--------------|-------------|-------|
|      |      |                 |        |                |              |             |       |

Every change to the numbers above belongs in this table, so that "it worked last
week" can be reproduced.
