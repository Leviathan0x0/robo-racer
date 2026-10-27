# Hardware assumptions

This file exists because the firmware had to pick numbers before the hardware was
in front of it. Every row below is either **confirmed by you** or **assumed**.
Nothing here has been measured on a real robot in this repository: no board was
flashed, no motor was spun.

Read the "if wrong" column before you connect a battery. A wrong assumption is
usually cheap to fix in `core/src/config.rs` or `firmware/src/pins.rs`, but two of
them (pack voltage and motor current) can destroy hardware.

## 1. Assumptions that need your confirmation

| # | Item | Assumed here | Why it matters | How to confirm | If wrong |
|---|------|--------------|----------------|----------------|----------|
| 1 | ESP32 board | **ESP32-WROOM-32**, 4 MB flash, no PSRAM, GPIO16/17 free | the pin map uses 16/17; `pins.rs` refuses to build for other chips | read the module marking, check `idf.py`/`espflash` chip detect | WROVER: 16/17 are PSRAM → pick other pins and move the LEDC channels. S3/C3: rewrite `pins.rs`, LEDC numbering differs |
| 2 | Motors | 4 × **600 RPM Johnson**, 12 V nominal, gearbox ratio unknown | sets top speed and current | motor label / supplier data | different RPM changes the speed estimate below, not the code |
| 3 | Motor current | **not measured**. Assumed 0.3–0.6 A free-running, 2–4 A at rated load, 8–15 A stall per motor | two motors share one BTS7960 | clamp meter in series with one motor, wheels loaded | if stall current per side exceeds ~25 A, the module shuts down or dies. Add a current limit or use one module per motor |
| 4 | BTS7960 module rating | ~25 A continuous with heatsink, 43 A peak (datasheet claim) | same as above | datasheet of the exact module | clone modules are often 10–15 A boards; derate accordingly |
| 5 | `B+` voltage | **12 V maximum**, i.e. a 3S Li-ion pack at 12.6 V is *above* the stated maximum | over-voltage on motors and drivers | the competition rules and the motor label | see the 12.6 V discussion below — this is the one real conflict in the brief |
| 6 | Pack | 3S Li-ion (11.1 V nominal, 12.6 V full, 9.0 V empty), capacity unknown | battery thresholds and divider | measure the pack, read the label | change `cell_count`, `warn_mv_per_cell`, `critical_mv_per_cell` |
| 7 | Battery divider | 100 kΩ / 27 kΩ → 127/27, 100 nF at the tap | pack voltage reading | measure both resistors | set `divider_num`/`divider_den`; recalibrate |
| 8 | Logic levels | 3.3 V from the ESP32 is enough for `R_EN`/`L_EN`/`RPWM`/`LPWM` | a driver that never enables = a robot that never moves | measure the module input threshold, or just test | add a 3.3→5 V level shifter (74HCT245) on the four control inputs per module |
| 9 | E-stop | latching NC mushroom switch, ≥ 20 A DC, in the motor `B+` line | the only firmware-independent stop | press it under load once | undersized contacts weld shut; replace with a DC-rated switch or a contactor |
| 10 | Mass / size | ≤ 5 kg, 30 × 30 × 40 cm | rules compliance and acceleration | kitchen scale, tape measure | reduce mass or gearing; the tuning still works |
| 11 | Wheels | 95 mm × 20 mm, no encoders | open-loop control only | measure | no change; there is no closed loop to tune |
| 12 | Status LED | GPIO2, active high (on-board LED on most dev boards) | operator feedback only | look at the board | change `pins.rs` / `init_led` |
| 13 | Toolchain | Espressif `esp` Rust toolchain, link with `rust-lld`, **no GCC Xtensa toolchain** | reproducible build | `cd firmware; cargo build --release --target xtensa-esp32-none-elf` | if rust-lld ever fails, install the GCC toolchain and switch the linker flags in `firmware/.cargo/config.toml` |
| 14 | Flashing | `espflash` (or `esptool.py`) over USB | getting the binary onto the board | `espflash flash --monitor` | any ESP32 flasher works; the ELF is the input |

## 2. The 12.6 V question — decide before you build the pack

The brief says **12 V DC maximum** and also that the pack is Li-ion. A 3S Li-ion
pack is 12.6 V at full charge, which is 5 % over the stated maximum, and every
component sees it at the start of a run, when the motors are also strongest.

Options, in order of preference for a competition:

1. **3S Li-ion, and treat 12.6 V as acceptable.** Most 12 V Johnson motors and
   BTS7960 modules tolerate 12.6 V; the risk is motor heating over a long run.
   Mitigate by capping `max_duty_permille` at about 900 and checking motor
   temperature after the first two-minute run. Defaults in this repo assume this.
2. **3S LiFePO4.** 9.9 V nominal, 10.8 V full, 9.0 V empty. Safest for the
   drivers, ~15 % slower, and the battery thresholds must be changed
   (`cell_count = 3`, `warn = 3200`, `critical = 3000` per cell is a sane start).
3. **2S Li-ion.** 8.4 V full, noticeably slower, no over-voltage argument.
   Change `cell_count = 2` and the thresholds.
4. **3S with a DC-DC buck to 12.0 V.** Clean, but a 30 A buck is heavy and
   expensive; not worth it at this scale.

Whatever you pick, the firmware reads the **total pack voltage**; only
`cell_count` and the per-cell thresholds need to change.

## 3. Derived numbers (not measurements)

* Wheel circumference: π × 0.095 m = **0.298 m**.
* No-load speed at 600 RPM: 10 rev/s × 0.298 m = **2.99 m/s** (10.7 km/h).
* On the ground expect roughly 2–2.5 m/s; the default `max_duty_permille = 1000`
  allows all of it, so use `SLOW` indoors.
* A 5 kg robot on four wheels needs ~0.12 N·m per motor to hold 1 m/s², before
  gearbox losses. Whether the motors deliver it is **unknown** until measured.

## 4. Verified by running code (on the host, not on hardware)

| Verified | Evidence |
|----------|----------|
| `core` logic, 82 tests | `cargo test -p robo-core` |
| Firmware compiles for `xtensa-esp32-none-elf` | `cargo build --release --target xtensa-esp32-none-elf`, 0 warnings |
| Image fits a 4 MB module | ELF sections sum to ≈ 1.27 MB (1.21 MiB) |
| Pin choices are consistent with the ESP32 boot rules | by inspection of `pins.rs` and the ESP32 datasheet |
| LEDC configuration API usage | matched against `esp-hal` 1.2.2 sources |

## 5. Not verified at all

* Boot on a real ESP32, heap sizing, `esp_rtos` start order.
* Wi-Fi AP, DHCP lease, HTTP, WebSocket handshake with a phone browser.
* PWM frequency and duty at the pins; motor direction; braking vs coasting.
* ADC linearity and the actual pack reading.
* Thermal behaviour of the drivers, current draw, fuse rating.
* Physical E-stop, wiring, connector ratings.
* Timing of the 100 Hz loop with Wi-Fi traffic.

The bench test in [TESTING.md](TESTING.md) is the shortest path to closing these.

## 6. Before the first power-up

1. Decide the pack chemistry (section 2) and set `cell_count` and the thresholds.
2. Measure your divider and set `divider_num` / `divider_den`.
3. Confirm the ESP32 variant and that GPIO16/17 are free.
4. Confirm the driver modules' `VCC` requirement (regulator or not).
5. Fit the fuse and the E-stop.
6. Do the pre-power checklist in [WIRING.md](WIRING.md).
7. Do the bench test in [TESTING.md](TESTING.md), then calibrate with
   [CALIBRATION.md](CALIBRATION.md).

Any number in this repository that came from an assumption is marked
**PROVISIONAL** in the source. Search for it:

```powershell
Select-String -Path core\src\*.rs,firmware\src\*.rs -Pattern PROVISIONAL
```
