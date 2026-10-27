# robo-racer

Firmware and control app for a four-wheel competition robot: an ESP32 driving two
BTS7960 modules (left pair / right pair of 600 RPM Johnson motors), controlled
from a phone browser over the robot's own Wi-Fi access point.

The firmware you flash is the ESP-IDF project in `idf/`. The phone still sends
button bits, and the firmware is still the only place that turns those bits into
motor commands. `core/` is the Rust specification those rules were ported from.
`firmware/` is the previous `esp-hal` build; it is not the one to flash.

| Piece | Runs on | What lives there |
|-------|---------|------------------|
| `idf/` | ESP32 (ESP-IDF) | Wi-Fi AP, DHCP, HTTP + WebSocket, LEDC, ADC, control loop pinned to the app core |
| `core/` | host | Rust specification of the safety, steering, ramp, battery and protocol rules |
| `web/` | phone browser | Control app, embedded into the IDF image at build time |
| `firmware/` | ESP32 (`esp-hal`) | Previous Rust HAL firmware, kept for reference |

## Status — read this first

What has actually been verified on this machine:

| Check | Command | Result |
|-------|---------|--------|
| C port of the control rules | `cc -std=c11 -Wall -Wextra -Werror -O2 -I idf/main idf/main/logic.c idf/host/test_logic.c -o /tmp/robo-logic-test && /tmp/robo-logic-test` | **107 passed, 0 failed** |
| IDF image | `idf.py -C idf build` | **Not built here.** `idf.py` is not installed on this machine. |

`core/` was not modified by the port. Its last recorded host run is the 82 Rust
tests already listed in git history; that suite was not re-run for this change.

What has **not** been verified, because no board was attached and no flasher is
installed:

* flashing and booting on real hardware,
* the Wi-Fi access point, DHCP, HTTP and the WebSocket handshake,
* PWM output, motor direction, motor current, braking behaviour,
* ADC accuracy and the battery thresholds,
* the physical emergency stop.

Every hardware number in these docs is either measured from the crate sources or
marked **PROVISIONAL**. See [docs/HARDWARE_ASSUMPTIONS.md](docs/HARDWARE_ASSUMPTIONS.md)
before you connect a battery.

## Hardware at a glance

* ESP32-WROOM-32 dev board (3.3 V logic). **GPIO16/17 must be free**, so not a WROVER.
* 2 × BTS7960 / IBT-2 modules: driver **A = left pair**, driver **B = right pair**.
* 4 × 600 RPM Johnson motors, 95 mm × 20 mm wheels, differential steering.
* 12 V DC max, chassis 30 × 30 × 40 cm, 5 kg.
* One physical, latching emergency stop in the battery line.

Pin map, wiring and the battery divider: [docs/WIRING.md](docs/WIRING.md).

## Why ESP-IDF

The motor math is tiny. The delay that matters is the radio and the time from a
button frame to a PWM update.

* The control task is pinned to the app core (CPU 1) at priority 22. Wi-Fi and
  HTTP stay on the protocol core, so a received frame does not wait behind the
  motor loop and the motor loop does not wait behind the radio.
* A drive, arm, disarm, ack or e-stop frame notifies that task. It does not sit
  until the next tick. The idle tick is 2 ms, which is the fastest period the
  old validator allowed. Ramp rates are still per second, so the speed profile
  does not get steeper; the steps just get smaller.
* AMPDU and AMSDU are off, modem sleep is off, and the command socket sets
  `TCP_NODELAY`. A drive frame is 12 bytes. Aggregation would hold it.
* LEDC runs at 20 kHz, 11-bit, and falls back to 10 kHz, 12-bit if that timer
  cannot be built. 20 kHz is above the audible band and under the BTS7960's
  25 kHz switching limit. The permille duty window is unchanged.
* The CPU is set to 240 MHz. The control math runs from IRAM. The LEDC and GPIO
  driver functions are placed in IRAM as well, so the duty-register write is
  not fetched from flash while the radio is using it.
* The softAP DHCP server is the one built into ESP-IDF. The robot is still
  `192.168.4.1`.

## Build, test, flash

Logic tests need a host C compiler and no hardware. The image needs ESP-IDF 5.3
or newer. The old Rust toolchain steps are in
[docs/SETUP_WINDOWS.md](docs/SETUP_WINDOWS.md) and apply only to `firmware/`.

```powershell
# logic tests - no hardware, no ESP-IDF
cc -std=c11 -Wall -Wextra -Werror -O2 -I idf/main idf/main/logic.c idf/host/test_logic.c -o robo-logic-test
./robo-logic-test

# firmware
cd idf
idf.py set-target esp32
idf.py build
idf.py -p COM5 flash monitor
```

On macOS or Linux the monitor port is `/dev/cu.usbserial-*` or `/dev/ttyUSB0`
instead of `COM5`.

The serial log prints the access point name, the Wi-Fi password and the session
token. Leave the password blank in `idf.py menuconfig` (Robo Racer) to generate
a new one every boot. Set it only if you want a fixed password; it must be 8 to
63 characters. The token is always random per boot and is only sent inside the
WPA2 link, in the WebSocket HELLO frame.

If the serial log never appears after flashing, the module may not like 4 MB
flash settings. Check the flash size with `idf.py menuconfig` under Serial
flasher config. The control behaviour does not depend on that setting.

## Driving it

1. Power the robot. The serial log prints the access point name, the Wi-Fi
   password and the session token.
2. Join `robo-racer` from the phone (WPA2).
3. Open <http://192.168.4.1>.
4. Tap **ARM**, then hold a direction. Release to stop.

The robot boots **disarmed** and never arms itself, not even after a reboot.
The AP password is random per boot unless you set one with `idf.py menuconfig`
under Robo Racer. The session token is always random per boot and is only
delivered inside the encrypted Wi-Fi link.

Safety model, in order of authority:

1. **Physical E-stop** — cuts motor power in hardware. Nothing in this repository
   can override it.
2. **Command timeout** (250 ms) — if a moving robot stops hearing commands it
   latches a fault and stops.
3. **Link watchdog** (2 s) — a silent browser is a lost link, which latches a
   fault while armed.
4. **In-app E-STOP** — latches a fault that only an explicit **CLEAR FAULT** plus
   a fresh **ARM** can clear.
5. **Ramp + duty limits** — acceleration, deceleration and a coast window on
   direction changes protect the gearboxes.

## Repository layout

```
idf/                 ESP-IDF firmware (this is what you flash)
  main/logic.c       safety, steering, ramp, battery, protocol
  main/main.c        core-1 control loop, ADC, boot order
  main/motors.c      LEDC, BTS7960 enable sequencing
  main/net.c         softAP, DHCP, HTTP, WebSocket
  main/mailbox.c     core-0 / core-1 command slot
  host/test_logic.c  host checks for the C port
core/                Rust specification of the same rules (no HAL)
firmware/            previous esp-hal firmware, not the flash target
web/                 phone app (HTML/CSS/JS, no build step, no dependencies)
docs/                wiring, setup, calibration, testing, security, checklist
```

## Documentation

* [docs/WIRING.md](docs/WIRING.md) — pin table, driver wiring, divider, E-stop, power
* [docs/SETUP_WINDOWS.md](docs/SETUP_WINDOWS.md) — Rust `firmware/` toolchain only. Flash `idf/` instead; the build commands are above.
* [docs/CALIBRATION.md](docs/CALIBRATION.md) — ADC two-point calibration, duty tuning
* [docs/TESTING.md](docs/TESTING.md) — automated tests and the bench test script
* [docs/SECURITY.md](docs/SECURITY.md) — threat model and what is actually protected
* [docs/COMPETITION_CHECKLIST.md](docs/COMPETITION_CHECKLIST.md) — event-day lists
* [docs/HARDWARE_ASSUMPTIONS.md](docs/HARDWARE_ASSUMPTIONS.md) — what is still unconfirmed
