# Testing

## 1. Automated tests (no hardware)

```powershell
cargo test -p robo-core
```

Expected today: **82 tests pass** — 71 unit tests plus 11 integration tests in
`core/tests/system.rs`. Doc-tests: 0.

| Module | Tests | What they pin down |
|--------|------:|--------------------|
| `safety` | 12 | arming is explicit; stop-on-release; command timeout only while moving; link loss only while armed; e-stop latches; ack requires link + battery; ack returns to DISARMED |
| `steering` | 13 | the four-button table; contradictory pairs stop; stop wins; scale modes; clamping |
| `ramp` | 7 | acceleration and deceleration limits; coast before reversing; no jump; reset |
| `protocol` | 13 | frame layout and length; magic/version; token check; replay and wrap-around sequence; bad kind/mode; error codes |
| `websocket` | 12 | handshake accept key; extended lengths; masking; fragmentation rejection; oversize payload rejection |
| `battery` | 5 | divider math; EMA; classification; critical debounce; offset |
| `config` | 7 | every range check; default configuration is valid |
| `lib.rs` (`DriveCommand`) | 2 | clamping and `is_stop` |
| integration | 11 | whole-journey scenarios chaining the modules in firmware order |

The integration tests are the ones to read first:
`core/tests/system.rs`.

```powershell
cargo test -p robo-core --test system -- --nocapture
```

## 2. Firmware build check

```powershell
cd firmware
cargo build --release --target xtensa-esp32-none-elf
```

Expected: `Finished` with **0 warnings**. A warning is a defect in this project;
the current tree is clean.

---

## 3. Bench test (wheels off the ground)

Prerequisites: wiring checked as in [WIRING.md](WIRING.md), phone on the same
bench, robot on blocks so all four wheels spin free.

Do this in order. Do not skip to driving on the floor.

| # | Step | Expected | Pass |
|---|------|----------|------|
| 1 | Power the robot with the app closed | Serial prints `boot`, the AP line and the token. **No motor moves.** | ☐ |
| 2 | Check the driver outputs with a meter | `R_EN` and `L_EN` at 0 V on both modules | ☐ |
| 3 | Join the AP, open `http://192.168.4.1` | Page loads, header shows `DISARMED`, battery voltage | ☐ |
| 4 | Tap **ARM** | Header shows `ARMED`, LED on. Wheels still. | ☐ |
| 5 | Hold **Forward** | Both sides drive forward, ramping up | ☐ |
| 6 | Release | Wheels stop immediately, state stays `ARMED`, **no fault** | ☐ |
| 7 | Hold **Backward** | Both sides reverse | ☐ |
| 8 | Hold **Left** | Left wheel slower than right; robot would turn left | ☐ |
| 9 | Hold **Right** | Right wheel slower than left | ☐ |
| 10 | Hold forward **and** backward together | Everything stops (conflict rule) | ☐ |
| 11 | Tap **STOP** while driving | Immediate stop | ☐ |
| 12 | Switch to **SLOW**, hold forward | Noticeably slower than NORMAL | ☐ |
| 13 | Hold forward, press the phone's home button | Robot stops within ~250 ms and shows `FAULT: command timeout` | ☐ |
| 14 | With the robot armed, put the phone in airplane mode | Within ~2 s: `FAULT: link lost` | ☐ |
| 15 | Reconnect, tap **CLEAR FAULT**, then **ARM** | Back to `ARMED` and drivable | ☐ |
| 16 | Hold **E-STOP** for 600 ms | `FAULT: emergency stop`; the robot does not move even if you keep pressing directions | ☐ |
| 17 | Tap **DISARM** while driving | Immediate stop, `DISARMED` | ☐ |
| 18 | Press the **physical** E-stop while driving | Power is cut to the motors; they stop dead and stay stopped | ☐ |
| 19 | Release the physical E-stop | Robot does **not** resume; still `DISARMED`/fault until re-armed | ☐ |
| 20 | Power-cycle the robot | Boots `DISARMED`, no motion, no auto-arm | ☐ |
| 21 | Open a second phone browser tab to the same URL | The second tab waits or disconnects; only one controller drives | ☐ |
| 22 | Watch the battery voltage in the header | Agrees with a multimeter within ~50 mV | ☐ |
| 23 | Disconnect the battery divider, power on | `BATTERY_OK = 0`; arming still possible, but a `LowBattery` fault appears once armed | ☐ |

Notes on the expected values:

* step 13 uses `command_timeout_ms = 250`, so 250 ms is the floor; a phone that
  keeps sending in the background (some browsers do) may take longer. If it
  never faults, the browser is still sending frames — that is a test failure of
  your app/OS combination, not of the firmware.
* step 14 waits for `link_timeout_ms = 2000` plus the TCP retransmit time.
* step 20 is the important one for competition: **no auto-resume, ever**.

## 4. Floor test

Only after every bench item passes.

1. Low speed mode, 5 m straight line: the robot tracks straight within ~0.5 m.
   If not, check wheel diameter, tyre pressure and the two motors on one side.
2. Slalom at normal speed: no fault, no wheelspin, no thermal shutdown in 3
   minutes of continuous running.
3. Full-speed run on the competition surface for 2 minutes, then touch both
   driver heatsinks. Too hot to hold = add airflow or a heatsink before the event.
4. Battery sag: watch the header voltage under acceleration. If it dips below
   10.2 V the pack is tired or undersized.
5. Range test: drive to the far corner of the arena and confirm the link stays
   up (the AP is inside the robot; range is typically tens of metres indoors).

## 5. Test record

| Date | Firmware commit/build | Bench pass | Floor pass | Notes |
|------|-----------------------|-----------|-----------|-------|
|      |                       |           |           |       |

## 6. What the tests do not cover

These are not tested by `cargo test` and were not tested on hardware in this
repository:

* real PWM output and motor direction,
* actual current draw, driver temperature, brownout behaviour,
* Wi-Fi AP behaviour, DHCP lease, WebSocket handshake with a real phone browser,
* ADC accuracy and battery thresholds against a real pack,
* the physical E-stop,
* timing jitter of the 100 Hz loop under Wi-Fi load.

Item 6 of [HARDWARE_ASSUMPTIONS.md](HARDWARE_ASSUMPTIONS.md) lists them as
unverified; the bench test above is how you close them.
