# Competition checklist

Print this. Tick with a pen; "I remember it" is not a checklist.

Robot envelope: **30 × 30 × 40 cm, 5 kg** limit. Weigh and measure it before the
event, with the battery fitted and the E-stop in its final position.

## A. One week before

- [ ] `git`/folder snapshot of the exact firmware that works, and a note of the
      toolchain (`rustc -V` from inside `firmware/`).
- [ ] `cargo test -p robo-core` passes.
- [ ] `cd firmware; cargo build --release --target xtensa-esp32-none-elf` is
      clean: 0 errors, 0 warnings.
- [ ] One spare flashed ESP32-WROOM-32 board with the same firmware.
- [ ] Both BTS7960 modules have a heatsink and airflow.
- [ ] Every screw on a wheel hub and on a motor mount checked with a driver.
- [ ] Battery divider measured and recorded; pack voltage in the app matches a
      multimeter.
- [ ] Charge every pack; label them 1, 2, 3 with the date.
- [ ] Two USB cables that carry data (not charge-only).
- [ ] Phone charged, and the browser bookmarked to `http://192.168.4.1`.
- [ ] Rules re-read: weight, size, battery chemistry, radio, E-stop.

## B. Packing list

- [ ] Robot, battery packs (2–3), charger.
- [ ] Laptop with the repo, the `esp` toolchain and `espflash` installed.
- [ ] USB cables, multimeter, screwdriver set, hex keys, tape, zip ties.
- [ ] Spare wheels, spare motor, spare BTS7960, spare ESP32.
- [ ] Fuse(s), spare E-stop switch.
- [ ] Phone + power bank.
- [ ] This checklist and the calibration record sheet.

## C. At the venue, before scrutineering

- [ ] Battery at storage voltage during transport; charge only in the designated
      area and never unattended.
- [ ] Weigh (≤ 5 kg) and measure (30 × 30 × 40 cm).
- [ ] Physical E-stop reachable, labelled, and **tested**: wheels off the ground,
      armed, pressed → wheels stop and cannot be driven.
- [ ] Wires taped down so no wheel can catch one.
- [ ] AP name is unique to this robot (change `ROBO_WIFI_SSID` and rebuild if two
      robots share `robo-racer`).
- [ ] Boot log checked: AP up, token printed, no `halted`.
- [ ] Full bench test from [TESTING.md](TESTING.md) done on this surface.

## D. Ten minutes before a run

- [ ] Pack freshly charged, resting voltage ≥ 12.0 V (3S). Below 11.0 V, swap it.
- [ ] Wheels free, nothing wrapped around an axle.
- [ ] Robot **DISARMED**, app showing `DISARMED` and a plausible voltage.
- [ ] ARM, brief forward pulse on blocks, DISARM. (Confirms the link and the
      drivetrain without moving the robot.)
- [ ] E-STOP held once to confirm the latch, then CLEAR FAULT.
- [ ] Phone: low-power mode off, auto-lock off or long, notifications silenced,
      other tabs closed, screen brightness up.
- [ ] Phone volume up: the app shows faults on screen, but you want to see them.

## E. During the run

- [ ] Only the driver touches the phone.
- [ ] Keep the physical E-stop within reach; press it first, ask questions later.
- [ ] Watch the pack voltage: a sustained drop under 10.2 V means the run is over.
- [ ] If the robot behaves oddly, **DISARM** and stop the run. Never debug while
      it is armed.

## F. Emergency playbook

| Symptom | Immediate action | Then |
|---------|------------------|------|
| Robot runs away or ignores the app | **Physical E-stop** | do not re-arm until you know why |
| App shows `FAULT: command timeout` while driving | release, CLEAR FAULT, ARM, continue | if it repeats, the phone is dropping frames (close other apps) |
| App shows `FAULT: link lost` | walk closer, wait for reconnect | if it repeats, the AP is out of range or the phone is roaming |
| `FAULT: low battery` | stop the run, swap the pack | charge; check the pack's internal resistance |
| `FAULT: emergency stop` | CLEAR FAULT, ARM | — |
| One side spins, the other does not | DISARM, E-stop | check the driver's `B+`/`B-`, the enable pins, then the motor pair |
| Robot pulls to one side | DISARM | check tyre diameter, hub screws, then the tuning |
| Drivers hot / smell hot | stop, E-stop | cool down, add airflow, re-check the current |
| Wi-Fi AP disappeared | power-cycle | the ESP32 may have browned out; check the 5 V supply |
| App will not connect | close all tabs, forget and rejoin the AP | read the password from the serial log |
| Nothing on serial | cable/port/driver | swap cable, check `espflash` port |

## G. After the run

- [ ] DISARM, then power down.
- [ ] Check motor and driver temperature.
- [ ] Check wheels and hubs for play; re-tighten.
- [ ] Record pack voltage and the run result on the record sheet.
- [ ] If you changed any firmware value, note it and rerun the tests before the
      next run.
- [ ] Charge packs back to storage voltage if the event is over.

## H. Do not do this

- No charging unattended, and never inside the robot.
- No firmware changes on the morning of a run without a full bench test.
- No "just one quick test" with the robot armed and held in your hand.
- No removing the physical E-stop to save weight.
- No sharing the AP password in photos, chat or on the robot's body.
