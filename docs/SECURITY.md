# Security

The real asset is not the data — it is a 5 kg machine moving at speed near people,
plus a Li-ion pack. The security goal is therefore narrow and honest:

> Nobody who is not standing next to the robot, holding the phone that is driving
> it, can make it move.

This document says what is implemented, what it protects against, and what it does
**not** protect against. Do not read the checklist as a claim of more than that.

## Trust boundaries

```
phone browser
   │  WPA2-PSK over the robot's own AP
   ▼
ESP32 Wi-Fi  ── DHCP / HTTP / WebSocket on 192.168.4.1
   │  session token + sequence numbers + bounded frames
   ▼
protocol decoder ── safety supervisor ── ramp ── PWM
                                        │
                                        └── physical E-stop (outside all of this)
```

The physical E-stop is the only control that does not depend on any of the
layers above it.

## Controls

| Control | Where | What it actually does | Limit |
|---------|-------|----------------------|-------|
| WPA2-Personal, random per-boot password | `firmware/src/net.rs` | the AP cannot be joined without the 12-character password printed on the serial console | anyone who sees the serial log or the sticker knows it; WPA2-PSK is not WPA3 |
| No credentials in the repository or the web assets | `option_env!("ROBO_WIFI_PASS")`, `web/*` | a checked-out repository contains no working password or token | if you set `ROBO_WIFI_PASS` at build time, the password is inside the binary — treat the binary as secret |
| Session token, random per boot | `main.rs`, `state.rs` | every client frame must carry the token; frames with another token are rejected | a client already on the AP gets the token from the `HELLO` frame; the token is not an operator identity |
| Token only in `HELLO` | `net.rs` | the token never appears in HTML, CSS or JavaScript, so it cannot be read from the page or the cache | — |
| Monotonic sequence numbers | `core/src/protocol.rs` | replayed or out-of-order frames are rejected (`StaleSequence`) | wrap-around is handled; 16 bits is enough for 20 Hz over ~54 minutes of continuous driving |
| Fixed 12-byte client frames, strict validation | `core/src/protocol.rs` | magic, version, kind, mode, flags, token all checked; unknown values rejected | — |
| Bounded WebSocket payloads | `core/src/websocket.rs` | payloads above `MAX_PAYLOAD` and fragmented messages are rejected before allocation | the framing layer is minimal by design |
| Bounded HTTP request head | `net.rs` (1024 bytes) | no unbounded read, no request body parsing | only `GET /`, `/styles.css`, `/app.js`, `/ws` exist |
| One control session at a time | `net.rs` | a second phone cannot drive while the first session is open | after the first session closes, the next client may connect — physical presence is the real gate |
| Socket inactivity timeout | `net.rs`, `link_timeout_ms` | a silent client is disconnected; while armed that latches `LinkLost` | TCP-level, 2 s |
| Command timeout | `core/src/safety.rs` | a moving robot with no fresh command faults in 250 ms | only while the last command asked for motion |
| Explicit arming, no auto-resume | `main.rs`, `core/src/safety.rs` | boots `DISARMED`; a reboot, link loss or fault always requires ARM again | — |
| In-app E-STOP latches | `core/src/safety.rs` | cannot be armed away; needs CLEAR FAULT + ARM | firmware-level, not a safety device |
| No OTA, no inbound internet, no cloud | — | the only network is the robot's own AP; there is no attack surface from the internet | — |

## Threat model

| Threat | Outcome |
|--------|---------|
| Someone joins the AP without the password | blocked by WPA2-PSK |
| Someone on the AP connects and drives | **possible by design** — the AP password is the only gate. Keep it off the robot's body and out of photos |
| A frame replayed from a capture | rejected by token + sequence |
| A crafted frame with a bad length/kind/mode/token | rejected, connection may be closed |
| A second phone takes over mid-run | refused while the first session is open; after a disconnect it can connect, so disarm before handing over |
| Brute force on the token | 32 bits, and the attacker must already be on the AP; there is no rate limiter, but the token is only useful with a live TCP session |
| Sniffing another client's traffic | blocked by WPA2 per-client unicast encryption (CCMP) |
| Deauthentication / jamming | **not prevented.** The robot faults and stops; this is a denial of service, not a hijack |
| Someone presses the buttons physically | **not prevented** — the arm state is in the robot. That is why the E-stop is physical |
| Stolen phone still associated | the app is a web page; close the tab. Disarm and power down the robot when unattended |
| Firmware tampering / reflash | **not prevented** — physical USB access is full control. Secure boot and flash encryption are not enabled |

## Known gaps

1. **No TLS.** The WebSocket is plain TCP inside WPA2. Anyone who knows the AP
   password and can capture traffic on the AP (for example by controlling the AP
   itself) can read and inject commands. Acceptable for a competition AP with a
   per-boot password; not acceptable for anything on a shared network.
2. **The token is per boot, not per session.** Two tabs on the same phone share
   it. A reconnect after a drop reuses it.
3. **No operator authentication.** Whoever holds the phone can arm. The physical
   E-stop is the compensating control.
4. **The AP password is printed on the serial console** and, with
   `ROBO_WIFI_PASS`, embedded in the binary. Do not show the boot log on a
   projector, and do not publish the binary.
5. **No persistent audit log.** Faults are visible in the app and on the serial
   console only.
6. **No rate limiting.** A client on the AP can open connections in a loop; the
   single-session design keeps it from driving, but it can keep the socket busy.

## Operating rules

* Leave `ROBO_WIFI_PASS` unset. A per-boot random password is stronger than a
  fixed one that ends up in a screenshot.
* Change the SSID to something unique per event if several robots run the same
  firmware; the app connects to whatever host served it, but *you* need to be
  sure you joined the right robot.
* Keep the physical E-stop within reach of the driver at all times.
* **DISARM before picking the robot up**, before changing a battery, and before
  handing the phone to anyone.
* Close the browser tab when you are done; a background tab that keeps polling
  looks like an active operator.
* Power the robot down when it is unattended. There is no idle lock.
* If the app shows an unexpected fault, treat it as real: clear it only after
  you have found the cause.

## Reporting a problem

This is a student competition project. If you find a way to make the robot move
without the AP password, or without a live session holding the token, that is a
serious bug: stop using the robot, keep the physical E-stop connected, and record
the steps (browser, OS, network state) before you change anything.
