# Windows setup (PowerShell)

Target: Windows 10/11, a USB cable that carries data (not a charge-only cable),
and an ESP32-WROOM-32 board.

Everything below is PowerShell. Run it in a normal window; no administrator
rights are needed except for the driver install in step 6.

## 1. Rust host toolchain

Install rustup from <https://win.rustup.rs/x86_64> and accept the default
`stable-x86_64-pc-windows-msvc` toolchain.

The host toolchain is what runs the tests and the build tools, so it needs the
MSVC linker. Install **Visual Studio Build Tools** with the
*Desktop development with C++* workload. Verify:

```powershell
rustc -V
cargo -V
```

## 2. The Xtensa toolchain

The ESP32 is an Xtensa chip; upstream Rust does not ship that target.

```powershell
cargo install espup --locked
espup install
```

`espup install` creates a toolchain named `esp` and writes
`$HOME\export-esp.ps1`. You do **not** need to source that file for this
project: the build uses `rust-lld` (configured in
`firmware/.cargo/config.toml`), so the GCC Xtensa toolchain is not required.
Sourcing it is harmless if you already use it for other projects.

Check the toolchain exists:

```powershell
rustup toolchain list
```

You should see a line containing `esp`. `firmware/rust-toolchain.toml` selects
it automatically **when the working directory is inside `firmware/`**.

## 3. Flasher

```powershell
cargo install espflash --locked
espflash --version
```

If you prefer Python, `pip install esptool` also works, but `espflash` is the
documented path because it understands the ELF directly.

## 4. Configure (optional)

Both settings are read at **compile time**, so changing them requires a rebuild.

```powershell
# access point name, default "robo-racer"
$env:ROBO_WIFI_SSID = "robo-racer"

# fixed WPA2 password, at least 8 characters
$env:ROBO_WIFI_PASS = "your-own-password"
```

Leave `ROBO_WIFI_PASS` unset for competition use. The firmware then generates a
fresh 12-character password on every boot and prints it on the serial console,
so no working credential exists in the binary, in the repository or in your shell
history. See [SECURITY.md](SECURITY.md).

## 5. Build

```powershell
cd firmware
cargo build --release --target xtensa-esp32-none-elf
```

The artifact is at the **workspace** target directory, one level up:

```
target\xtensa-esp32-none-elf\release\robo-racer
```

Expected: `Finished` with no errors and no warnings, about 1.27 MB (1.21 MiB) of image.

Two traps:

* Run the build from `firmware\`, not from the repository root. From the root,
  `rust-toolchain.toml` is not found, Cargo picks the stable host toolchain and
  the Xtensa crates fail to compile.
* `cargo build` without `--target` builds for Windows and fails inside
  `xtensa-lx-rt`. Always pass the target.

Host tests need no hardware and no Xtensa toolchain:

```powershell
cd ..
cargo test -p robo-core
```

## 6. Flash and monitor

Plug the board in, then:

```powershell
espflash flash --monitor --chip esp32 --port COM5 ..\target\xtensa-esp32-none-elf\release\robo-racer
```

* Replace `COM5` with your port (`Get-CimInstance Win32_SerialPort` lists them,
  or use Device Manager).
* Omit `--port` to let espflash prompt with detected ports.
* `--monitor` keeps the serial console open at 115200 baud. Exit with
  `Ctrl+C`.
* If flashing fails with "Failed to connect", hold **BOOT** (GPIO0) while it
  starts, or press EN/RST.

### What the boot log should say

```
robo-racer: boot
robo-racer: access point 'robo-racer' up, password 'xxxxxxxxxxxx', open http://192.168.4.1
robo-racer: session token 1a2b3c4d
```

Then join the access point from the phone and open <http://192.168.4.1>.

The token line is for verification only — the app adopts the token from the
WebSocket `HELLO` frame, you never type it.

If the log instead shows `configuration rejected`, `PWM init failed` or
`access point failed`, the firmware halts with the motors disabled and repeats
`halted, motors disabled` once a second. The reason is printed once, above it.

## 7. Driver

If Windows shows the board as an unknown device, install the Silicon Labs
CP210x or the WCH CH340 driver, depending on the USB-serial chip on your board.
Both are linked from the usual ESP32 getting-started pages; install the one
matching the chip printed on the board.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `can't find crate for core` / `xtensa-lx-rt` errors | built from the repo root, or no `--target` | `cd firmware`, pass `--target xtensa-esp32-none-elf` |
| `error: toolchain 'esp' is not installed` | `espup install` not run, or run as another user | rerun `espup install` |
| `linker 'xtensa-esp32-elf-gcc' not found` | `firmware/.cargo/config.toml` missing or edited | restore the rust-lld linker flags |
| espflash: `No serial ports found` | charge-only cable, or missing driver | try another cable, install CP210x/CH340 |
| Boot loop with `rst:0x...` and a panic address | power supply sags when Wi-Fi starts | power from a 5 V source that can give 500 mA, add a 470 µF capacitor at the board |
| AP appears but no IP is handed out | DHCP task failed (logged once) | reset the board; check that only one phone connects during setup |
| App connects, then `DISCONNECTED` every 2 s | another browser tab holds the control session | close other tabs; the firmware serves one controller at a time |
| Robot does not move when armed | enable pins not wired, or E-stop open | check the E-stop, then measure R_EN/L_EN at 3.3 V while driving |

## 8. Cleaning up

```powershell
cargo clean                      # from the repo root, removes both targets
rustup toolchain uninstall esp   # only if you no longer need Xtensa
```
