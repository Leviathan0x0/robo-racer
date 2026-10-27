//! robo-racer firmware entry point.
//!
//! Boot order is deliberate:
//!
//! 1. heap + logging,
//! 2. start the esp-rtos scheduler (required before the radio),
//! 3. drive the motor enable pins low (the motors must be inert from the
//!    first moment),
//! 4. validate the configuration - a bad configuration aborts boot,
//! 5. start the access point and the control server,
//! 6. enter the control loop, which owns the motors, the battery ADC and the
//!    status LED.

#![no_std]
#![no_main]

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::ram;
use esp_hal::rng::Rng;
use esp_hal::time::Instant;
use esp_hal::timer::timg::TimerGroup;

use robo_core::config::Config;

mod battery;
mod control;
mod motors;
mod net;
mod pins;
mod state;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: embassy_executor::Spawner) {
    let boot = Instant::now();
    esp_println::logger::init_logger_from_env();

    // Wi-Fi needs a heap; two regions keep the radio's buffers out of the way
    // of the rest of the firmware.
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 40 * 1024);

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // The scheduler must run before the radio is initialised.
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    esp_println::println!("robo-racer: boot");

    // The status LED and the battery tap are configured before the drivers so
    // the board has a visible sign of life even if motor setup fails.
    let led = control::init_led(peripherals.GPIO2);
    let battery = battery::BatterySensor::new(peripherals.ADC1, peripherals.GPIO34);

    // Enable pins go low inside `Motors::new`, before any PWM is configured.
    let ledc = motors::init_ledc(peripherals.LEDC);
    let board = pins::Board::new(
        peripherals.GPIO16,
        peripherals.GPIO17,
        peripherals.GPIO18,
        peripherals.GPIO19,
        peripherals.GPIO21,
        peripherals.GPIO22,
        peripherals.GPIO23,
        peripherals.GPIO25,
    );

    let config = Config::default();
    if let Err(error) = config.validate() {
        esp_println::println!("robo-racer: configuration rejected: {:?}", error);
        // Motors stay disabled: the driver enables were never raised.
        dead_loop().await;
    }

    let motors = match motors::Motors::new(&ledc, board, &config.motors) {
        Ok(motors) => motors,
        Err(error) => {
            esp_println::println!("robo-racer: PWM init failed: {:?}", error);
            dead_loop().await;
        }
    };

    let token = session_token();
    state::set_token(token);
    esp_println::println!("robo-racer: session token {:08x}", token);

    let (wifi_controller, interface) = match net::start_ap(peripherals.WIFI) {
        Ok(parts) => parts,
        Err(error) => {
            esp_println::println!("robo-racer: access point failed: {:?}", error);
            dead_loop().await;
        }
    };
    let (stack, runner) = net::start_stack(interface);
    spawner.spawn(net::net_runner(runner).unwrap());
    spawner.spawn(net::dhcp_task(stack).unwrap());
    spawner.spawn(net::http_task(stack, token, boot, config.safety.link_timeout_ms as u64).unwrap());

    // The controller must outlive the radio: it is never dropped because the
    // control loop below never returns.
    let _wifi_controller = wifi_controller;

    control::run(motors, battery, led, config, boot).await
}

/// Non-zero random session token.
fn session_token() -> u32 {
    let token = Rng::new().random();
    if token == 0 { 1 } else { token }
}

/// Halts with the motors disabled, reporting once a second.
async fn dead_loop() -> ! {
    loop {
        esp_println::println!("robo-racer: halted, motors disabled");
        Timer::after(Duration::from_secs(1)).await;
    }
}
