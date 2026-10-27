//! The control loop: safety supervision, ramping and motor output.
//!
//! This task runs at `Config::safety.control_period_ms` (10 ms by default).
//! It never blocks on I/O: the network task only writes the mailbox, and the
//! battery ADC is sampled synchronously every 200 ms (about 40 us).
//!
//! Ordering inside one cycle matters:
//!
//! 1. battery sample (only every 200 ms),
//! 2. operator events (emergency stop, disarm, acknowledge, arm),
//! 3. new drive frame -> safety supervisor,
//! 4. safety decision -> ramp -> motor driver.

use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::AdcChannel;
use esp_hal::gpio::{AnalogPin, AnyPin, Level, Output, OutputConfig};
use esp_hal::time::Instant;

use robo_core::battery::BatteryMonitor;
use robo_core::config::{Config, SpeedMode, SpeedScales};
use robo_core::ramp::Ramp;
use robo_core::safety::{Output as SafetyOutput, RobotState, SafetySupervisor};
use robo_core::steering;
use robo_core::DriveCommand;

use crate::battery::BatterySensor;
use crate::motors::Motors;
use crate::state;

/// How often the pack voltage is sampled.
const BATTERY_PERIOD_MS: u64 = 200;
/// Upper bound for the loop time step, so a stalled tick cannot create a huge
/// ramp jump.
const MAX_STEP_MS: u32 = 100;

/// Runs the control loop forever.
pub async fn run<P: AdcChannel + AnalogPin>(
    mut motors: Motors,
    mut battery: BatterySensor<P>,
    mut led: Output<'static>,
    cfg: Config,
    boot: Instant,
) -> ! {
    let mut supervisor = SafetySupervisor::new(cfg.safety.clone());
    supervisor.boot_complete();
    let mut ramp = Ramp::new();
    let mut monitor = BatteryMonitor::new();

    let mut last_seq = 0u16;
    let mut last_ms = 0u32;
    let mut last_battery_ms = 0u32;
    let mut led_toggle_ms = 0u32;

    loop {
        Timer::after(Duration::from_millis(cfg.safety.control_period_ms as u64)).await;
        let now_ms = boot.elapsed().as_millis() as u32;

        // ---- battery ----
        if now_ms.wrapping_sub(last_battery_ms) >= BATTERY_PERIOD_MS as u32 {
            last_battery_ms = now_ms;
            if let Some(millivolts) = battery.read_mv() {
                monitor.update(millivolts, &cfg.battery);
            }
            state::with(|m| {
                m.battery_mv = monitor.pack_mv().unwrap_or(0).min(u16::MAX as u32) as u16;
                m.battery_ok = !monitor.critical_confirmed(&cfg.battery);
            });
        }

        let inputs = state::take_inputs();

        // ---- operator events ----
        supervisor.note_link(inputs.link_up);
        supervisor.note_battery(inputs.battery_ok);

        if inputs.estop {
            supervisor.emergency_stop();
        }
        if inputs.disarm {
            supervisor.disarm();
            ramp.reset();
        }
        if inputs.ack {
            let _ = supervisor.acknowledge_fault();
            ramp.reset();
        }
        if inputs.arm {
            if supervisor.arm().is_ok() {
                ramp.reset();
            }
        }

        // ---- new command ----
        if inputs.seq != last_seq {
            last_seq = inputs.seq;
            let command = steering::resolve(
                inputs.buttons,
                &cfg.drive,
                scale_for(inputs.mode, &cfg.scales),
            );
            supervisor.note_command(now_ms, command.command);
        }

        // ---- safety decision ----
        let applied = match supervisor.update(now_ms) {
            SafetyOutput::Move(target) => {
                let dt = now_ms.wrapping_sub(last_ms).min(MAX_STEP_MS);
                ramp.update(target, dt, &cfg.drive)
            }
            SafetyOutput::Disabled => {
                // Released, disarmed or faulted: stop now, do not coast down.
                ramp.reset();
                DriveCommand::STOP
            }
        };
        last_ms = now_ms;

        motors.apply(applied);

        // ---- status LED ----
        match supervisor.state() {
            RobotState::Armed | RobotState::Driving => led.set_high(),
            RobotState::Fault => {
                if now_ms.wrapping_sub(led_toggle_ms) >= 250 {
                    led_toggle_ms = now_ms;
                    led.toggle();
                }
            }
            _ => led.set_low(),
        }

        // ---- publish ----
        state::with(|m| {
            m.state = supervisor.state();
            m.fault = supervisor.fault();
            m.applied = applied;
        });
    }
}

/// Speed-mode scale in percent.
fn scale_for(mode: SpeedMode, scales: &SpeedScales) -> u16 {
    match mode {
        SpeedMode::Slow => scales.slow_percent,
        SpeedMode::Normal => scales.normal_percent,
        SpeedMode::Fast => scales.fast_percent,
    }
}

/// Creates the status LED output, off at boot.
pub fn init_led(pin: impl Into<AnyPin<'static>>) -> Output<'static> {
    Output::new(pin.into(), Level::Low, OutputConfig::default())
}
