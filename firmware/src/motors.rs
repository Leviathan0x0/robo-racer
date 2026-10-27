//! BTS7960 / IBT-2 dual half-bridge driver output.
//!
//! One [`Side`] owns the two enable pins and the two PWM pins of a single
//! BTS7960 module, which drives the two motors of one side in parallel.
//!
//! Hardware rules enforced here:
//!
//! * both enable pins are driven low before the PWM channels are configured,
//!   so the driver is disabled from the first microsecond,
//! * a non-zero command never leaves both PWM inputs high (that would brake),
//! * the duty is mapped into `[min_duty_permille, max_duty_permille]`, which
//!   keeps the motor above its stiction threshold and below the electrical
//!   limit the operator configured,
//! * [`Motors::apply`] with a stop command disables the driver instead of
//!   leaving it enabled with a zero duty.

use esp_hal::gpio::{AnyPin, DriveMode, Level, Output, OutputConfig};
use esp_hal::ledc::channel::config::Config as ChannelConfig;
use esp_hal::ledc::channel::{self, Channel, ChannelHW, ChannelIFace, Number};
use esp_hal::ledc::timer::{self, Timer, TimerIFace};
use esp_hal::ledc::{Ledc, LowSpeed, LSGlobalClkSource};
use esp_hal::time::Rate;
use static_cell::StaticCell;

use robo_core::config::MotorTuning;
use robo_core::{DriveCommand, COMMAND_MAX};

use crate::pins::Board;

/// PWM frequency. 10 kHz keeps the driver out of the audible band and stays
/// far below the BTS7960 switching limit.
const PWM_FREQ_KHZ: u32 = 10;
/// PWM resolution: 12 bits gives 0.024 % duty steps at 10 kHz.
const PWM_RESOLUTION_BITS: u32 = 12;
/// One period in raw duty counts.
const FULL_SCALE: u32 = 1 << PWM_RESOLUTION_BITS;

/// The LEDC timer must outlive the channels that borrow it, so it lives in a
/// `StaticCell` instead of on the stack.
static PWM_TIMER: StaticCell<Timer<'static, LowSpeed>> = StaticCell::new();

/// Everything that can go wrong while setting the PWM up.
///
/// The payloads are only surfaced through `Debug` in the boot log.
#[allow(dead_code)]
#[derive(Debug)]
pub enum MotorError {
    /// The timer could not be configured.
    Timer(timer::Error),
    /// A channel could not be configured.
    Channel(channel::Error),
}

impl From<timer::Error> for MotorError {
    fn from(error: timer::Error) -> Self {
        Self::Timer(error)
    }
}

impl From<channel::Error> for MotorError {
    fn from(error: channel::Error) -> Self {
        Self::Channel(error)
    }
}

/// Duty limits shared by both sides.
#[derive(Clone, Copy)]
struct Pulse {
    min_duty: u16,
    max_duty: u16,
}

impl Pulse {
    /// Converts a normalised magnitude (`1 ..= COMMAND_MAX`) into a raw duty.
    ///
    /// The mapping is linear from `min_duty` at the smallest command to
    /// `max_duty` at full command, so there is no jump at zero.
    fn raw(&self, magnitude: u16) -> u32 {
        let span = (self.max_duty - self.min_duty) as u32;
        let permille = self.min_duty as u32
            + span * magnitude.min(COMMAND_MAX as u16) as u32 / COMMAND_MAX as u32;
        (permille * FULL_SCALE / 1000).min(FULL_SCALE - 1)
    }
}

/// One BTS7960 module and the motors connected to it.
pub struct Side {
    rpwm: Channel<'static, LowSpeed>,
    lpwm: Channel<'static, LowSpeed>,
    r_en: Output<'static>,
    l_en: Output<'static>,
    pulse: Pulse,
}

impl Side {
    /// Disables the driver and parks both PWM inputs low.
    pub fn disable(&mut self) {
        self.r_en.set_low();
        self.l_en.set_low();
        self.rpwm.set_duty_hw(0);
        self.lpwm.set_duty_hw(0);
    }

    /// Enables the driver (both halves) and applies a signed command.
    ///
    /// A zero command disables the driver, which lets the motor coast.
    pub fn drive(&mut self, command: i16) {
        if command == 0 {
            self.disable();
            return;
        }
        self.r_en.set_high();
        self.l_en.set_high();

        let raw = self.pulse.raw(command.unsigned_abs());
        if command > 0 {
            self.lpwm.set_duty_hw(0);
            self.rpwm.set_duty_hw(raw);
        } else {
            self.rpwm.set_duty_hw(0);
            self.lpwm.set_duty_hw(raw);
        }
    }
}

/// Both sides of the robot.
pub struct Motors {
    left: Side,
    right: Side,
}

impl Motors {
    /// Builds the PWM timer and the four channels.
    ///
    /// The enable pins are driven low *before* the LEDC is touched. If the
    /// LEDC cannot be configured the caller must keep the robot disarmed, and
    /// the enable pins stay low, so the motors cannot move either way.
    pub fn new(ledc: &Ledc<'static>, board: Board, tuning: &MotorTuning) -> Result<Self, MotorError> {
        let timer = configure_timer(ledc)?;
        let pulse = Pulse {
            min_duty: tuning.min_duty_permille,
            max_duty: tuning.max_duty_permille,
        };
        let mut left = Side {
            rpwm: configure_channel(ledc, timer, Number::Channel0, board.left_rpwm)?,
            lpwm: configure_channel(ledc, timer, Number::Channel1, board.left_lpwm)?,
            r_en: Output::new(board.left_r_en, Level::Low, OutputConfig::default()),
            l_en: Output::new(board.left_l_en, Level::Low, OutputConfig::default()),
            pulse,
        };
        let mut right = Side {
            rpwm: configure_channel(ledc, timer, Number::Channel2, board.right_rpwm)?,
            lpwm: configure_channel(ledc, timer, Number::Channel3, board.right_lpwm)?,
            r_en: Output::new(board.right_r_en, Level::Low, OutputConfig::default()),
            l_en: Output::new(board.right_l_en, Level::Low, OutputConfig::default()),
            pulse,
        };

        left.disable();
        right.disable();
        Ok(Self { left, right })
    }

    /// Applies a command: stop disables both drivers.
    ///
    /// The caller must only pass commands that the safety supervisor approved.
    pub fn apply(&mut self, cmd: DriveCommand) {
        if cmd.is_stop() {
            self.force_disable();
            return;
        }
        self.left.drive(cmd.left);
        self.right.drive(cmd.right);
    }

    /// Immediate, unconditional stop: both drivers disabled, both PWM low.
    pub fn force_disable(&mut self) {
        self.left.disable();
        self.right.disable();
    }
}

/// Enables the LEDC peripheral and selects its low-speed clock source.
pub fn init_ledc(peripheral: esp_hal::peripherals::LEDC<'static>) -> Ledc<'static> {
    let mut ledc = Ledc::new(peripheral);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    ledc
}

/// Creates and configures one PWM channel on the shared timer.
fn configure_channel(
    ledc: &Ledc<'static>,
    timer: &'static dyn TimerIFace<LowSpeed>,
    number: Number,
    pin: AnyPin<'static>,
) -> Result<Channel<'static, LowSpeed>, channel::Error> {
    let mut channel = ledc.channel::<LowSpeed>(number, pin);
    channel.configure(ChannelConfig {
        timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })?;
    channel.set_duty_hw(0);
    Ok(channel)
}

/// Configures the shared LEDC timer.
///
/// `StaticCell::init` panics if it is called twice; [`Motors::new`] is only
/// called once during boot.
fn configure_timer(ledc: &Ledc<'static>) -> Result<&'static dyn TimerIFace<LowSpeed>, timer::Error> {
    let timer = PWM_TIMER.init(ledc.timer::<LowSpeed>(timer::Number::Timer0));
    timer.configure(timer::config::Config {
        duty: timer::config::Duty::Duty12Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_khz(PWM_FREQ_KHZ),
    })?;
    Ok(&*timer)
}
