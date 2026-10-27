//! Central configuration for the robo racer.
//!
//! One struct holds every tunable value, [`Config::validate`] rejects
//! impossible or unsafe combinations at start-up, and safety-critical limits
//! are kept separate from operator-preference values.
//!
//! The defaults are **provisional**: they follow the initial calibration table
//! from the project brief and must be re-tuned on the real hardware before the
//! robot is used at speed.

/// Configuration of the differential drive behaviour.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DriveTuning {
    /// Forward command applied to both sides (normalised units).
    pub forward: i16,
    /// Reverse command applied to both sides (absolute value, normalised units).
    pub reverse: i16,
    /// Inner wheel command during a forward turn (absolute value).
    pub turn_inner: i16,
    /// Outer wheel command during a forward turn (absolute value).
    pub turn_outer: i16,
    /// Inner wheel command during a reverse turn (absolute value).
    pub reverse_turn_inner: i16,
    /// Outer wheel command during a reverse turn (absolute value).
    pub reverse_turn_outer: i16,
    /// Acceleration limit in normalised units per second.
    pub accel_per_s: i16,
    /// Deceleration / braking limit in normalised units per second.
    pub decel_per_s: i16,
    /// Motor coast time enforced before a direction reversal is applied.
    pub direction_change_coast_ms: u16,
}

impl Default for DriveTuning {
    fn default() -> Self {
        Self {
            forward: 800,
            reverse: 600,
            turn_inner: 350,
            turn_outer: 800,
            reverse_turn_inner: 350,
            reverse_turn_outer: 600,
            accel_per_s: 2000,
            decel_per_s: 4000,
            direction_change_coast_ms: 60,
        }
    }
}

/// Motor output limits.
///
/// Duties are expressed in permille of the PWM period so the same numbers work
/// for any LEDC resolution.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotorTuning {
    /// Smallest duty that reliably breaks motor stiction (permille).
    pub min_duty_permille: u16,
    /// Largest duty the firmware is allowed to request (permille).
    pub max_duty_permille: u16,
}

impl Default for MotorTuning {
    fn default() -> Self {
        Self {
            min_duty_permille: 200,
            max_duty_permille: 1000,
        }
    }
}

/// Command / link timing limits.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SafetyTuning {
    /// A command older than this (ms) is stale and stops the robot.
    pub command_timeout_ms: u32,
    /// A control link silent for this long (ms) is considered lost.
    pub link_timeout_ms: u32,
    /// Period of the control loop (ms). 10 ms == 100 Hz.
    pub control_period_ms: u32,
}

impl Default for SafetyTuning {
    fn default() -> Self {
        Self {
            command_timeout_ms: 250,
            link_timeout_ms: 2000,
            control_period_ms: 10,
        }
    }
}

/// Battery sensing configuration.
///
/// The ADC sees the pack voltage through a resistive divider; `divider_num` and
/// `divider_den` describe that divider as `V_pack = V_adc * num / den`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BatteryConfig {
    /// Number of cells in series in the pack.
    pub cell_count: u8,
    /// Warning threshold per cell (mV).
    pub warn_mv_per_cell: u16,
    /// Critical threshold per cell (mV) - reaching it latches a fault.
    pub critical_mv_per_cell: u16,
    /// Divider numerator: `V_pack = V_adc * divider_num / divider_den`.
    pub divider_num: u32,
    /// Divider denominator.
    pub divider_den: u32,
    /// Fixed calibration offset added to the computed pack voltage (mV).
    pub calibration_offset_mv: i32,
    /// Weight of the newest sample in the exponential moving average (percent).
    pub ema_alpha_percent: u8,
    /// Consecutive critical samples required before a low-battery fault is
    /// latched (debounce against a single voltage spike).
    pub critical_samples: u8,
}

impl Default for BatteryConfig {
    fn default() -> Self {
        Self {
            // PROVISIONAL: the pack has not been confirmed yet.
            cell_count: 3,
            warn_mv_per_cell: 3400,
            critical_mv_per_cell: 3300,
            // PROVISIONAL: 100 kOhm / 27 kOhm divider -> 127/27.
            divider_num: 127,
            divider_den: 27,
            calibration_offset_mv: 0,
            ema_alpha_percent: 20,
            critical_samples: 3,
        }
    }
}

impl BatteryConfig {
    /// Warning threshold of the whole pack (mV).
    pub fn warn_mv(&self) -> u32 {
        self.warn_mv_per_cell as u32 * self.cell_count as u32
    }

    /// Critical threshold of the whole pack (mV).
    pub fn critical_mv(&self) -> u32 {
        self.critical_mv_per_cell as u32 * self.cell_count as u32
    }
}

/// Operator-selectable speed mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpeedMode {
    /// Reduced speed for tight spaces and low-grip floors.
    Slow,
    /// Calibration speeds (default).
    #[default]
    Normal,
    /// Full output, only allowed up to the configured forward limit.
    Fast,
}

/// Runtime speed-mode scales (percent).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpeedScales {
    /// Scale applied in [`SpeedMode::Slow`].
    pub slow_percent: u16,
    /// Scale applied in [`SpeedMode::Normal`].
    pub normal_percent: u16,
    /// Scale applied in [`SpeedMode::Fast`].
    pub fast_percent: u16,
}

impl Default for SpeedScales {
    fn default() -> Self {
        Self {
            slow_percent: 50,
            normal_percent: 100,
            fast_percent: 100,
        }
    }
}

/// Complete configuration of the control system.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Config {
    /// Differential drive tuning.
    pub drive: DriveTuning,
    /// Motor output limits.
    pub motors: MotorTuning,
    /// Timing limits.
    pub safety: SafetyTuning,
    /// Battery sensing.
    pub battery: BatteryConfig,
    /// Speed-mode scales.
    pub scales: SpeedScales,
}

/// Reason a configuration was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// A drive command was zero, negative or above the normalised maximum.
    InvalidDriveValue(&'static str),
    /// Inner wheel speed was faster than the outer wheel speed.
    InvalidTurnRatio,
    /// Acceleration or deceleration limit was zero.
    InvalidRate(&'static str),
    /// Command timeout outside the supported window.
    InvalidCommandTimeout,
    /// Link timeout outside the supported window.
    InvalidLinkTimeout,
    /// Control period outside the supported window.
    InvalidControlPeriod,
    /// Battery thresholds inconsistent or out of range.
    InvalidBatteryThreshold,
    /// Divider description invalid.
    InvalidDivider,
    /// Speed-mode scale out of range or not monotonic.
    InvalidSpeedScale,
}

impl Config {
    /// Validates the configuration.
    ///
    /// Called once at start-up; a rejected configuration must abort start-up
    /// before any motor output is enabled.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let d = &self.drive;
        for (name, value) in [
            ("forward", d.forward),
            ("reverse", d.reverse),
            ("turn_inner", d.turn_inner),
            ("turn_outer", d.turn_outer),
            ("reverse_turn_inner", d.reverse_turn_inner),
            ("reverse_turn_outer", d.reverse_turn_outer),
        ] {
            if value <= 0 || value > crate::COMMAND_MAX {
                return Err(ConfigError::InvalidDriveValue(name));
            }
        }
        if d.turn_inner > d.turn_outer || d.reverse_turn_inner > d.reverse_turn_outer {
            return Err(ConfigError::InvalidTurnRatio);
        }
        if d.accel_per_s <= 0 {
            return Err(ConfigError::InvalidRate("accel_per_s"));
        }
        if d.decel_per_s <= 0 {
            return Err(ConfigError::InvalidRate("decel_per_s"));
        }

        if self.safety.command_timeout_ms < 50 || self.safety.command_timeout_ms > 2000 {
            return Err(ConfigError::InvalidCommandTimeout);
        }
        if self.safety.link_timeout_ms < 100 || self.safety.link_timeout_ms > 10_000 {
            return Err(ConfigError::InvalidLinkTimeout);
        }
        if self.safety.control_period_ms < 2 || self.safety.control_period_ms > 100 {
            return Err(ConfigError::InvalidControlPeriod);
        }

        if self.battery.cell_count == 0 {
            return Err(ConfigError::InvalidBatteryThreshold);
        }
        // A Li-ion cell is between ~2.5 V (deep discharge) and 4.2 V (full).
        if self.battery.warn_mv_per_cell < 2500
            || self.battery.warn_mv_per_cell > 4200
            || self.battery.critical_mv_per_cell < 2500
            || self.battery.critical_mv_per_cell > 4200
        {
            return Err(ConfigError::InvalidBatteryThreshold);
        }
        if self.battery.critical_mv_per_cell >= self.battery.warn_mv_per_cell {
            return Err(ConfigError::InvalidBatteryThreshold);
        }
        if self.battery.divider_num == 0 || self.battery.divider_den == 0 {
            return Err(ConfigError::InvalidDivider);
        }
        if self.battery.ema_alpha_percent == 0 || self.battery.ema_alpha_percent > 100 {
            return Err(ConfigError::InvalidBatteryThreshold);
        }
        if self.battery.critical_samples == 0 {
            return Err(ConfigError::InvalidBatteryThreshold);
        }

        let s = &self.scales;
        for scale in [s.slow_percent, s.normal_percent, s.fast_percent] {
            if scale == 0 || scale > 100 {
                return Err(ConfigError::InvalidSpeedScale);
            }
        }
        if !(s.slow_percent <= s.normal_percent && s.normal_percent <= s.fast_percent) {
            return Err(ConfigError::InvalidSpeedScale);
        }

        if self.motors.min_duty_permille == 0
            || self.motors.min_duty_permille > self.motors.max_duty_permille
            || self.motors.max_duty_permille > 1000
        {
            return Err(ConfigError::InvalidDivider);
        }

        Ok(())
    }

    /// Scale factor (percent) of a speed mode.
    pub fn scale_percent(&self, mode: SpeedMode) -> u16 {
        match mode {
            SpeedMode::Slow => self.scales.slow_percent,
            SpeedMode::Normal => self.scales.normal_percent,
            SpeedMode::Fast => self.scales.fast_percent,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_valid() {
        assert_eq!(Config::default().validate(), Ok(()));
    }

    #[test]
    fn rejects_out_of_range_drive_value() {
        let mut c = Config::default();
        c.drive.forward = 1001;
        assert_eq!(c.validate(), Err(ConfigError::InvalidDriveValue("forward")));
    }

    #[test]
    fn rejects_inverted_turn_ratio() {
        let mut c = Config::default();
        c.drive.turn_inner = 900;
        c.drive.turn_outer = 100;
        assert_eq!(c.validate(), Err(ConfigError::InvalidTurnRatio));
    }

    #[test]
    fn rejects_too_short_command_timeout() {
        let mut c = Config::default();
        c.safety.command_timeout_ms = 10;
        assert_eq!(c.validate(), Err(ConfigError::InvalidCommandTimeout));
    }

    #[test]
    fn rejects_non_monotonic_speed_scales() {
        let mut c = Config::default();
        c.scales.slow_percent = 90;
        c.scales.normal_percent = 40;
        assert_eq!(c.validate(), Err(ConfigError::InvalidSpeedScale));
    }

    #[test]
    fn rejects_battery_thresholds_in_wrong_order() {
        let mut c = Config::default();
        c.battery.warn_mv_per_cell = 3300;
        c.battery.critical_mv_per_cell = 3400;
        assert_eq!(c.validate(), Err(ConfigError::InvalidBatteryThreshold));
    }

    #[test]
    fn battery_pack_thresholds_scale_with_cell_count() {
        let mut c = Config::default();
        c.battery.cell_count = 2;
        assert_eq!(c.battery.warn_mv(), 6800);
        assert_eq!(c.battery.critical_mv(), 6600);
    }
}
