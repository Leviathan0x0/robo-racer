//! Battery voltage measurement, filtering and threshold classification.
//!
//! The ESP32 ADC can only measure a divided pack voltage, so the raw reading
//! is scaled back to pack voltage, filtered with an exponential moving average
//! and finally classified.
//!
//! Voltage alone is **not** a state-of-charge indicator while the motors draw
//! current; the reading sags during acceleration and skid turns. The monitor
//! therefore only uses the voltage for *safety* thresholds, never for a
//! percentage display.

use crate::config::BatteryConfig;

/// Classification of the current battery voltage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BatteryLevel {
    /// No measurement yet.
    #[default]
    Unknown,
    /// Above the warning threshold.
    Ok,
    /// Below the warning threshold but above the critical threshold.
    Warning,
    /// Below the critical threshold.
    Critical,
}

impl BatteryLevel {
    /// True when the level is critical.
    pub fn is_critical(&self) -> bool {
        *self == BatteryLevel::Critical
    }
}

/// Battery monitor with an exponential moving average filter.
#[derive(Debug, Clone, Copy, Default)]
pub struct BatteryMonitor {
    /// Filtered pack voltage (mV).
    filtered_mv: Option<u32>,
    /// Consecutive critical samples seen so far.
    critical_streak: u8,
}

impl BatteryMonitor {
    /// Creates a monitor without any measurement.
    pub const fn new() -> Self {
        Self {
            filtered_mv: None,
            critical_streak: 0,
        }
    }

    /// Latest filtered pack voltage (mV).
    pub fn pack_mv(&self) -> Option<u32> {
        self.filtered_mv
    }

    /// Number of consecutive critical samples.
    pub fn critical_streak(&self) -> u8 {
        self.critical_streak
    }

    /// Feeds a new ADC reading (millivolts at the ADC pin) and returns the
    /// resulting classification.
    pub fn update(&mut self, adc_mv: u32, cfg: &BatteryConfig) -> BatteryLevel {
        let pack_mv = adc_to_pack_mv(adc_mv, cfg);
        self.filtered_mv = Some(match self.filtered_mv {
            None => pack_mv,
            Some(previous) => {
                let alpha = cfg.ema_alpha_percent as u64;
                let value = previous as u64;
                let new_value = pack_mv as u64;
                // filtered = old + alpha * (new - old)
                let delta = (new_value as i64 - value as i64) as i64;
                let stepped = value as i64 + (delta * alpha as i64) / 100;
                stepped.clamp(0, u32::MAX as i64) as u32
            }
        });

        let level = self.classify(self.filtered_mv.unwrap_or(0), cfg);
        if level.is_critical() {
            self.critical_streak = self.critical_streak.saturating_add(1);
        } else {
            self.critical_streak = 0;
        }
        level
    }

    /// True once the critical threshold has been seen `cfg.critical_samples`
    /// times in a row (debounce against a single voltage spike).
    pub fn critical_confirmed(&self, cfg: &BatteryConfig) -> bool {
        self.critical_streak >= cfg.critical_samples
    }

    /// Classifies a pack voltage.
    pub fn classify(&self, pack_mv: u32, cfg: &BatteryConfig) -> BatteryLevel {
        if pack_mv < cfg.critical_mv() {
            BatteryLevel::Critical
        } else if pack_mv < cfg.warn_mv() {
            BatteryLevel::Warning
        } else {
            BatteryLevel::Ok
        }
    }
}

/// Converts an ADC reading into a pack voltage (mV).
pub fn adc_to_pack_mv(adc_mv: u32, cfg: &BatteryConfig) -> u32 {
    let scaled = (adc_mv as u64 * cfg.divider_num as u64) / cfg.divider_den as u64;
    let scaled = scaled as i64 + cfg.calibration_offset_mv as i64;
    scaled.clamp(0, u32::MAX as i64) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> BatteryConfig {
        BatteryConfig {
            cell_count: 3,
            warn_mv_per_cell: 3400,
            critical_mv_per_cell: 3300,
            divider_num: 1000,
            divider_den: 1000,
            calibration_offset_mv: 0,
            ema_alpha_percent: 100,
            critical_samples: 3,
        }
    }

    #[test]
    fn scales_the_divider_back_to_pack_voltage() {
        let mut c = cfg();
        c.divider_num = 127;
        c.divider_den = 27;
        // 2.7 V at the pin -> 12.7 V at the pack
        assert_eq!(adc_to_pack_mv(2700, &c), 12700);
    }

    #[test]
    fn classifies_thresholds() {
        let m = BatteryMonitor::new();
        // Exactly at the warning threshold is still "Ok"; the warning starts
        // *below* the threshold.
        assert_eq!(m.classify(12000, &cfg()), BatteryLevel::Ok);
        assert_eq!(m.classify(10200, &cfg()), BatteryLevel::Ok);
        assert_eq!(m.classify(10199, &cfg()), BatteryLevel::Warning);
        assert_eq!(m.classify(9900, &cfg()), BatteryLevel::Warning);
        assert_eq!(m.classify(9899, &cfg()), BatteryLevel::Critical);
    }

    #[test]
    fn critical_needs_consecutive_samples() {
        let mut c = cfg();
        c.ema_alpha_percent = 100;
        let mut m = BatteryMonitor::new();
        m.update(9000, &c);
        assert_eq!(m.critical_streak(), 1);
        assert!(!m.critical_confirmed(&c));
        m.update(9000, &c);
        m.update(9000, &c);
        assert!(m.critical_confirmed(&c));
    }

    #[test]
    fn a_single_spike_does_not_trip_the_fault() {
        let mut c = cfg();
        c.ema_alpha_percent = 100;
        let mut m = BatteryMonitor::new();
        m.update(12000, &c);
        m.update(9000, &c);
        assert!(!m.critical_confirmed(&c));
        m.update(12000, &c);
        assert_eq!(m.critical_streak(), 0);
    }

    #[test]
    fn filter_smooths_the_reading() {
        let mut c = cfg();
        c.ema_alpha_percent = 50;
        let mut m = BatteryMonitor::new();
        m.update(10000, &c);
        m.update(12000, &c);
        let filtered = m.pack_mv().unwrap();
        assert!(filtered > 10000 && filtered < 12000);
    }
}
