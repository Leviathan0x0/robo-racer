//! Battery voltage sensing through a resistive divider.
//!
//! The ADC returns raw 12-bit counts (the default `()` calibration scheme of
//! esp-hal does not convert to millivolts), so the conversion lives here and is
//! calibrated in `docs/CALIBRATION.md`.
//!
//! With the 11 dB attenuation the ESP32 input range is roughly 0-3.1 V. The
//! divider (100k / 27k) maps a 12.6 V pack to 2.68 V, which leaves headroom at
//! the top of the range.

use esp_hal::analog::adc::{Adc, AdcChannel, AdcConfig, AdcPin, Attenuation};
use esp_hal::gpio::AnalogPin;
use esp_hal::peripherals::ADC1;
use esp_hal::Blocking;

/// Millivolts corresponding to a full-scale (4095 count) reading.
///
/// PROVISIONAL: 3100 mV is the nominal ESP32 11 dB range. Replace it with the
/// value measured during two-point calibration.
const ADC_FULL_SCALE_MV: u32 = 3100;
/// ESP32 SAR ADC is 12 bit.
const ADC_MAX_COUNTS: u32 = 4095;
/// Upper bound on conversion polls; a real conversion needs a handful.
const ADC_POLL_LIMIT: u32 = 10_000;

/// Battery voltage sensor.
pub struct BatterySensor<P> {
    adc: Adc<'static, ADC1<'static>, Blocking>,
    pin: AdcPin<P, ADC1<'static>>,
}

impl<P: AdcChannel + AnalogPin> BatterySensor<P> {
    /// Configures ADC1 on the battery divider tap.
    pub fn new(adc: ADC1<'static>, pin: P) -> Self {
        let mut config = AdcConfig::new();
        let pin = config.enable_pin(pin, Attenuation::_11dB);
        Self {
            adc: Adc::new(adc, config),
            pin,
        }
    }

    /// Reads the divider voltage in millivolts.
    ///
    /// The ESP32 ADC driver is non-blocking: it returns `WouldBlock` while a
    /// conversion is running. The conversion takes microseconds, so a bounded
    /// poll is cheaper than driving the state machine from the control loop.
    /// Called five times per second, it cannot disturb the 100 Hz loop.
    pub fn read_mv(&mut self) -> Option<u32> {
        for _ in 0..ADC_POLL_LIMIT {
            if let Ok(raw) = self.adc.read_oneshot(&mut self.pin) {
                return Some((raw as u32 * ADC_FULL_SCALE_MV) / ADC_MAX_COUNTS);
            }
        }
        None
    }
}
