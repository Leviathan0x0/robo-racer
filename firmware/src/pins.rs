//! Single source of truth for the GPIO map.
//!
//! Only the classic **ESP32 (WROOM-32)** map has been chosen so far. Every
//! other chip fails to compile instead of silently driving the wrong pins:
//! the pin map depends on the board, and the board has not been confirmed yet
//! (see `docs/HARDWARE_ASSUMPTIONS.md`).
//!
//! | Signal              | GPIO | Notes                                  |
//! |---------------------|------|----------------------------------------|
//! | Left driver RPWM    | 16   | LEDC channel 0                         |
//! | Left driver LPWM    | 17   | LEDC channel 1                         |
//! | Left driver R_EN    | 18   | driven low at boot (driver disabled)   |
//! | Left driver L_EN    | 19   | driven low at boot                     |
//! | Right driver RPWM   | 21   | LEDC channel 2                         |
//! | Right driver LPWM   | 22   | LEDC channel 3                         |
//! | Right driver R_EN   | 23   | driven low at boot                     |
//! | Right driver L_EN   | 25   | driven low at boot                     |
//! | Battery sense       | 34   | ADC1 channel 6, input only             |
//! | Status LED          | 2    | on-board LED, active high              |
//!
//! Pins avoided on purpose: 6-11 (SPI flash), 0/2/5/12/15 (boot strapping),
//! 1/3 (USB serial), 34-39 (input only, fine for the ADC but nothing else).
//! GPIO16/17 are unavailable on ESP32-WROVER modules because the PSRAM uses
//! them; use a WROOM-32 board.

#[cfg(not(feature = "esp32"))]
compile_error!(
    "GPIO map is confirmed for the ESP32 (WROOM-32) only. \
     Update firmware/src/pins.rs before building for another chip."
);

use esp_hal::gpio::AnyPin;

/// GPIO numbers, for the wiring documentation and the logs.
///
/// The firmware drives the pins through the peripherals struct, so these
/// constants exist to keep the table and `docs/WIRING.md` in sync.
#[allow(dead_code)]
pub mod gpio {
    /// Left driver RPWM.
    pub const LEFT_RPWM: u8 = 16;
    /// Left driver LPWM.
    pub const LEFT_LPWM: u8 = 17;
    /// Left driver right-enable.
    pub const LEFT_R_EN: u8 = 18;
    /// Left driver left-enable.
    pub const LEFT_L_EN: u8 = 19;
    /// Right driver RPWM.
    pub const RIGHT_RPWM: u8 = 21;
    /// Right driver LPWM.
    pub const RIGHT_LPWM: u8 = 22;
    /// Right driver right-enable.
    pub const RIGHT_R_EN: u8 = 23;
    /// Right driver left-enable.
    pub const RIGHT_L_EN: u8 = 25;
    /// Battery divider tap (ADC1 channel 6).
    pub const BATTERY_SENSE: u8 = 34;
    /// On-board status LED.
    pub const STATUS_LED: u8 = 2;
}

/// The motor-driver pins the firmware works with.
pub struct Board {
    /// Left driver RPWM.
    pub left_rpwm: AnyPin<'static>,
    /// Left driver LPWM.
    pub left_lpwm: AnyPin<'static>,
    /// Left driver right-enable.
    pub left_r_en: AnyPin<'static>,
    /// Left driver left-enable.
    pub left_l_en: AnyPin<'static>,
    /// Right driver RPWM.
    pub right_rpwm: AnyPin<'static>,
    /// Right driver LPWM.
    pub right_lpwm: AnyPin<'static>,
    /// Right driver right-enable.
    pub right_r_en: AnyPin<'static>,
    /// Right driver left-enable.
    pub right_l_en: AnyPin<'static>,
}

impl Board {
    /// Groups the eight motor-driver pins.
    ///
    /// The battery tap and the status LED are taken from the peripherals
    /// struct directly, because they do not belong to the motor driver.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        left_rpwm: impl Into<AnyPin<'static>>,
        left_lpwm: impl Into<AnyPin<'static>>,
        left_r_en: impl Into<AnyPin<'static>>,
        left_l_en: impl Into<AnyPin<'static>>,
        right_rpwm: impl Into<AnyPin<'static>>,
        right_lpwm: impl Into<AnyPin<'static>>,
        right_r_en: impl Into<AnyPin<'static>>,
        right_l_en: impl Into<AnyPin<'static>>,
    ) -> Self {
        Self {
            left_rpwm: left_rpwm.into(),
            left_lpwm: left_lpwm.into(),
            left_r_en: left_r_en.into(),
            left_l_en: left_l_en.into(),
            right_rpwm: right_rpwm.into(),
            right_lpwm: right_lpwm.into(),
            right_r_en: right_r_en.into(),
            right_l_en: right_l_en.into(),
        }
    }
}
