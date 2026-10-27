//! Hardware-independent control logic for the robo racer.
//!
//! Everything in this crate is `no_std`-friendly and contains **no** hardware
//! access, no I/O and no blocking calls, so it can be exercised exhaustively on
//! a host machine (`cargo test -p robo-core`) and reused unchanged by the
//! ESP32 firmware.
//!
//! Module responsibilities:
//!
//! * [`config`]   – central, validated configuration (single source of truth).
//! * [`steering`] – input bitmask -> normalised differential drive command.
//! * [`ramp`]     – acceleration / deceleration / direction-change limiting.
//! * [`safety`]   – robot state machine, command timeout, fault latching.
//! * [`protocol`] – binary wire format (encode / decode / validate).
//! * [`battery`]  – battery voltage filtering and threshold classification.
//! * [`websocket`] – RFC 6455 handshake and frame codec.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod battery;
pub mod config;
pub mod protocol;
pub mod ramp;
pub mod safety;
pub mod steering;
pub mod websocket;

/// Normalised command range: `-1000 ..= +1000`.
pub const COMMAND_MAX: i16 = 1000;

/// Signed differential drive command, one value per motor side.
///
/// The values are *normalised requests*, not measured wheel speeds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DriveCommand {
    /// Left side (two motors wired to driver A).
    pub left: i16,
    /// Right side (two motors wired to driver B).
    pub right: i16,
}

impl DriveCommand {
    /// A command that requests no motion.
    pub const STOP: DriveCommand = DriveCommand { left: 0, right: 0 };

    /// Builds a command, clamping both sides into `-1000 ..= 1000`.
    pub fn new(left: i16, right: i16) -> Self {
        Self {
            left: left.clamp(-COMMAND_MAX, COMMAND_MAX),
            right: right.clamp(-COMMAND_MAX, COMMAND_MAX),
        }
    }

    /// True when neither side is requested to move.
    pub fn is_stop(&self) -> bool {
        self.left == 0 && self.right == 0
    }

    /// Largest absolute side value, used for telemetry.
    pub fn peak(&self) -> i16 {
        self.left.abs().max(self.right.abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_is_clamped() {
        let c = DriveCommand::new(5000, -9000);
        assert_eq!(c.left, COMMAND_MAX);
        assert_eq!(c.right, -COMMAND_MAX);
    }

    #[test]
    fn stop_is_stop() {
        assert!(DriveCommand::STOP.is_stop());
        assert!(!DriveCommand::new(1, 0).is_stop());
    }
}
