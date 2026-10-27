//! Input resolution: operator button state -> normalised differential command.
//!
//! The controller app never sends motor values; it sends the *state of the
//! buttons*. All calibration lives here so the app and the firmware can never
//! disagree about what "Forward" means.
//!
//! Conflict rules (deterministic, documented, tested):
//!
//! | Input                | Result                                   |
//! |----------------------|------------------------------------------|
//! | STOP                 | stop, always wins                        |
//! | Forward + Backward   | stop (unsafe combination)                |
//! | Left + Right         | stop (unsafe combination)                |
//! | Forward (+ Left/Right) | turn with the configured turn ratio    |
//! | Backward (+ Left/Right) | reverse turn with the configured ratio |
//! | Forward / Backward   | straight forward / reverse               |
//! | Left / Right         | forward turn with the configured ratio   |

use crate::config::DriveTuning;
use crate::DriveCommand;

/// Button bitmask: forward.
pub const BTN_FORWARD: u8 = 1 << 0;
/// Button bitmask: backward.
pub const BTN_BACKWARD: u8 = 1 << 1;
/// Button bitmask: left.
pub const BTN_LEFT: u8 = 1 << 2;
/// Button bitmask: right.
pub const BTN_RIGHT: u8 = 1 << 3;
/// Button bitmask: stop (priority over everything else).
pub const BTN_STOP: u8 = 1 << 4;

/// Mask of all bits the firmware understands.
pub const BTN_MASK: u8 = BTN_FORWARD | BTN_BACKWARD | BTN_LEFT | BTN_RIGHT | BTN_STOP;

/// Outcome of resolving a button mask.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Resolved {
    /// Command to feed into the acceleration limiter.
    pub command: DriveCommand,
    /// True when the operator pressed a mutually exclusive combination.
    pub conflict: bool,
}

/// Resolves a button mask into a drive command.
///
/// `scale_percent` scales the whole command (speed modes) and is expected to
/// be in `1 ..= 100`.
pub fn resolve(mask: u8, tuning: &DriveTuning, scale_percent: u16) -> Resolved {
    // STOP always wins, even in combination with other buttons.
    if mask & BTN_STOP != 0 {
        return Resolved {
            command: DriveCommand::STOP,
            conflict: false,
        };
    }

    let forward = mask & BTN_FORWARD != 0;
    let backward = mask & BTN_BACKWARD != 0;
    let left = mask & BTN_LEFT != 0;
    let right = mask & BTN_RIGHT != 0;

    // Mutually exclusive combinations resolve to a full stop.
    if (forward && backward) || (left && right) {
        return Resolved {
            command: DriveCommand::STOP,
            conflict: true,
        };
    }

    let (left_mag, right_mag) = if forward {
        if left {
            (tuning.turn_inner, tuning.turn_outer)
        } else if right {
            (tuning.turn_outer, tuning.turn_inner)
        } else {
            (tuning.forward, tuning.forward)
        }
    } else if backward {
        if left {
            (tuning.reverse_turn_inner, tuning.reverse_turn_outer)
        } else if right {
            (tuning.reverse_turn_outer, tuning.reverse_turn_inner)
        } else {
            (tuning.reverse, tuning.reverse)
        }
    } else if left {
        (tuning.turn_inner, tuning.turn_outer)
    } else if right {
        (tuning.turn_outer, tuning.turn_inner)
    } else {
        (0, 0)
    };

    // Reverse commands are negative.
    let sign = if backward { -1 } else { 1 };
    let command = DriveCommand::new(
        scale(left_mag, scale_percent) * sign,
        scale(right_mag, scale_percent) * sign,
    );

    Resolved {
        command,
        conflict: false,
    }
}

/// Scales a magnitude by a percentage, rounding down.
fn scale(value: i16, percent: u16) -> i16 {
    let percent = percent.clamp(1, 100);
    ((value as i32 * percent as i32) / 100) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t() -> DriveTuning {
        DriveTuning::default()
    }

    #[test]
    fn forward_uses_calibration_table() {
        let r = resolve(BTN_FORWARD, &t(), 100);
        assert_eq!(r.command, DriveCommand::new(800, 800));
        assert!(!r.conflict);
    }

    #[test]
    fn backward_uses_calibration_table() {
        let r = resolve(BTN_BACKWARD, &t(), 100);
        assert_eq!(r.command, DriveCommand::new(-600, -600));
    }

    #[test]
    fn left_turn_uses_calibration_table() {
        let r = resolve(BTN_LEFT, &t(), 100);
        assert_eq!(r.command, DriveCommand::new(350, 800));
    }

    #[test]
    fn right_turn_uses_calibration_table() {
        let r = resolve(BTN_RIGHT, &t(), 100);
        assert_eq!(r.command, DriveCommand::new(800, 350));
    }

    #[test]
    fn stop_always_wins() {
        let r = resolve(BTN_STOP | BTN_FORWARD | BTN_LEFT, &t(), 100);
        assert_eq!(r.command, DriveCommand::STOP);
        assert!(!r.conflict);
    }

    #[test]
    fn forward_backward_conflict_stops() {
        let r = resolve(BTN_FORWARD | BTN_BACKWARD, &t(), 100);
        assert_eq!(r.command, DriveCommand::STOP);
        assert!(r.conflict);
    }

    #[test]
    fn left_right_conflict_stops() {
        let r = resolve(BTN_LEFT | BTN_RIGHT, &t(), 100);
        assert_eq!(r.command, DriveCommand::STOP);
        assert!(r.conflict);
    }

    #[test]
    fn forward_left_turns() {
        let r = resolve(BTN_FORWARD | BTN_LEFT, &t(), 100);
        assert_eq!(r.command, DriveCommand::new(350, 800));
    }

    #[test]
    fn backward_left_reverses_with_turn_ratio() {
        let r = resolve(BTN_BACKWARD | BTN_LEFT, &t(), 100);
        assert_eq!(r.command, DriveCommand::new(-350, -600));
    }

    #[test]
    fn backward_right_reverses_with_turn_ratio() {
        let r = resolve(BTN_BACKWARD | BTN_RIGHT, &t(), 100);
        assert_eq!(r.command, DriveCommand::new(-600, -350));
    }

    #[test]
    fn slow_mode_scales_everything() {
        let r = resolve(BTN_FORWARD, &t(), 50);
        assert_eq!(r.command, DriveCommand::new(400, 400));
    }

    #[test]
    fn empty_mask_stops() {
        let r = resolve(0, &t(), 100);
        assert_eq!(r.command, DriveCommand::STOP);
    }

    #[test]
    fn result_never_exceeds_command_max() {
        let mut tuning = t();
        tuning.forward = crate::COMMAND_MAX;
        let r = resolve(BTN_FORWARD, &tuning, 100);
        assert!(r.command.peak() <= crate::COMMAND_MAX);
    }
}
