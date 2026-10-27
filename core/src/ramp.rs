//! Acceleration limiting, braking and safe direction changes.
//!
//! The ramp sits between the *requested* command (from the operator) and the
//! *applied* command (to the motor driver). It guarantees:
//!
//! * the applied magnitude changes by at most `accel_per_s` / `decel_per_s`
//!   normalised units per second,
//! * a reversal first coasts to a full stop for `direction_change_coast_ms`
//!   before the opposite direction is enabled,
//! * the output is always within `-1000 ..= 1000`.

use crate::config::DriveTuning;
use crate::DriveCommand;

/// Acceleration limiter state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Ramp {
    /// Currently applied command.
    current: DriveCommand,
    /// Remaining coast time before a reversal may start (ms).
    coast_remaining_ms: u32,
}

impl Ramp {
    /// Creates a ramp that starts from a full stop.
    pub const fn new() -> Self {
        Self {
            current: DriveCommand::STOP,
            coast_remaining_ms: 0,
        }
    }

    /// Currently applied command.
    pub fn current(&self) -> DriveCommand {
        self.current
    }

    /// Remaining coast time before a reversal may start (ms).
    pub fn coast_remaining_ms(&self) -> u32 {
        self.coast_remaining_ms
    }

    /// True while a direction reversal is still coasting.
    pub fn is_coasting(&self) -> bool {
        self.coast_remaining_ms > 0
    }

    /// Forces the ramp to a full stop, clearing any pending coast time.
    pub fn reset(&mut self) {
        self.current = DriveCommand::STOP;
        self.coast_remaining_ms = 0;
    }

    /// Advances the limiter by `dt_ms` towards `target`.
    ///
    /// Returns the command that may be applied to the motor drivers.
    pub fn update(&mut self, target: DriveCommand, dt_ms: u32, tuning: &DriveTuning) -> DriveCommand {
        if dt_ms == 0 {
            return self.current;
        }

        // Direction reversal: coast to a stop first, never brake through zero
        // by driving the opposite direction.
        if target.left != 0
            && self.current.left != 0
            && (target.left < 0) != (self.current.left < 0)
        {
            self.current.left = 0;
            self.coast_remaining_ms = tuning.direction_change_coast_ms as u32;
        }
        if target.right != 0
            && self.current.right != 0
            && (target.right < 0) != (self.current.right < 0)
        {
            self.current.right = 0;
            self.coast_remaining_ms = tuning.direction_change_coast_ms as u32;
        }

        if self.coast_remaining_ms > 0 {
            self.coast_remaining_ms = self.coast_remaining_ms.saturating_sub(dt_ms);
            self.current = DriveCommand::STOP;
            return self.current;
        }

        self.current.left = approach(self.current.left, target.left, dt_ms, tuning);
        self.current.right = approach(self.current.right, target.right, dt_ms, tuning);
        self.current
    }
}

/// Moves `current` one step towards `target` honouring the acceleration and
/// deceleration limits.
fn approach(current: i16, target: i16, dt_ms: u32, tuning: &DriveTuning) -> i16 {
    if current == target {
        return target;
    }
    let cur_mag = current.unsigned_abs();
    let target_mag = target.unsigned_abs();

    let (rate, growing) = if target_mag > cur_mag {
        (tuning.accel_per_s, true)
    } else {
        (tuning.decel_per_s, false)
    };

    // Integer step, at least one unit so a very slow rate still converges.
    let step = ((rate as i32 * dt_ms as i32) / 1000).max(1) as u16;
    let new_mag = if growing {
        cur_mag.saturating_add(step).min(target_mag)
    } else {
        cur_mag.saturating_sub(step).max(target_mag)
    };

    let sign = if target < 0 { -1i16 } else { 1i16 };
    sign * new_mag.min(crate::COMMAND_MAX as u16) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t() -> DriveTuning {
        DriveTuning::default()
    }

    #[test]
    fn ramps_up_at_configured_rate() {
        let mut r = Ramp::new();
        let out = r.update(DriveCommand::new(800, 800), 10, &t());
        // accel_per_s = 2000 -> 20 units per 10 ms tick
        assert_eq!(out, DriveCommand::new(20, 20));
    }

    #[test]
    fn never_exceeds_target() {
        let mut r = Ramp::new();
        let mut out = DriveCommand::STOP;
        for _ in 0..200 {
            out = r.update(DriveCommand::new(800, 800), 10, &t());
        }
        assert_eq!(out, DriveCommand::new(800, 800));
    }

    #[test]
    fn brakes_faster_than_it_accelerates() {
        let mut r = Ramp::new();
        for _ in 0..100 {
            r.update(DriveCommand::new(800, 800), 10, &t());
        }
        let out = r.update(DriveCommand::STOP, 10, &t());
        // decel_per_s = 4000 -> 40 units per tick
        assert_eq!(out, DriveCommand::new(760, 760));
    }

    #[test]
    fn reversal_coasts_before_changing_direction() {
        let mut r = Ramp::new();
        for _ in 0..100 {
            r.update(DriveCommand::new(800, 800), 10, &t());
        }
        // Request reverse: the first tick must produce zero, not negative.
        let out = r.update(DriveCommand::new(-600, -600), 10, &t());
        assert_eq!(out, DriveCommand::STOP);
        assert!(r.is_coasting());

        // Coast for the configured 60 ms.
        for _ in 0..5 {
            let out = r.update(DriveCommand::new(-600, -600), 10, &t());
            assert_eq!(out, DriveCommand::STOP);
        }
        // Now the ramp may build up in the opposite direction.
        let out = r.update(DriveCommand::new(-600, -600), 10, &t());
        assert!(out.left < 0 && out.right < 0);
    }

    #[test]
    fn coast_expires_and_reverse_builds_up() {
        let mut r = Ramp::new();
        for _ in 0..100 {
            r.update(DriveCommand::new(800, 800), 10, &t());
        }
        // The coast lasts exactly `direction_change_coast_ms` (60 ms = 6 ticks of
        // zero output); the tick that detects the reversal is the first of the
        // seven updates below.
        let mut ticks = 0;
        let mut out = DriveCommand::STOP;
        while out.is_stop() && ticks < 20 {
            out = r.update(DriveCommand::new(-600, -600), 10, &t());
            ticks += 1;
        }
        assert_eq!(ticks, 7, "coast must last 60 ms (6 ticks) plus the reversing tick");
        assert!(out.left < 0 && out.right < 0);
        assert!(!r.is_coasting());
    }

    #[test]
    fn zero_dt_is_a_no_op() {
        let mut r = Ramp::new();
        let out = r.update(DriveCommand::new(800, 800), 0, &t());
        assert_eq!(out, DriveCommand::STOP);
    }

    #[test]
    fn output_always_within_bounds() {
        let mut r = Ramp::new();
        let targets = [
            DriveCommand::new(1000, 1000),
            DriveCommand::new(-1000, -1000),
            DriveCommand::new(-1000, 1000),
            DriveCommand::STOP,
        ];
        for t in targets {
            for _ in 0..50 {
                let out = r.update(t, 10, &tuning());
                assert!(out.left.abs() <= crate::COMMAND_MAX);
                assert!(out.right.abs() <= crate::COMMAND_MAX);
            }
        }
        fn tuning() -> DriveTuning {
            DriveTuning::default()
        }
    }
}
