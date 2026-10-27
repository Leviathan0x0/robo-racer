//! Safety supervisor: robot state machine, command watchdog and fault latching.
//!
//! The supervisor is completely independent of the network stack: it only ever
//! consumes *facts* (a valid command arrived, the link is up, the battery is
//! healthy) and produces a single decision per call - either "motors disabled"
//! or "apply this command".
//!
//! ```text
//! BOOT --boot_complete--> DISARMED --arm--> ARMED --valid command--> DRIVING
//!   ^                        ^                |                        |
//!   |                        |        release command                 |
//!   |                        +----------------+------------------------+
//!   |                                                                  |
//!   |                        any critical fault                        |
//!   |                                                                  v
//!   +---------------------- acknowledge_fault <---------------------- FAULT
//! ```

use crate::config::SafetyTuning;
use crate::DriveCommand;

/// Robot state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RobotState {
    /// Start-up, hardware not initialised yet.
    #[default]
    Boot,
    /// Initialised and waiting for an explicit arm.
    Disarmed,
    /// Armed but not moving.
    Armed,
    /// Armed and executing a valid command.
    Driving,
    /// Latched fault, motors disabled until acknowledged.
    Fault,
}

/// Reason the robot latched a fault.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultKind {
    /// No valid command arrived within the configured timeout.
    CommandTimeout,
    /// The control link was lost.
    LinkLost,
    /// Battery voltage fell below the critical threshold.
    LowBattery,
    /// Motor driver reported a fault.
    DriverFault,
    /// The operator pressed the in-app emergency stop.
    EmergencyStop,
    /// An internal firmware error.
    Internal,
}

impl FaultKind {
    /// Short, human readable description used by the app and the logs.
    pub fn as_str(&self) -> &'static str {
        match self {
            FaultKind::CommandTimeout => "command timeout",
            FaultKind::LinkLost => "control link lost",
            FaultKind::LowBattery => "battery critical",
            FaultKind::DriverFault => "motor driver fault",
            FaultKind::EmergencyStop => "emergency stop",
            FaultKind::Internal => "internal fault",
        }
    }
}

/// Decision produced by the supervisor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Output {
    /// Motor outputs must be disabled.
    Disabled,
    /// Apply this command.
    Move(DriveCommand),
}

/// Why an operation was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyError {
    /// The robot is still in the boot state.
    NotInitialised,
    /// A fault is latched and must be acknowledged first.
    FaultLatched(FaultKind),
    /// The robot is already armed.
    AlreadyArmed,
    /// The robot is not armed.
    NotArmed,
}

/// Safety supervisor.
#[derive(Debug, Clone)]
pub struct SafetySupervisor {
    state: RobotState,
    fault: Option<FaultKind>,
    last_command: DriveCommand,
    last_command_ms: Option<u32>,
    link_up: bool,
    battery_ok: bool,
    tuning: SafetyTuning,
}

impl SafetySupervisor {
    /// Creates a supervisor in the [`RobotState::Boot`] state.
    pub fn new(tuning: SafetyTuning) -> Self {
        Self {
            state: RobotState::Boot,
            fault: None,
            last_command: DriveCommand::STOP,
            last_command_ms: None,
            link_up: false,
            battery_ok: true,
            tuning,
        }
    }

    /// Current state.
    pub fn state(&self) -> RobotState {
        self.state
    }

    /// Latched fault, if any.
    pub fn fault(&self) -> Option<FaultKind> {
        self.fault
    }

    /// True when the motors may be driven.
    pub fn is_enabled(&self) -> bool {
        matches!(self.state, RobotState::Armed | RobotState::Driving)
    }

    /// Marks the hardware initialisation as complete.
    pub fn boot_complete(&mut self) {
        if self.state == RobotState::Boot {
            self.state = RobotState::Disarmed;
        }
    }

    /// Arms the robot.
    pub fn arm(&mut self) -> Result<(), SafetyError> {
        match self.state {
            RobotState::Boot => Err(SafetyError::NotInitialised),
            RobotState::Fault => Err(SafetyError::FaultLatched(
                self.fault.unwrap_or(FaultKind::Internal),
            )),
            RobotState::Armed | RobotState::Driving => Err(SafetyError::AlreadyArmed),
            RobotState::Disarmed => {
                self.state = RobotState::Armed;
                self.last_command = DriveCommand::STOP;
                self.last_command_ms = None;
                Ok(())
            }
        }
    }

    /// Disarms the robot; motors are disabled immediately.
    pub fn disarm(&mut self) {
        self.last_command = DriveCommand::STOP;
        self.last_command_ms = None;
        if self.state != RobotState::Boot {
            self.state = RobotState::Disarmed;
        }
    }

    /// Acknowledges a latched fault.
    ///
    /// Only succeeds when the cause has disappeared (link up again, battery
    /// healthy again). The robot then returns to [`RobotState::Disarmed`] and
    /// must be armed again explicitly.
    pub fn acknowledge_fault(&mut self) -> Result<(), SafetyError> {
        if self.state != RobotState::Fault {
            return Ok(());
        }
        if !self.link_up {
            return Err(SafetyError::FaultLatched(FaultKind::LinkLost));
        }
        if !self.battery_ok {
            return Err(SafetyError::FaultLatched(FaultKind::LowBattery));
        }
        self.fault = None;
        self.state = RobotState::Disarmed;
        self.last_command = DriveCommand::STOP;
        self.last_command_ms = None;
        Ok(())
    }

    /// Reports the state of the control link.
    pub fn note_link(&mut self, up: bool) {
        if self.link_up && !up && self.is_enabled() {
            self.latch(FaultKind::LinkLost);
        }
        self.link_up = up;
    }

    /// Reports whether the battery is above the critical threshold.
    pub fn note_battery(&mut self, ok: bool) {
        if !ok && self.is_enabled() {
            self.latch(FaultKind::LowBattery);
        }
        self.battery_ok = ok;
    }

    /// Latches an arbitrary critical fault.
    pub fn latch(&mut self, kind: FaultKind) {
        self.fault = Some(kind);
        self.state = RobotState::Fault;
        self.last_command = DriveCommand::STOP;
        self.last_command_ms = None;
    }

    /// Latches the in-app emergency stop.
    ///
    /// Distinct from an ordinary release because it stays latched: the operator
    /// must acknowledge it and arm again before the robot can move.
    pub fn emergency_stop(&mut self) {
        self.latch(FaultKind::EmergencyStop);
    }

    /// Records a valid command received at `now_ms`.
    ///
    /// Commands are only accepted while the robot is armed; they never arm it.
    pub fn note_command(&mut self, now_ms: u32, command: DriveCommand) {
        if !self.is_enabled() {
            return;
        }
        self.last_command = command;
        self.last_command_ms = Some(now_ms);
        self.state = if command.is_stop() {
            RobotState::Armed
        } else {
            RobotState::Driving
        };
    }

    /// Age of the last valid command in milliseconds (`None` if there is none).
    pub fn command_age_ms(&self, now_ms: u32) -> Option<u32> {
        self.last_command_ms
            .map(|t| now_ms.wrapping_sub(t))
    }

    /// Evaluates the supervisor and returns the decision for this cycle.
    pub fn update(&mut self, now_ms: u32) -> Output {
        // Only a robot that is being asked to move can run away. A stopped
        // robot stays armed: the operator is allowed to pause.
        if self.is_enabled() && !self.last_command.is_stop() {
            if let Some(age) = self.command_age_ms(now_ms) {
                if age > self.tuning.command_timeout_ms {
                    self.latch(FaultKind::CommandTimeout);
                }
            }
        }

        match self.state {
            RobotState::Armed | RobotState::Driving => {
                if self.last_command.is_stop() {
                    Output::Disabled
                } else {
                    Output::Move(self.last_command)
                }
            }
            _ => Output::Disabled,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn supervisor() -> SafetySupervisor {
        let mut s = SafetySupervisor::new(SafetyTuning::default());
        s.boot_complete();
        s
    }

    fn armed() -> SafetySupervisor {
        let mut s = supervisor();
        s.arm().unwrap();
        s
    }

    #[test]
    fn boot_keeps_motors_disabled() {
        let mut s = SafetySupervisor::new(SafetyTuning::default());
        assert_eq!(s.state(), RobotState::Boot);
        assert_eq!(s.update(0), Output::Disabled);
        assert_eq!(s.arm(), Err(SafetyError::NotInitialised));
    }

    #[test]
    fn disarmed_never_moves() {
        let mut s = supervisor();
        s.note_command(0, DriveCommand::new(800, 800));
        assert_eq!(s.update(0), Output::Disabled);
    }

    #[test]
    fn arming_is_required_before_moving() {
        let mut s = armed();
        s.note_command(10, DriveCommand::new(800, 800));
        assert_eq!(s.state(), RobotState::Driving);
        assert_eq!(s.update(10), Output::Move(DriveCommand::new(800, 800)));
    }

    #[test]
    fn releasing_the_controls_stops() {
        let mut s = armed();
        s.note_command(10, DriveCommand::new(800, 800));
        assert_eq!(s.update(10), Output::Move(DriveCommand::new(800, 800)));
        s.note_command(20, DriveCommand::STOP);
        assert_eq!(s.state(), RobotState::Armed);
        assert_eq!(s.update(20), Output::Disabled);
    }

    #[test]
    fn a_stopped_robot_does_not_time_out() {
        let mut s = armed();
        s.note_command(10, DriveCommand::new(800, 800));
        s.note_command(20, DriveCommand::STOP);
        // Ten seconds of silence while stopped is a pause, not a fault.
        assert_eq!(s.update(10_020), Output::Disabled);
        assert_eq!(s.state(), RobotState::Armed);
        assert_eq!(s.fault(), None);
    }

    #[test]
    fn command_timeout_latches_a_fault() {        let mut s = armed();
        s.note_command(0, DriveCommand::new(800, 800));
        // timeout is 250 ms
        assert_eq!(s.update(200), Output::Move(DriveCommand::new(800, 800)));
        assert_eq!(s.update(300), Output::Disabled);
        assert_eq!(s.state(), RobotState::Fault);
        assert_eq!(s.fault(), Some(FaultKind::CommandTimeout));
    }

    #[test]
    fn fault_requires_acknowledgement_and_rearm() {
        let mut s = armed();
        s.note_command(0, DriveCommand::new(800, 800));
        s.update(1000);
        assert_eq!(s.state(), RobotState::Fault);
        assert_eq!(s.arm(), Err(SafetyError::FaultLatched(FaultKind::CommandTimeout)));
        // Link is up and battery ok, so the fault can be acknowledged.
        s.note_link(true);
        assert_eq!(s.acknowledge_fault(), Ok(()));
        assert_eq!(s.state(), RobotState::Disarmed);
        assert_eq!(s.arm(), Ok(()));
    }

    #[test]
    fn link_loss_latches_a_fault() {
        let mut s = armed();
        s.note_link(true);
        s.note_link(false);
        assert_eq!(s.state(), RobotState::Fault);
        assert_eq!(s.fault(), Some(FaultKind::LinkLost));
        assert_eq!(s.update(0), Output::Disabled);
        // Acknowledging fails while the link is still down.
        assert_eq!(
            s.acknowledge_fault(),
            Err(SafetyError::FaultLatched(FaultKind::LinkLost))
        );
    }

    #[test]
    fn low_battery_latches_a_fault() {
        let mut s = armed();
        s.note_link(true);
        s.note_battery(false);
        assert_eq!(s.state(), RobotState::Fault);
        assert_eq!(s.fault(), Some(FaultKind::LowBattery));
    }

    #[test]
    fn fault_state_never_produces_output() {
        let mut s = armed();
        s.latch(FaultKind::DriverFault);
        s.note_command(0, DriveCommand::new(800, 800));
        assert_eq!(s.update(0), Output::Disabled);
    }

    #[test]
    fn disarm_stops_immediately() {
        let mut s = armed();
        s.note_command(0, DriveCommand::new(800, 800));
        s.disarm();
        assert_eq!(s.update(0), Output::Disabled);
        assert_eq!(s.state(), RobotState::Disarmed);
    }

    #[test]
    fn emergency_stop_stays_latched_until_acknowledged() {
        let mut s = armed();
        s.note_link(true);
        s.note_command(0, DriveCommand::new(800, 800));
        assert_eq!(s.update(0), Output::Move(DriveCommand::new(800, 800)));

        s.emergency_stop();
        assert_eq!(s.state(), RobotState::Fault);
        assert_eq!(s.fault(), Some(FaultKind::EmergencyStop));
        assert_eq!(s.update(10), Output::Disabled);
        // Releasing the buttons must not clear it, arming must not either.
        s.note_command(20, DriveCommand::STOP);
        assert_eq!(s.arm(), Err(SafetyError::FaultLatched(FaultKind::EmergencyStop)));
        assert_eq!(s.update(20), Output::Disabled);

        assert_eq!(s.acknowledge_fault(), Ok(()));
        assert_eq!(s.state(), RobotState::Disarmed);
    }
}
