//! End-to-end tests over the public `robo-core` API.
//!
//! The unit tests inside each module check one rule at a time. These tests
//! chain the pieces the firmware chains, in the order it chains them, so a
//! regression in the wiring between modules is caught as well.

use robo_core::battery::BatteryMonitor;
use robo_core::config::{Config, SpeedMode};
use robo_core::protocol::{self, kind, status_flags, FrameDecoder, ProtocolError};
use robo_core::ramp::Ramp;
use robo_core::safety::{FaultKind, Output, RobotState, SafetyError, SafetySupervisor};
use robo_core::steering::{self, BTN_BACKWARD, BTN_FORWARD, BTN_LEFT, BTN_RIGHT, BTN_STOP};
use robo_core::DriveCommand;

/// Resolves a mask with the default calibration at full scale.
fn resolve(mask: u8) -> DriveCommand {
    steering::resolve(mask, &Config::default().drive, 100).command
}

/// Boots a supervisor that has a live link and a healthy battery.
fn ready_supervisor() -> SafetySupervisor {
    let mut supervisor = SafetySupervisor::new(Config::default().safety.clone());
    supervisor.boot_complete();
    supervisor.note_link(true);
    supervisor.note_battery(true);
    supervisor
}

#[test]
fn calibration_table_matches_the_brief() {
    assert_eq!(resolve(BTN_FORWARD), DriveCommand::new(800, 800));
    assert_eq!(resolve(BTN_BACKWARD), DriveCommand::new(-600, -600));
    assert_eq!(resolve(BTN_LEFT), DriveCommand::new(350, 800));
    assert_eq!(resolve(BTN_RIGHT), DriveCommand::new(800, 350));
}

#[test]
fn contradictory_buttons_stop_the_robot() {
    assert!(resolve(BTN_FORWARD | BTN_BACKWARD).is_stop());
    assert!(resolve(BTN_LEFT | BTN_RIGHT).is_stop());
    // STOP wins over any direction.
    assert!(resolve(BTN_STOP | BTN_FORWARD | BTN_LEFT).is_stop());
}

#[test]
fn full_safety_journey() {
    let cfg = Config::default();
    let mut supervisor = ready_supervisor();

    // Arming is explicit and only from DISARMED.
    assert_eq!(supervisor.state(), RobotState::Disarmed);
    assert_eq!(supervisor.arm(), Ok(()));
    assert_eq!(supervisor.arm(), Err(SafetyError::AlreadyArmed));

    // A command moves the robot.
    supervisor.note_command(1_000, DriveCommand::new(800, 800));
    assert_eq!(
        supervisor.update(1_000),
        Output::Move(DriveCommand::new(800, 800))
    );
    assert_eq!(supervisor.state(), RobotState::Driving);

    // Releasing the buttons disarms the output but keeps the robot armed.
    supervisor.note_command(1_020, DriveCommand::STOP);
    assert_eq!(supervisor.update(1_020), Output::Disabled);
    assert_eq!(supervisor.state(), RobotState::Armed);

    // A pause while stopped is not a fault, however long it lasts.
    let paused = 1_020 + 5_000;
    assert_eq!(supervisor.update(paused), Output::Disabled);
    assert_eq!(supervisor.state(), RobotState::Armed);
    assert_eq!(supervisor.fault(), None);

    // Silence longer than the command timeout is a fault.
    supervisor.note_command(paused, DriveCommand::new(800, 800));
    let late = paused + cfg.safety.command_timeout_ms + 1;
    assert_eq!(supervisor.update(late), Output::Disabled);
    assert_eq!(supervisor.fault(), Some(FaultKind::CommandTimeout));
    assert_eq!(
        supervisor.arm(),
        Err(SafetyError::FaultLatched(FaultKind::CommandTimeout))
    );

    // Only an explicit acknowledgement clears it, and only back to DISARMED.
    assert_eq!(supervisor.acknowledge_fault(), Ok(()));
    assert_eq!(supervisor.state(), RobotState::Disarmed);
    assert_eq!(supervisor.fault(), None);
    assert_eq!(supervisor.arm(), Ok(()));
}

#[test]
fn link_loss_is_a_fault_only_while_enabled() {
    // Losing the link while disarmed is not a fault.
    let mut supervisor = ready_supervisor();
    supervisor.note_link(false);
    assert_eq!(supervisor.state(), RobotState::Disarmed);
    assert_eq!(supervisor.fault(), None);

    // While armed it latches, and cannot be cleared before the link is back.
    let mut supervisor = ready_supervisor();
    supervisor.arm().unwrap();
    supervisor.note_link(false);
    assert_eq!(supervisor.state(), RobotState::Fault);
    assert_eq!(supervisor.fault(), Some(FaultKind::LinkLost));
    assert!(supervisor.acknowledge_fault().is_err());
    supervisor.note_link(true);
    assert_eq!(supervisor.acknowledge_fault(), Ok(()));
    assert_eq!(supervisor.state(), RobotState::Disarmed);
}

#[test]
fn emergency_stop_cannot_be_armed_away() {
    let mut supervisor = ready_supervisor();
    supervisor.arm().unwrap();
    supervisor.emergency_stop();
    assert_eq!(supervisor.fault(), Some(FaultKind::EmergencyStop));
    assert_eq!(
        supervisor.arm(),
        Err(SafetyError::FaultLatched(FaultKind::EmergencyStop))
    );
    assert_eq!(supervisor.update(0), Output::Disabled);
}

#[test]
fn ramp_respects_acceleration_limit() {
    let cfg = Config::default();
    let step_ms = cfg.safety.control_period_ms;
    let max_step = (cfg.drive.accel_per_s as i32 * step_ms as i32 / 1000) as i16;
    let target = DriveCommand::new(1_000, 1_000);
    let mut ramp = Ramp::new();
    let mut previous = ramp.current();
    let mut steps = 0;

    loop {
        let next = ramp.update(target, step_ms, &cfg.drive);
        let delta = (next.left - previous.left).abs();
        assert!(
            delta <= max_step,
            "acceleration exceeded: {delta} > {max_step}"
        );
        previous = next;
        steps += 1;
        if next == target {
            break;
        }
        assert!(steps < 1_000, "ramp never reached the target");
    }

    // Deceleration back to zero is also limited.
    let mut next = ramp.update(DriveCommand::STOP, step_ms, &cfg.drive);
    while next != DriveCommand::STOP {
        assert!(steps < 2_000);
        assert!(next.left >= 0, "deceleration crossed zero");
        next = ramp.update(DriveCommand::STOP, step_ms, &cfg.drive);
        steps += 1;
    }
}

#[test]
fn ramp_coasts_before_reversing() {
    let cfg = Config::default();
    let step_ms = 10;
    let mut ramp = Ramp::new();
    // Reach full forward first.
    for _ in 0..200 {
        ramp.update(DriveCommand::new(800, 800), step_ms, &cfg.drive);
    }
    assert_eq!(ramp.current(), DriveCommand::new(800, 800));

    // Ask for full reverse; the output must pass through zero, never jump.
    let mut crossed_after_ms = None;
    for elapsed in 0..1_000 {
        let output = ramp.update(DriveCommand::new(-600, -600), step_ms, &cfg.drive);
        if output.left < 0 {
            crossed_after_ms = Some(elapsed * step_ms);
            break;
        }
    }
    let crossed_after_ms = crossed_after_ms.expect("reverse never engaged");
    assert!(
        crossed_after_ms >= cfg.drive.direction_change_coast_ms as u32,
        "reversed after {crossed_after_ms} ms, coast window is {} ms",
        cfg.drive.direction_change_coast_ms
    );
}

#[test]
fn protocol_session_enforces_token_and_sequence() {
    let token = 0xDEAD_BEEF;
    let mut decoder = FrameDecoder::new(token);
    let mode = SpeedMode::Normal;

    let first = protocol::encode_client_frame(kind::DRIVE, BTN_FORWARD, mode, 0, 7, token);
    let decoded = decoder.decode(&first).expect("first frame accepted");
    assert_eq!(decoded.kind, kind::DRIVE);
    assert_eq!(decoded.buttons, BTN_FORWARD);
    assert_eq!(decoded.mode, mode);
    assert_eq!(decoded.seq, 7);

    // A replay of the same sequence number is stale.
    assert_eq!(
        decoder.decode(&first),
        Err(ProtocolError::StaleSequence(7))
    );

    // A frame from another session is rejected.
    let foreign = protocol::encode_client_frame(kind::DRIVE, BTN_FORWARD, mode, 0, 8, 0x1234);
    assert_eq!(decoder.decode(&foreign), Err(ProtocolError::BadToken));

    // Normal progress is accepted.
    let next = protocol::encode_client_frame(kind::DRIVE, BTN_LEFT, mode, 0, 8, token);
    assert!(decoder.decode(&next).is_ok());

    // Dropped buttons are accepted as a stop, not as a fault.
    let stop = protocol::encode_client_frame(kind::DRIVE, 0, mode, 0, 9, token);
    assert_eq!(decoder.decode(&stop).map(|f| f.buttons), Ok(0));
}

#[test]
fn status_and_hello_frames_carry_the_session() {
    let token = 0x0BAD_F00D;
    let hello = protocol::encode_hello(token, 12_345);
    assert_eq!(hello.len(), protocol::HELLO_FRAME_LEN);
    assert_eq!(hello[0], protocol::MAGIC);
    assert_eq!(hello[2], protocol::server_kind::HELLO);
    assert_eq!(u32::from_le_bytes(hello[4..8].try_into().unwrap()), token);
    assert_eq!(
        u32::from_le_bytes(hello[8..12].try_into().unwrap()),
        12_345
    );

    let status = protocol::encode_status(
        RobotState::Armed,
        None,
        status_flags::ARMED | status_flags::LINK_UP | status_flags::SESSION_OK,
        11_400,
        350,
        -600,
        42,
        SpeedMode::Slow,
    );
    assert_eq!(status.len(), protocol::STATUS_FRAME_LEN);
    assert_eq!(status[0], protocol::MAGIC);
    assert_eq!(status[2], protocol::server_kind::STATUS);
    assert_eq!(status[3], RobotState::Armed as u8);
    assert_eq!(u16::from_le_bytes(status[6..8].try_into().unwrap()), 11_400);
    assert_eq!(i16::from_le_bytes(status[8..10].try_into().unwrap()), 350);
    assert_eq!(i16::from_le_bytes(status[10..12].try_into().unwrap()), -600);
    assert_eq!(u16::from_le_bytes(status[12..14].try_into().unwrap()), 42);
}

#[test]
fn battery_fault_needs_consecutive_critical_samples() {
    let cfg = Config::default().battery;
    let mut monitor = BatteryMonitor::new();

    // 2.0 V at the divider pin is below the critical pack voltage.
    let critical_adc_mv = 2_000;
    for _ in 0..cfg.critical_samples.saturating_sub(1) {
        monitor.update(critical_adc_mv, &cfg);
        assert!(!monitor.critical_confirmed(&cfg));
    }
    monitor.update(critical_adc_mv, &cfg);
    assert!(monitor.critical_confirmed(&cfg));

    // One healthy sample clears the streak.
    monitor.update(2_600, &cfg);
    assert!(!monitor.critical_confirmed(&cfg));
    assert!(monitor.pack_mv().unwrap() > cfg.critical_mv());
}

#[test]
fn default_configuration_is_safe_to_flash() {
    let cfg = Config::default();
    assert_eq!(cfg.validate(), Ok(()));
    // The safety envelope the docs promise.
    assert!((200..=300).contains(&cfg.safety.command_timeout_ms));
    assert!((1_000..=3_000).contains(&cfg.safety.link_timeout_ms));
    assert_eq!(cfg.safety.control_period_ms, 10);
    assert!(cfg.battery.critical_mv() < cfg.battery.warn_mv());
    assert!(cfg.battery.warn_mv() < cfg.battery.cell_count as u32 * 4_200);
    // The duty window must be usable: above the motor's stiction threshold
    // and never above a full period.
    assert!(cfg.motors.min_duty_permille > 0);
    assert!(cfg.motors.min_duty_permille < cfg.motors.max_duty_permille);
    assert!(cfg.motors.max_duty_permille <= 1_000);
}
