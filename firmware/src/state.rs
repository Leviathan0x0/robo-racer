//! State shared between the network task and the control task.
//!
//! The control task is the only writer of the published fields (state, fault,
//! applied command, battery). The network task is the only writer of the input
//! fields (buttons, mode, sequence, link state, operator requests).
//!
//! Every access goes through a critical section, so both tasks can live on
//! different threads without a lock in the control path.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;

use robo_core::config::SpeedMode;
use robo_core::safety::{FaultKind, RobotState};
use robo_core::DriveCommand;

/// Values exchanged between the tasks.
#[derive(Debug, Clone, Copy)]
pub struct Mailbox {
    // ---- written by the network task ----
    /// Latest button mask received.
    pub buttons: u8,
    /// Latest speed mode received.
    pub mode: SpeedMode,
    /// Incremented for every accepted drive frame.
    pub seq: u16,
    /// True while a controller is connected and authenticated.
    pub link_up: bool,
    /// An arm request is pending.
    pub arm_request: bool,
    /// A disarm request is pending.
    pub disarm_request: bool,
    /// An acknowledgement of the latched fault is pending.
    pub ack_request: bool,
    /// An in-app emergency stop is pending.
    pub estop_request: bool,
    /// Last accepted client sequence number, for the status frame.
    pub client_seq: u16,

    // ---- written by the control task ----
    /// Current robot state.
    pub state: RobotState,
    /// Latched fault, if any.
    pub fault: Option<FaultKind>,
    /// Command actually applied to the motor drivers.
    pub applied: DriveCommand,
    /// Filtered pack voltage (mV), 0 before the first measurement.
    pub battery_mv: u16,
    /// Below the critical threshold (debounced) - a safety input.
    pub battery_ok: bool,
}

impl Mailbox {
    /// All fields at their safe defaults: no link, disarmed, stopped.
    pub const fn new() -> Self {
        Self {
            buttons: 0,
            mode: SpeedMode::Normal,
            seq: 0,
            link_up: false,
            arm_request: false,
            disarm_request: false,
            ack_request: false,
            estop_request: false,
            client_seq: 0,
            state: RobotState::Boot,
            fault: None,
            applied: DriveCommand::STOP,
            battery_mv: 0,
            battery_ok: true,
        }
    }
}

impl Default for Mailbox {
    fn default() -> Self {
        Self::new()
    }
}

static MAILBOX: Mutex<RefCell<Mailbox>> = Mutex::new(RefCell::new(Mailbox::new()));

/// Runs `f` with exclusive access to the mailbox.
pub fn with<R>(f: impl FnOnce(&mut Mailbox) -> R) -> R {
    critical_section::with(|cs| f(&mut MAILBOX.borrow_ref_mut(cs)))
}

/// Reads all inputs the control task needs in one critical section.
#[derive(Debug, Clone, Copy)]
pub struct Inputs {
    /// Latest button mask.
    pub buttons: u8,
    /// Latest speed mode.
    pub mode: SpeedMode,
    /// Command sequence number.
    pub seq: u16,
    /// Link state.
    pub link_up: bool,
    /// Pending operator requests, already cleared in the mailbox.
    pub arm: bool,
    /// Pending disarm request.
    pub disarm: bool,
    /// Pending fault acknowledgement.
    pub ack: bool,
    /// Pending emergency stop.
    pub estop: bool,
    /// Battery healthy (debounced).
    pub battery_ok: bool,
}

/// Takes the pending inputs and clears the one-shot requests.
pub fn take_inputs() -> Inputs {
    with(|m| Inputs {
        buttons: m.buttons,
        mode: m.mode,
        seq: m.seq,
        link_up: m.link_up,
        arm: core::mem::take(&mut m.arm_request),
        disarm: core::mem::take(&mut m.disarm_request),
        ack: core::mem::take(&mut m.ack_request),
        estop: core::mem::take(&mut m.estop_request),
        battery_ok: m.battery_ok,
    })
}

/// Session token issued at boot, also published for the logs.
pub static TOKEN: AtomicU32 = AtomicU32::new(0);

/// Stores the session token.
pub fn set_token(token: u32) {
    TOKEN.store(token, Ordering::Relaxed);
}
