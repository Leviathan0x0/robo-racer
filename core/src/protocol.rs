//! Binary wire format between the control app and the firmware.
//!
//! The protocol is intentionally small and fixed-size so that the firmware can
//! validate every frame with a handful of comparisons and no allocation.
//!
//! # Client -> device frame (12 bytes, little endian)
//!
//! ```text
//! offset 0  u8   magic      (0xA5)
//! offset 1  u8   version    (1)
//! offset 2  u8   kind       (see `kind`)
//! offset 3  u8   buttons    (bitmask, see `crate::steering`)
//! offset 4  u8   mode       (0 slow, 1 normal, 2 fast)
//! offset 5  u8   flags      (bit0 = emergency stop request)
//! offset 6  u16  seq        (monotonically increasing per session)
//! offset 8  u32  token      (session token issued at boot)
//! ```
//!
//! # Device -> client frames
//!
//! * `STATUS` (16 bytes) - state, fault, battery, applied command.
//! * `PONG` (12 bytes) - echo of the client timestamp plus device uptime.
//! * `ERROR` (8 bytes) - reason a frame was rejected.

use crate::config::SpeedMode;
use crate::safety::{FaultKind, RobotState};
use crate::steering::BTN_MASK;

/// Frame magic byte.
pub const MAGIC: u8 = 0xA5;
/// Protocol version.
pub const VERSION: u8 = 1;
/// Exact size of a client frame.
pub const CLIENT_FRAME_LEN: usize = 12;

/// Client frame kinds.
pub mod kind {
    /// Movement request (button state).
    pub const DRIVE: u8 = 1;
    /// Arm request.
    pub const ARM: u8 = 2;
    /// Disarm request.
    pub const DISARM: u8 = 3;
    /// Latency probe.
    pub const PING: u8 = 4;
    /// Acknowledge a latched fault.
    pub const ACK: u8 = 5;
}

/// Device frame kinds.
pub mod server_kind {
    /// Session announcement: carries the token the client must use.
    pub const HELLO: u8 = 0x80;
    /// Periodic status report.
    pub const STATUS: u8 = 0x81;
    /// Answer to a ping.
    pub const PONG: u8 = 0x82;
    /// Rejected frame notification.
    pub const ERROR: u8 = 0x83;
}

/// Client frame flags.
pub mod flags {
    /// Latched emergency stop request.
    pub const ESTOP: u8 = 1 << 0;
}

/// Size of a status frame.
pub const STATUS_FRAME_LEN: usize = 16;
/// Size of a pong frame.
pub const PONG_FRAME_LEN: usize = 12;
/// Size of an error frame.
pub const ERROR_FRAME_LEN: usize = 8;
/// Size of a hello frame.
pub const HELLO_FRAME_LEN: usize = 12;

/// Why a frame was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolError {
    /// Frame shorter than the fixed size.
    TooShort,
    /// Frame longer than the fixed size.
    TooLong,
    /// Wrong magic byte.
    BadMagic,
    /// Unsupported protocol version.
    BadVersion,
    /// Unknown frame kind.
    UnknownKind(u8),
    /// Button bits outside the known mask.
    InvalidButtons(u8),
    /// Unknown speed mode.
    InvalidMode(u8),
    /// Session token mismatch.
    BadToken,
    /// Sequence number not newer than the last accepted one.
    StaleSequence(u16),
}

impl ProtocolError {
    /// Short machine readable code, sent back to the app.
    pub fn code(&self) -> u8 {
        match self {
            ProtocolError::TooShort => 1,
            ProtocolError::TooLong => 2,
            ProtocolError::BadMagic => 3,
            ProtocolError::BadVersion => 4,
            ProtocolError::UnknownKind(_) => 5,
            ProtocolError::InvalidButtons(_) => 6,
            ProtocolError::InvalidMode(_) => 7,
            ProtocolError::BadToken => 8,
            ProtocolError::StaleSequence(_) => 9,
        }
    }
}

/// A decoded client frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClientFrame {
    /// Frame kind.
    pub kind: u8,
    /// Button bitmask.
    pub buttons: u8,
    /// Selected speed mode.
    pub mode: SpeedMode,
    /// Frame flags.
    pub flags: u8,
    /// Sequence number.
    pub seq: u16,
    /// Session token.
    pub token: u32,
}

impl ClientFrame {
    /// True when the emergency stop flag is set.
    pub fn estop_requested(&self) -> bool {
        self.flags & flags::ESTOP != 0
    }

    /// Decodes and structurally validates a client frame.
    ///
    /// This does **not** check the session token or the sequence number; use
    /// [`FrameDecoder`] for that.
    pub fn decode(buf: &[u8]) -> Result<Self, ProtocolError> {
        if buf.len() < CLIENT_FRAME_LEN {
            return Err(ProtocolError::TooShort);
        }
        if buf.len() > CLIENT_FRAME_LEN {
            return Err(ProtocolError::TooLong);
        }
        if buf[0] != MAGIC {
            return Err(ProtocolError::BadMagic);
        }
        if buf[1] != VERSION {
            return Err(ProtocolError::BadVersion);
        }
        let kind = buf[2];
        if !matches!(
            kind,
            kind::DRIVE | kind::ARM | kind::DISARM | kind::PING | kind::ACK
        ) {
            return Err(ProtocolError::UnknownKind(kind));
        }
        let buttons = buf[3];
        if buttons & !BTN_MASK != 0 {
            return Err(ProtocolError::InvalidButtons(buttons));
        }
        let mode = match buf[4] {
            0 => SpeedMode::Slow,
            1 => SpeedMode::Normal,
            2 => SpeedMode::Fast,
            other => return Err(ProtocolError::InvalidMode(other)),
        };
        let frame_flags = buf[5];
        let seq = u16::from_le_bytes([buf[6], buf[7]]);
        let token = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        Ok(ClientFrame {
            kind,
            buttons,
            mode,
            flags: frame_flags,
            seq,
            token,
        })
    }
}

/// Stateful decoder that enforces the session token and rejects stale or
/// duplicated sequence numbers.
#[derive(Debug, Clone, Copy)]
pub struct FrameDecoder {
    token: u32,
    last_seq: Option<u16>,
}

impl FrameDecoder {
    /// Creates a decoder for the given session token.
    pub const fn new(token: u32) -> Self {
        Self {
            token,
            last_seq: None,
        }
    }

    /// Session token expected by this decoder.
    pub fn token(&self) -> u32 {
        self.token
    }

    /// Sequence number of the last accepted frame.
    pub fn last_seq(&self) -> Option<u16> {
        self.last_seq
    }

    /// Decodes a frame and applies the session and sequence rules.
    ///
    /// A duplicate or out-of-order frame yields [`ProtocolError::StaleSequence`]
    /// and is ignored by the caller; it never causes a fault.
    pub fn decode(&mut self, buf: &[u8]) -> Result<ClientFrame, ProtocolError> {
        let frame = ClientFrame::decode(buf)?;
        if frame.token != self.token {
            return Err(ProtocolError::BadToken);
        }
        if let Some(last) = self.last_seq {
            // Wrapping comparison: a sequence number is "newer" when the
            // unsigned distance to it is in (0, 32768).
            if frame.seq.wrapping_sub(last) == 0 || frame.seq.wrapping_sub(last) > 0x8000 {
                return Err(ProtocolError::StaleSequence(frame.seq));
            }
        }
        self.last_seq = Some(frame.seq);
        Ok(frame)
    }
}

/// Builds a client frame (used by the app and by the tests).
pub fn encode_client_frame(
    kind: u8,
    buttons: u8,
    mode: SpeedMode,
    flags: u8,
    seq: u16,
    token: u32,
) -> [u8; CLIENT_FRAME_LEN] {
    let mut out = [0u8; CLIENT_FRAME_LEN];
    out[0] = MAGIC;
    out[1] = VERSION;
    out[2] = kind;
    out[3] = buttons;
    out[4] = match mode {
        SpeedMode::Slow => 0,
        SpeedMode::Normal => 1,
        SpeedMode::Fast => 2,
    };
    out[5] = flags;
    out[6..8].copy_from_slice(&seq.to_le_bytes());
    out[8..12].copy_from_slice(&token.to_le_bytes());
    out
}

/// Status frame flags.
pub mod status_flags {
    /// The robot is armed.
    pub const ARMED: u8 = 1 << 0;
    /// The control link is up.
    pub const LINK_UP: u8 = 1 << 1;
    /// The battery is above the critical threshold.
    pub const BATTERY_OK: u8 = 1 << 2;
    /// The client is the authorised controller.
    pub const SESSION_OK: u8 = 1 << 3;
}

/// Encodes a status frame.
#[allow(clippy::too_many_arguments)]
pub fn encode_status(
    state: RobotState,
    fault: Option<FaultKind>,
    flags: u8,
    battery_mv: u16,
    left: i16,
    right: i16,
    last_seq: u16,
    mode: SpeedMode,
) -> [u8; STATUS_FRAME_LEN] {
    let mut out = [0u8; STATUS_FRAME_LEN];
    out[0] = MAGIC;
    out[1] = VERSION;
    out[2] = server_kind::STATUS;
    out[3] = state as u8;
    out[4] = fault.map(|f| f as u8).unwrap_or(0);
    out[5] = flags;
    out[6..8].copy_from_slice(&battery_mv.to_le_bytes());
    out[8..10].copy_from_slice(&left.to_le_bytes());
    out[10..12].copy_from_slice(&right.to_le_bytes());
    out[12..14].copy_from_slice(&last_seq.to_le_bytes());
    out[14] = match mode {
        SpeedMode::Slow => 0,
        SpeedMode::Normal => 1,
        SpeedMode::Fast => 2,
    };
    out[15] = 0;
    out
}

/// Encodes the session announcement: the token the client must echo plus the
/// device uptime, so the app can detect a reboot.
pub fn encode_hello(token: u32, uptime_ms: u32) -> [u8; HELLO_FRAME_LEN] {
    let mut out = [0u8; HELLO_FRAME_LEN];
    out[0] = MAGIC;
    out[1] = VERSION;
    out[2] = server_kind::HELLO;
    out[3] = 0;
    out[4..8].copy_from_slice(&token.to_le_bytes());
    out[8..12].copy_from_slice(&uptime_ms.to_le_bytes());
    out
}

/// Encodes a pong frame carrying the client timestamp and the device uptime.
pub fn encode_pong(client_ts: u32, uptime_ms: u32) -> [u8; PONG_FRAME_LEN] {
    let mut out = [0u8; PONG_FRAME_LEN];
    out[0] = MAGIC;
    out[1] = VERSION;
    out[2] = server_kind::PONG;
    out[3] = 0;
    out[4..8].copy_from_slice(&client_ts.to_le_bytes());
    out[8..12].copy_from_slice(&uptime_ms.to_le_bytes());
    out
}

/// Encodes an error frame.
pub fn encode_error(code: u8) -> [u8; ERROR_FRAME_LEN] {
    let mut out = [0u8; ERROR_FRAME_LEN];
    out[0] = MAGIC;
    out[1] = VERSION;
    out[2] = server_kind::ERROR;
    out[3] = code;
    out[4..8].copy_from_slice(&0u32.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::steering::{BTN_FORWARD, BTN_STOP};

    const TOKEN: u32 = 0xDEAD_BEEF;

    #[test]
    fn round_trips_a_drive_frame() {
        let bytes = encode_client_frame(
            kind::DRIVE,
            BTN_FORWARD,
            SpeedMode::Normal,
            0,
            7,
            TOKEN,
        );
        let frame = ClientFrame::decode(&bytes).unwrap();
        assert_eq!(frame.kind, kind::DRIVE);
        assert_eq!(frame.buttons, BTN_FORWARD);
        assert_eq!(frame.mode, SpeedMode::Normal);
        assert_eq!(frame.seq, 7);
        assert_eq!(frame.token, TOKEN);
    }

    #[test]
    fn rejects_wrong_magic() {
        let mut bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 1, TOKEN);
        bytes[0] = 0x00;
        assert_eq!(ClientFrame::decode(&bytes), Err(ProtocolError::BadMagic));
    }

    #[test]
    fn rejects_wrong_version() {
        let mut bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 1, TOKEN);
        bytes[1] = 9;
        assert_eq!(ClientFrame::decode(&bytes), Err(ProtocolError::BadVersion));
    }

    #[test]
    fn rejects_unknown_kind() {
        let mut bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 1, TOKEN);
        bytes[2] = 42;
        assert_eq!(ClientFrame::decode(&bytes), Err(ProtocolError::UnknownKind(42)));
    }

    #[test]
    fn rejects_unknown_buttons() {
        let mut bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 1, TOKEN);
        bytes[3] = 0x80;
        assert_eq!(ClientFrame::decode(&bytes), Err(ProtocolError::InvalidButtons(0x80)));
    }

    #[test]
    fn rejects_unknown_mode() {
        let mut bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 1, TOKEN);
        bytes[4] = 7;
        assert_eq!(ClientFrame::decode(&bytes), Err(ProtocolError::InvalidMode(7)));
    }

    #[test]
    fn rejects_wrong_length() {
        let bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 1, TOKEN);
        assert_eq!(
            ClientFrame::decode(&bytes[..11]),
            Err(ProtocolError::TooShort)
        );
        let mut long = bytes.to_vec();
        long.push(0);
        assert_eq!(ClientFrame::decode(&long), Err(ProtocolError::TooLong));
    }

    #[test]
    fn rejects_bad_token() {
        let mut decoder = FrameDecoder::new(TOKEN);
        let bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 1, TOKEN ^ 1);
        assert_eq!(decoder.decode(&bytes), Err(ProtocolError::BadToken));
    }

    #[test]
    fn rejects_duplicate_and_stale_sequences() {
        let mut decoder = FrameDecoder::new(TOKEN);
        let first = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 10, TOKEN);
        assert!(decoder.decode(&first).is_ok());
        // duplicate
        assert_eq!(
            decoder.decode(&first),
            Err(ProtocolError::StaleSequence(10))
        );
        // older
        let older = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 9, TOKEN);
        assert_eq!(decoder.decode(&older), Err(ProtocolError::StaleSequence(9)));
        // newer
        let newer = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 11, TOKEN);
        assert!(decoder.decode(&newer).is_ok());
    }

    #[test]
    fn sequence_wraparound_is_accepted() {
        let mut decoder = FrameDecoder::new(TOKEN);
        let bytes = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, u16::MAX, TOKEN);
        assert!(decoder.decode(&bytes).is_ok());
        let wrapped = encode_client_frame(kind::DRIVE, 0, SpeedMode::Normal, 0, 0, TOKEN);
        assert!(decoder.decode(&wrapped).is_ok());
    }

    #[test]
    fn estop_flag_is_decoded() {
        let bytes = encode_client_frame(
            kind::DRIVE,
            BTN_STOP,
            SpeedMode::Normal,
            flags::ESTOP,
            1,
            TOKEN,
        );
        let frame = ClientFrame::decode(&bytes).unwrap();
        assert!(frame.estop_requested());
    }

    #[test]
    fn status_frame_has_expected_size() {
        let status = encode_status(
            RobotState::Armed,
            None,
            status_flags::ARMED,
            12345,
            -100,
            200,
            3,
            SpeedMode::Normal,
        );
        assert_eq!(status.len(), STATUS_FRAME_LEN);
        assert_eq!(status[0], MAGIC);
        assert_eq!(status[2], server_kind::STATUS);
        assert_eq!(status[3], RobotState::Armed as u8);
        assert_eq!(u16::from_le_bytes([status[6], status[7]]), 12345);
    }

    #[test]
    fn pong_echoes_the_client_timestamp() {
        let pong = encode_pong(0xDEAD_BEEF, 1234);
        assert_eq!(pong.len(), PONG_FRAME_LEN);
        assert_eq!(
            u32::from_le_bytes([pong[4], pong[5], pong[6], pong[7]]),
            0xDEAD_BEEF
        );
    }
}
