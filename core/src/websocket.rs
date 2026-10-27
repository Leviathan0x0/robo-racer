//! Minimal RFC 6455 WebSocket support: handshake key, frame codec, no allocation.
//!
//! The control app uses a WebSocket because it is the only low-latency,
//! bidirectional transport every mobile browser supports without a native
//! client. Only the small subset this project needs is implemented:
//!
//! * the server handshake (SHA-1 + base64 of `Sec-WebSocket-Key`),
//! * unfragmented binary/text frames from the client,
//! * unmasked server frames up to 65535 bytes.
//!
//! Deliberate limitations (all fail closed, never silently):
//!
//! * fragmented messages are rejected ([`WsError::Fragmented`]),
//! * client frames larger than [`MAX_PAYLOAD`] are rejected,
//! * unmasked client frames are rejected ([`WsError::Unmasked`]).
//!
//! The module has no I/O and no heap use, so it is exercised directly by the
//! host test-suite.

use sha1::{Digest, Sha1};

/// Fixed GUID from RFC 6455 section 1.3.
pub const MAGIC_GUID: &[u8; 36] = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

/// Largest client frame payload the firmware accepts.
///
/// The control frame is 12 bytes; 64 leaves room for a future, larger command
/// without letting a peer make the firmware buffer an unbounded message.
pub const MAX_PAYLOAD: usize = 64;

/// Worst-case frame header: 2 base bytes + 8 length bytes + 4 mask bytes.
const MAX_HEADER: usize = 14;

/// WebSocket opcodes understood by this implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Opcode {
    /// Continuation of a fragmented message (rejected by the decoder).
    Continuation,
    /// UTF-8 text.
    Text,
    /// Binary payload.
    Binary,
    /// Close the connection.
    Close,
    /// Ping.
    Ping,
    /// Pong.
    Pong,
}

impl Opcode {
    /// Maps a raw nibble onto an opcode, returning `None` for reserved values.
    pub fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            0x0 => Opcode::Continuation,
            0x1 => Opcode::Text,
            0x2 => Opcode::Binary,
            0x8 => Opcode::Close,
            0x9 => Opcode::Ping,
            0xA => Opcode::Pong,
            _ => return None,
        })
    }

    /// Raw nibble for this opcode.
    pub fn as_u8(self) -> u8 {
        match self {
            Opcode::Continuation => 0x0,
            Opcode::Text => 0x1,
            Opcode::Binary => 0x2,
            Opcode::Close => 0x8,
            Opcode::Ping => 0x9,
            Opcode::Pong => 0xA,
        }
    }

    /// True for opcodes that are allowed to carry application data.
    pub fn is_data(self) -> bool {
        matches!(self, Opcode::Text | Opcode::Binary)
    }
}

/// Reason a frame could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WsError {
    /// Peer sent more bytes than the internal buffer can hold.
    Overflow,
    /// Client-to-server frames must be masked (RFC 6455 section 5.1).
    Unmasked,
    /// Fragmented messages are not supported.
    Fragmented,
    /// Reserved opcode.
    BadOpcode(u8),
    /// A control frame may not be fragmented or exceed 125 bytes.
    BadControlFrame,
    /// Payload or extended length outside the supported range.
    BadLength,
}

/// A decoded frame whose payload lives in the decoder's buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// Frame opcode.
    pub opcode: Opcode,
    /// Payload bytes.
    pub payload: [u8; MAX_PAYLOAD],
    /// Number of valid bytes in `payload`.
    pub len: u8,
}

impl Frame {
    /// The valid payload slice.
    pub fn data(&self) -> &[u8] {
        &self.payload[..self.len as usize]
    }
}

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Encodes exactly 20 bytes (a SHA-1 digest) as standard, padded base64.
pub fn base64_20(input: &[u8; 20]) -> [u8; 28] {
    let mut out = [0u8; 28];
    let mut i = 0;
    let mut o = 0;
    while i + 3 <= 20 {
        let n = ((input[i] as u32) << 16) | ((input[i + 1] as u32) << 8) | input[i + 2] as u32;
        out[o] = B64[(n >> 18) as usize & 63];
        out[o + 1] = B64[(n >> 12) as usize & 63];
        out[o + 2] = B64[(n >> 6) as usize & 63];
        out[o + 3] = B64[n as usize & 63];
        i += 3;
        o += 4;
    }
    // 20 % 3 == 2: one final group with two input bytes and one '=' pad.
    let n = ((input[18] as u32) << 16) | ((input[19] as u32) << 8);
    out[o] = B64[(n >> 18) as usize & 63];
    out[o + 1] = B64[(n >> 12) as usize & 63];
    out[o + 2] = B64[(n >> 6) as usize & 63];
    out[o + 3] = b'=';
    out
}

/// Computes the `Sec-WebSocket-Accept` value for a client key.
pub fn accept_key(client_key: &[u8]) -> [u8; 28] {
    let mut hasher = Sha1::new();
    hasher.update(client_key);
    hasher.update(MAGIC_GUID);
    let digest = hasher.finalize();
    let mut raw = [0u8; 20];
    raw.copy_from_slice(&digest);
    base64_20(&raw)
}

/// Incremental frame decoder for the bytes received on a TCP stream.
///
/// Frames are consumed one at a time; [`Decoder::next`] returns `Ok(None)` when
/// more bytes are required and an error when the peer violated the protocol.
/// An error is terminal for the connection.
#[derive(Debug, Clone)]
pub struct Decoder {
    buf: [u8; MAX_PAYLOAD + MAX_HEADER],
    filled: usize,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    /// Creates an empty decoder.
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_PAYLOAD + MAX_HEADER],
            filled: 0,
        }
    }

    /// Bytes currently buffered but not yet consumed.
    pub fn buffered(&self) -> usize {
        self.filled
    }

    /// Appends received bytes to the buffer.
    pub fn feed(&mut self, data: &[u8]) -> Result<(), WsError> {
        if self.filled + data.len() > self.buf.len() {
            return Err(WsError::Overflow);
        }
        self.buf[self.filled..self.filled + data.len()].copy_from_slice(data);
        self.filled += data.len();
        Ok(())
    }

    /// Attempts to decode the next complete frame.
    pub fn next(&mut self) -> Result<Option<Frame>, WsError> {
        if self.filled < 2 {
            return Ok(None);
        }

        let first = self.buf[0];
        let second = self.buf[1];

        if first & 0x70 != 0 {
            // RSV1..3 must be zero: no extensions were negotiated.
            return Err(WsError::BadOpcode(first & 0x0F));
        }
        if first & 0x80 == 0 {
            return Err(WsError::Fragmented);
        }
        let opcode = Opcode::from_u8(first & 0x0F).ok_or(WsError::BadOpcode(first & 0x0F))?;
        if second & 0x80 == 0 {
            return Err(WsError::Unmasked);
        }

        let len7 = (second & 0x7F) as usize;
        let mut header = 2usize;
        let payload_len = if len7 < 126 {
            len7
        } else if len7 == 126 {
            if self.filled < 4 {
                return Ok(None);
            }
            header = 4;
            u16::from_be_bytes([self.buf[2], self.buf[3]]) as usize
        } else {
            // 64-bit lengths are far beyond any command this firmware accepts.
            return Err(WsError::BadLength);
        };

        if payload_len > MAX_PAYLOAD {
            return Err(WsError::BadLength);
        }
        if !opcode.is_data() && payload_len > 125 {
            return Err(WsError::BadControlFrame);
        }

        let total = header + 4 + payload_len;
        if self.filled < total {
            return Ok(None);
        }

        let mask = [
            self.buf[header],
            self.buf[header + 1],
            self.buf[header + 2],
            self.buf[header + 3],
        ];
        let mut frame = Frame {
            opcode,
            payload: [0; MAX_PAYLOAD],
            len: payload_len as u8,
        };
        let start = header + 4;
        for i in 0..payload_len {
            frame.payload[i] = self.buf[start + i] ^ mask[i & 3];
        }

        // Drop the consumed frame and keep any trailing bytes for the next call.
        self.buf.copy_within(total..self.filled, 0);
        self.filled -= total;

        Ok(Some(frame))
    }
}

/// Encodes one unmasked server frame into `out`, returning the byte count.
///
/// Returns `None` when `out` is too small; the caller keeps a buffer sized for
/// the largest frame it ever sends.
pub fn encode_frame(opcode: Opcode, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let len = payload.len();
    let header = if len < 126 {
        2
    } else if len <= u16::MAX as usize {
        4
    } else {
        return None;
    };
    if out.len() < header + len {
        return None;
    }
    out[0] = 0x80 | opcode.as_u8();
    if header == 2 {
        out[1] = len as u8;
    } else {
        out[1] = 126;
        out[2..4].copy_from_slice(&(len as u16).to_be_bytes());
    }
    out[header..header + len].copy_from_slice(payload);
    Some(header + len)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Masked binary frame builder used by the tests.
    fn masked(opcode: u8, payload: &[u8], mask: [u8; 4]) -> [u8; 32] {
        let mut out = [0u8; 32];
        out[0] = 0x80 | opcode;
        out[1] = 0x80 | payload.len() as u8;
        out[2..6].copy_from_slice(&mask);
        for (i, b) in payload.iter().enumerate() {
            out[6 + i] = b ^ mask[i & 3];
        }
        out
    }

    #[test]
    fn rfc6455_handshake_vector() {
        // RFC 6455 section 1.3.
        let accept = accept_key(b"dGhlIHNhbXBsZSBub25jZQ==");
        assert_eq!(&accept, b"s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
    }

    #[test]
    fn base64_pads_two_byte_tail() {
        let mut input = [0u8; 20];
        input[18] = 0xFF;
        input[19] = 0xFF;
        let encoded = base64_20(&input);
        assert_eq!(encoded[27], b'=');
    }

    #[test]
    fn decodes_a_masked_binary_frame() {
        let mut d = Decoder::new();
        let bytes = masked(0x2, &[1, 2, 3, 4], [0xAA, 0xBB, 0xCC, 0xDD]);
        d.feed(&bytes[..6]).unwrap();
        assert_eq!(d.next().unwrap(), None, "header without payload is incomplete");
        d.feed(&bytes[6..10]).unwrap();
        let frame = d.next().unwrap().unwrap();
        assert_eq!(frame.opcode, Opcode::Binary);
        assert_eq!(frame.data(), &[1, 2, 3, 4]);
        assert_eq!(d.buffered(), 0);
    }

    #[test]
    fn decodes_two_frames_from_one_read() {
        let mut d = Decoder::new();
        let mut bytes = [0u8; 20];
        let a = masked(0x2, &[7, 8], [1, 2, 3, 4]);
        bytes[..8].copy_from_slice(&a[..8]);
        let b = masked(0x2, &[9], [5, 6, 7, 8]);
        bytes[8..15].copy_from_slice(&b[..7]);
        d.feed(&bytes[..15]).unwrap();

        assert_eq!(d.next().unwrap().unwrap().data(), &[7, 8]);
        assert_eq!(d.next().unwrap().unwrap().data(), &[9]);
        assert_eq!(d.next().unwrap(), None);
    }

    #[test]
    fn rejects_unmasked_client_frame() {
        let mut d = Decoder::new();
        d.feed(&[0x82, 0x01, 0x00]).unwrap();
        assert_eq!(d.next(), Err(WsError::Unmasked));
    }

    #[test]
    fn rejects_fragmented_message() {
        let mut d = Decoder::new();
        // FIN = 0
        d.feed(&[0x02, 0x80, 1, 2, 3, 4, 0x00]).unwrap();
        assert_eq!(d.next(), Err(WsError::Fragmented));
    }

    #[test]
    fn rejects_reserved_opcode() {
        let mut d = Decoder::new();
        d.feed(&[0x83, 0x80, 1, 2, 3, 4]).unwrap();
        assert_eq!(d.next(), Err(WsError::BadOpcode(3)));
    }

    #[test]
    fn rejects_oversized_payload() {
        let mut d = Decoder::new();
        // 126 -> extended length 65 (> MAX_PAYLOAD)
        d.feed(&[0x82, 0xFE, 0x00, 0x41]).unwrap();
        assert_eq!(d.next(), Err(WsError::BadLength));
    }

    #[test]
    fn feed_overflow_is_detected() {
        let mut d = Decoder::new();
        let big = [0u8; MAX_PAYLOAD + MAX_HEADER];
        d.feed(&big).unwrap();
        assert_eq!(d.feed(&[0]), Err(WsError::Overflow));
    }

    #[test]
    fn encodes_server_frames_both_length_forms() {
        let mut out = [0u8; 300];
        let n = encode_frame(Opcode::Binary, &[1, 2, 3], &mut out).unwrap();
        assert_eq!(&out[..n], &[0x82, 0x03, 1, 2, 3]);

        let payload = [0xABu8; 200];
        let n = encode_frame(Opcode::Binary, &payload, &mut out).unwrap();
        assert_eq!(out[0], 0x82);
        assert_eq!(out[1], 126);
        assert_eq!(u16::from_be_bytes([out[2], out[3]]), 200);
        assert_eq!(&out[4..n], &payload[..]);
    }

    #[test]
    fn encode_reports_small_buffer() {
        let mut out = [0u8; 2];
        assert_eq!(encode_frame(Opcode::Binary, &[1, 2, 3], &mut out), None);
    }

    #[test]
    fn control_frames_are_recognised() {
        for (raw, expected) in [
            (0x8, Opcode::Close),
            (0x9, Opcode::Ping),
            (0xA, Opcode::Pong),
            (0x1, Opcode::Text),
            (0x2, Opcode::Binary),
            (0x0, Opcode::Continuation),
        ] {
            assert_eq!(Opcode::from_u8(raw), Some(expected));
        }
        assert_eq!(Opcode::from_u8(0xB), None);
    }
}
