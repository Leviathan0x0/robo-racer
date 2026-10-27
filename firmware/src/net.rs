//! Wi-Fi access point, DHCP server and the HTTP / WebSocket control server.
//!
//! The whole control surface is one TCP port (80):
//!
//! * `GET /`, `/styles.css`, `/app.js` - the control app, embedded at build
//!   time so no flash filesystem is needed,
//! * `GET /ws` with an `Upgrade: websocket` header - the control channel.
//!
//! Only one controller can hold the control channel at a time: the server
//! serves a single connection until it closes, then accepts the next one. That
//! keeps the firmware free of concurrent-session logic and stops a second phone
//! from fighting the first one for the robot.
//!
//! The session token is generated at boot and announced in a `HELLO` frame,
//! never in the HTML or the JavaScript.

use core::fmt::{self, Write as _};
use core::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

use embassy_net::tcp::TcpSocket;
use embassy_net::{Config as NetConfig, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_time::Duration;
use embedded_io_async::Write as _;
use edge_nal::UdpBind;
use esp_hal::rng::Rng;
use esp_hal::time::Instant;
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig, Interface, Password, Ssid,
    WifiController, WifiError,
};
use static_cell::StaticCell;

use robo_core::protocol::{self, encode_status, kind, status_flags, ClientFrame, FrameDecoder};
use robo_core::safety::RobotState;
use robo_core::websocket::{self, Opcode};

use crate::state;

/// Address of the robot in its own network.
const AP_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 4, 1);

/// Embedded control app.
const INDEX_HTML: &str = include_str!("../../web/index.html");
/// Embedded stylesheet.
const STYLES_CSS: &str = include_str!("../../web/styles.css");
/// Embedded application script.
const APP_JS: &str = include_str!("../../web/app.js");

/// TCP buffers for the single control connection.
const TCP_BUFFER: usize = 2048;
static TCP_RX: StaticCell<[u8; TCP_BUFFER]> = StaticCell::new();
static TCP_TX: StaticCell<[u8; TCP_BUFFER]> = StaticCell::new();

/// Network stack resources: one TCP socket (HTTP/WS) plus one UDP socket for
/// the DHCP server.
const SOCKETS: usize = 4;
static STACK_RESOURCES: StaticCell<StackResources<SOCKETS>> = StaticCell::new();
static DHCP_BUFFERS: StaticCell<edge_nal_embassy::UdpBuffers<1, 1500, 1500>> =
    StaticCell::new();

/// Build-time SSID, overridable with `ROBO_WIFI_SSID`.
fn ssid() -> &'static str {
    option_env!("ROBO_WIFI_SSID").unwrap_or("robo-racer")
}

/// Build-time password, overridable with `ROBO_WIFI_PASS`.
///
/// When it is not set at build time a per-boot random password is generated
/// and printed on the serial console, so the repository never contains a
/// working credential.
fn build_time_password() -> Option<&'static str> {
    option_env!("ROBO_WIFI_PASS")
}

/// Generates a 12 character password from the hardware RNG.
///
/// The alphabet omits look-alike characters so it can be typed from a screen.
fn random_password(rng: &Rng) -> [u8; 12] {
    const ALPHABET: &[u8] = b"abcdefghijkmnopqrstuvwxyz23456789";
    let mut out = [0u8; 12];
    let mut value = rng.random();
    for slot in &mut out {
        *slot = ALPHABET[(value % ALPHABET.len() as u32) as usize];
        value >>= 5;
        if value == 0 {
            value = rng.random();
        }
    }
    out
}

/// Starts the access point.
///
/// The returned controller must be kept alive: dropping it shuts the radio down.
pub fn start_ap(
    wifi: esp_hal::peripherals::WIFI<'static>,
) -> Result<(WifiController<'static>, Interface), WifiError> {
    // The buffer only needs to outlive the `Password` construction, which
    // copies the bytes.
    let generated;
    let password: &str = match build_time_password() {
        Some(configured) => configured,
        None => {
            let rng = Rng::new();
            generated = random_password(&rng);
            core::str::from_utf8(&generated).unwrap_or("roboracer")
        }
    };

    let ap = AccessPointConfig::default()
        .with_ssid(Ssid::try_from(ssid())?)
        .with_authentication(AuthenticationMethodConfig::Wpa2Personal(Password::try_from(
            password,
        )?));
    let mut controller = WifiController::new(wifi, ControllerConfig::default())?;
    controller.set_config(&WifiConfig::AccessPoint(ap))?;
    esp_println::println!(
        "robo-racer: access point '{}' up, password '{}', open http://{}",
        ssid(),
        password,
        AP_IP
    );
    Ok((controller, Interface::access_point()))
}

/// Creates the network stack with the static address `192.168.4.1/24`.
pub fn start_stack(iface: Interface) -> (Stack<'static>, Runner<'static, Interface>) {
    let config = NetConfig::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(AP_IP, 24),
        gateway: None,
        dns_servers: Default::default(),
    });
    embassy_net::new(
        iface,
        config,
        STACK_RESOURCES.init(StackResources::new()),
        0x1234_5678_9ABC_DEF0,
    )
}

/// Drives the network stack.
#[embassy_executor::task]
pub async fn net_runner(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

/// Minimal DHCP server, so phones get an address without manual configuration.
#[embassy_executor::task]
pub async fn dhcp_task(stack: Stack<'static>) -> ! {
    use edge_dhcp::io::server::run;
    use edge_dhcp::server::{Server, ServerOptions};

    let buffers = DHCP_BUFFERS.init(edge_nal_embassy::UdpBuffers::<1, 1500, 1500>::new());
    let udp = edge_nal_embassy::Udp::new(stack, buffers);
    let mut socket = match udp
        .bind(SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 67)))
        .await
    {
        Ok(socket) => socket,
        Err(error) => {
            esp_println::println!("robo-racer: DHCP socket failed: {:?}", error);
            loop {
                embassy_time::Timer::after(Duration::from_secs(3600)).await;
            }
        }
    };

    let start = Instant::now();
    let now = move || (start.elapsed().as_millis() / 1000) as u64;
    let mut server = Server::<_, 8>::new(now, AP_IP);
    let options = ServerOptions::new(AP_IP, None);
    let mut buf = [0u8; 1500];

    loop {
        if let Err(error) = run(&mut server, &options, &mut socket, &mut buf).await {
            esp_println::println!("robo-racer: DHCP server error: {:?}", error);
            embassy_time::Timer::after(Duration::from_millis(500)).await;
        }
    }
}

/// Serves the control app and the control channel.
#[embassy_executor::task]
pub async fn http_task(
    stack: Stack<'static>,
    token: u32,
    boot: Instant,
    link_timeout_ms: u64,
) -> ! {
    let rx = TCP_RX.init([0; TCP_BUFFER]);
    let tx = TCP_TX.init([0; TCP_BUFFER]);
    let mut socket = TcpSocket::new(stack, rx, tx);
    let mut head = [0u8; 1024];

    loop {
        socket.set_timeout(Some(Duration::from_secs(10)));
        if socket.accept(80).await.is_err() {
            socket.abort();
            continue;
        }

        let len = match read_request(&mut socket, &mut head).await {
            Some(len) => len,
            None => {
                socket.abort();
                continue;
            }
        };
        let request = &head[..len];

        if is_websocket_request(request) {
            if let Some(key) = header_value(request, b"sec-websocket-key") {
                if upgrade(&mut socket, key).await.is_ok() {
                    // A live controller; losing the link is a safety input.
                    state::with(|m| m.link_up = true);
                    let _ = session(&mut socket, token, boot, link_timeout_ms).await;
                    state::with(|m| m.link_up = false);
                }
            }
        } else {
            let _ = serve_static(&mut socket, request).await;
        }

        socket.close();
        let _ = socket.flush().await;
        socket.abort();
    }
}

/// Reads until the end of the HTTP headers or the buffer is full.
async fn read_request(socket: &mut TcpSocket<'_>, head: &mut [u8]) -> Option<usize> {
    socket.set_timeout(Some(Duration::from_secs(5)));
    let mut filled = 0;
    while filled < head.len() {
        match socket.read(&mut head[filled..]).await {
            Ok(0) => return None,
            Ok(n) => {
                filled += n;
                if find_subslice(&head[..filled], b"\r\n\r\n").is_some() {
                    return Some(filled);
                }
            }
            Err(_) => return None,
        }
    }
    None
}

/// Finds `needle` in `haystack`.
fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    (0..=haystack.len() - needle.len()).find(|&i| &haystack[i..i + needle.len()] == needle)
}

/// Returns the value of a header, case-insensitively, without surrounding
/// whitespace.
fn header_value<'a>(request: &'a [u8], name: &[u8]) -> Option<&'a [u8]> {
    let end = find_subslice(request, b"\r\n\r\n").unwrap_or(request.len());
    for line in request[..end].split(|&b| b == b'\n') {
        let line = match line.split_last() {
            Some((b'\r', rest)) => rest,
            _ => line,
        };
        let colon = match line.iter().position(|&b| b == b':') {
            Some(colon) => colon,
            None => continue,
        };
        if line[..colon].eq_ignore_ascii_case(name) {
            return Some(line[colon + 1..].trim_ascii());
        }
    }
    None
}

/// True when the request asks for a WebSocket upgrade.
fn is_websocket_request(request: &[u8]) -> bool {
    header_value(request, b"upgrade").is_some_and(|value| value.eq_ignore_ascii_case(b"websocket"))
}

/// Extracts the request path, without the query string.
fn request_path(request: &[u8]) -> &[u8] {
    let end = find_subslice(request, b"\r\n").unwrap_or(request.len());
    let line = &request[..end];
    let mut parts = line.split(|&b| b == b' ');
    let _method = parts.next();
    let with_query = parts.next().unwrap_or(b"/");
    match with_query.iter().position(|&b| b == b'?') {
        Some(index) => &with_query[..index],
        None => with_query,
    }
}

/// Answers the WebSocket handshake.
async fn upgrade(socket: &mut TcpSocket<'_>, key: &[u8]) -> Result<(), ()> {
    let accept = websocket::accept_key(key);
    let mut response = [0u8; 192];
    let len = {
        let mut writer = SliceWriter::new(&mut response);
        write!(
            writer,
            "HTTP/1.1 101 Switching Protocols\r\n\
             Upgrade: websocket\r\n\
             Connection: Upgrade\r\n\
             Sec-WebSocket-Accept: {}\r\n\r\n",
            core::str::from_utf8(&accept).unwrap_or_default()
        )
        .map_err(|_| ())?;
        writer.len
    };
    socket.write_all(&response[..len]).await.map_err(|_| ())?;
    socket.flush().await.map_err(|_| ())
}

/// Serves one static asset.
async fn serve_static(socket: &mut TcpSocket<'_>, request: &[u8]) -> Result<(), ()> {
    let (code, reason, content_type, body) = route(request_path(request));
    let mut head = [0u8; 256];
    let len = {
        let mut writer = SliceWriter::new(&mut head);
        write!(
            writer,
            "HTTP/1.1 {code} {reason}\r\n\
             Content-Type: {content_type}\r\n\
             Content-Length: {}\r\n\
             Cache-Control: no-store\r\n\
             Connection: close\r\n\r\n",
            body.len()
        )
        .map_err(|_| ())?;
        writer.len
    };
    socket.write_all(&head[..len]).await.map_err(|_| ())?;
    socket.write_all(body.as_bytes()).await.map_err(|_| ())?;
    socket.flush().await.map_err(|_| ())
}

/// Maps a path onto an embedded asset.
fn route(path: &[u8]) -> (u16, &'static str, &'static str, &'static str) {
    match path {
        b"/" | b"/index.html" => (200, "OK", "text/html; charset=utf-8", INDEX_HTML),
        b"/styles.css" => (200, "OK", "text/css; charset=utf-8", STYLES_CSS),
        b"/app.js" => (200, "OK", "application/javascript; charset=utf-8", APP_JS),
        _ => (404, "Not Found", "text/plain; charset=utf-8", "not found"),
    }
}

/// Runs one authenticated control session.
async fn session(
    socket: &mut TcpSocket<'_>,
    token: u32,
    boot: Instant,
    link_timeout_ms: u64,
) -> Result<(), ()> {
    let mut decoder = websocket::Decoder::new();
    let mut frames = FrameDecoder::new(token);
    let mut buf = [0u8; 512];

    let hello = protocol::encode_hello(token, now_ms(boot));
    send_frame(socket, Opcode::Binary, &hello).await?;
    send_status(socket).await?;

    // A live app sends frames at 10-20 Hz, so an idle socket means the browser
    // is gone. The timeout aborts the connection and the link is declared lost.
    socket.set_timeout(Some(Duration::from_millis(link_timeout_ms)));

    loop {
        let read = match socket.read(&mut buf).await {
            Ok(0) => return Ok(()),
            Ok(n) => n,
            Err(_) => return Ok(()),
        };
        if decoder.feed(&buf[..read]).is_err() {
            return Ok(());
        }
        while let Some(frame) = match decoder.next() {
            Ok(frame) => frame,
            Err(_) => return Ok(()),
        } {
            match frame.opcode {
                Opcode::Close => {
                    let _ = send_frame(socket, Opcode::Close, &[]).await;
                    return Ok(());
                }
                Opcode::Ping => {
                    let _ = send_frame(socket, Opcode::Pong, frame.data()).await;
                }
                Opcode::Text | Opcode::Binary => match frames.decode(frame.data()) {
                    Ok(client) => {
                        apply(&client);
                        if client.kind == kind::PING {
                            let pong = protocol::encode_pong(client.seq as u32, now_ms(boot));
                            let _ = send_frame(socket, Opcode::Binary, &pong).await;
                        }
                        let _ = send_status(socket).await;
                    }
                    Err(error) => {
                        let reply = protocol::encode_error(error.code());
                        let _ = send_frame(socket, Opcode::Binary, &reply).await;
                        let _ = send_status(socket).await;
                    }
                },
                Opcode::Continuation | Opcode::Pong => {}
            }
        }
    }
}

/// Turns an accepted client frame into a mailbox update.
fn apply(client: &ClientFrame) {
    if client.estop_requested() {
        state::with(|m| m.estop_request = true);
    }
    match client.kind {
        kind::DRIVE => state::with(|m| {
            m.buttons = client.buttons;
            m.mode = client.mode;
            m.seq = m.seq.wrapping_add(1);
            m.client_seq = client.seq;
        }),
        kind::ARM => state::with(|m| m.arm_request = true),
        kind::DISARM => state::with(|m| m.disarm_request = true),
        kind::ACK => state::with(|m| m.ack_request = true),
        _ => {}
    }
}

/// Sends the current robot status.
async fn send_status(socket: &mut TcpSocket<'_>) -> Result<(), ()> {
    let (robot, fault, flags, battery_mv, left, right, seq, mode) = state::with(|m| {
        let mut flags = status_flags::SESSION_OK;
        if matches!(m.state, RobotState::Armed | RobotState::Driving) {
            flags |= status_flags::ARMED;
        }
        if m.link_up {
            flags |= status_flags::LINK_UP;
        }
        if m.battery_ok {
            flags |= status_flags::BATTERY_OK;
        }
        (
            m.state,
            m.fault,
            flags,
            m.battery_mv,
            m.applied.left,
            m.applied.right,
            m.client_seq,
            m.mode,
        )
    });
    let frame = encode_status(robot, fault, flags, battery_mv, left, right, seq, mode);
    send_frame(socket, Opcode::Binary, &frame).await
}

/// Sends one unmasked WebSocket frame.
async fn send_frame(
    socket: &mut TcpSocket<'_>,
    opcode: Opcode,
    payload: &[u8],
) -> Result<(), ()> {
    let mut buf = [0u8; 96];
    let len = websocket::encode_frame(opcode, payload, &mut buf).ok_or(())?;
    socket.write_all(&buf[..len]).await.map_err(|_| ())
}

/// Milliseconds since boot.
fn now_ms(boot: Instant) -> u32 {
    boot.elapsed().as_millis() as u32
}

/// `core::fmt::Write` into a fixed byte slice.
struct SliceWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> SliceWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }
}

impl fmt::Write for SliceWriter<'_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let bytes = text.as_bytes();
        if self.len + bytes.len() > self.buf.len() {
            return Err(fmt::Error);
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}
