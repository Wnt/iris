// license:BSD-3-Clause
// copyright-holders:osgallery lab
//! `mamectl/1` — the Kernel Hive host-native input plane for Iris.
//!
//! A unix `SOCK_STREAM` control channel serving protocol **`mamectl/1`**, the
//! same wire streamhost's `mamesock` backend already speaks to MAME's ctlsock
//! OSD module (`third_party/mame-irix/src/osd/modules/ctlsock/ctlsock.cpp`) and
//! to the `Wnt/previous` fork on the `nextstep` station. Because the wire is
//! identical, **streamhost needs no new code** for this station: the whole
//! integration is `SH_INPUT_BACKEND=mamesock` + `SH_MAMECTL_SOCK` +
//! `SH_MAMESOCK_KEYMAP` in the station env.
//!
//! ## The gate
//!
//! `IRIS_CTL_SOCK` unset => this module is never constructed, no thread is
//! spawned, nothing is bound, and the binary is byte-behaviourally identical to
//! stock Iris. Everything below happens only when the launcher sets it.
//!
//! ## Threading — one rule
//!
//! The socket threads **parse and enqueue only**. One `iris-ctl-engine` thread
//! owns every `Ps2Controller::push_*` call, the MOVEA convergence engine, key
//! hold/gap pacing and the button queue. So an `OK` on the wire means the verb
//! reached the emulated PS/2 controller's queue, not that a socket read
//! happened. (Iris's `push_kb`/`push_mouse_input` are internally locked and are
//! already called off the winit thread upstream, so unlike MAME there is no
//! ioport thread-affinity constraint — the single engine thread is here for
//! ordering and pacing determinism, not for safety.)
//!
//! ## The pointer is ABSOLUTE, closed-loop, against VC2
//!
//! The Indy's Newport board carries a hardware cursor whose position lives in
//! the VC2 registers `CURRENT_CURSOR_X` (0x04) and `WORKING_CURSOR_Y` (0x0D)
//! (`crate::vc2`), latched at VBLANK by the REX3 refresh loop. The compositor
//! draws the glyph at `reg - 31 (+ cursor_x_adjust)` (`compositor.rs:128-129`)
//! — the identical arithmetic MAME's `mame-vc2-cursor-swap.patch` exploits on
//! the sibling `irix` station. So the emulator can **read where the guest's
//! pointer actually is**, and `MOVEA x y` becomes a closed loop:
//!
//!   read the registers -> residual error in pixels -> a bounded, paced burst
//!   of relative PS/2 counts sized by a learned per-axis counts->pixels gain ->
//!   read again -> repeat until the residual is inside `IRIS_CTL_DEADBAND`.
//!
//! That is what makes the pointer immune to IRIX's pointer acceleration: an
//! open loop cannot undo a history-dependent ~3.5x gain, a closed loop never
//! needs to. The loop terminates by construction (bounded rounds, and a round
//! that observes no movement decays the gain estimate), and `EV MOVEA` reports
//! the landing.
//!
//! Two properties the daemon relies on (`streamhost/src/mame_sock.rs:1-48`):
//! **`MOVEA` acks on ACCEPT** (completion is the async `EV MOVEA` line), and
//! **button edges ack when the edge APPLIES** — a `DOWNn` issued while a MOVEA
//! is still converging is deferred until it lands, so a click cannot fire at
//! the old position.
//!
//! ## Keyboard
//!
//! `KEY <0|1> kbd <winit KeyCode name>` -> `Ps2Controller::push_kb`. Iris takes
//! `winit::keyboard::KeyCode` and applies no layout translation of its own
//! (`ui.rs:923`; the guest applies its `keybd=` layout), so the station's
//! keymap file is `browser XT set-1 scancode <TAB> kbd <TAB> KeyCode name` and
//! `NAMES` below is its single source of truth. Unlike a scanned matrix, a PS/2
//! keyboard is a *queue*: a make/break pair is never missed however fast it is
//! sent, so `IRIS_CTL_KEY_EXCL` exists but is **not** required here (contrast
//! the MAME stations, where it is mandatory). Hold/gap pacing is still applied
//! so the guest's own auto-repeat and X's key-press timing behave.
//!
//! ## The silent-drop trap
//!
//! `push_kb` returns early unless `running && scanning_enabled && !(config &
//! 0x10)`, and `push_mouse_input` unless `running && mouse_enabled && !(config
//! & 0x20)` (`ps2.rs:413-417,814-817`) — the i8042 CTR port-disable bits both
//! Linux and IRIX toggle while probing. Injecting into a disabled port does
//! nothing and reports nothing. This module therefore checks
//! `Ps2Controller::input_ready()` before every injection and replies
//! **`ERR kbdoff` / `ERR mouseoff`** rather than a lying `OK`, so a rig cannot
//! look healthy while the guest ignores every verb.
//!
//! ## Env
//!
//! | knob | default | meaning |
//! |---|---|---|
//! | `IRIS_CTL_SOCK` | *(unset)* | listener path; unset => module off entirely |
//! | `IRIS_CTL_SCREEN` | *(from VC2)* | `WxH` clamp surface + `screen=` in HELLO |
//! | `IRIS_CTL_CAL_X` / `_Y` | `-31` | cursor register -> pixel calibration |
//! | `IRIS_CTL_DEADBAND` | `1` | MOVEA convergence deadband, px |
//! | `IRIS_CTL_MOVE_STEP` | `96` | pacing budget, counts per window per axis |
//! | `IRIS_CTL_MOVE_WINDOW` | `10` | pacing window, wall ms (PS/2 samples at 100 Hz) |
//! | `IRIS_CTL_MOVEA_ROUNDS` | `80` | give-up cap for one flight |
//! | `IRIS_CTL_SETTLE` | `40` | ms a round waits for its counts to reach the cursor registers |
//! | `IRIS_CTL_ACCEL_THRESHOLD` | `4` | px at/below which the guest is 1:1 (IRIX's own accel threshold) |
//! | `IRIS_CTL_GAIN_MARGIN` | `1.10` | never extrapolate ahead of real movement |
//! | `IRIS_CTL_KEY_HOLD` / `_GAP` | `40` / `40` | key edge pacing, wall ms |
//! | `IRIS_CTL_KEY_EXCL` | `0` | 1 => one key held at a time |
//! | `IRIS_CTL_STAT_PERIOD` | `15` | `EV STATS` heartbeat period, wall s |
//! | `IRIS_CTL_TRACE` | `0` | 1 => per-verb engine trace on stderr |

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use winit::keyboard::KeyCode;

use crate::machine::Machine;
use crate::ps2::Ps2Controller;
use crate::rex3::Rex3;
use crate::vc2::{VC2_REG_CURRENT_CURSOR_X, VC2_REG_CURSOR_Y_LOC};

pub const PROTO_ID: &str = "mamectl/1";

// ---------------------------------------------------------------------------
// the machine pointer — one socket, all the verbs
// ---------------------------------------------------------------------------

/// The `Machine` this listener drives, carried so the engine thread can hand
/// the lifecycle verbs (`SAVEST` / `LOADST` / `RESET` / `FBSYNC` / `CKPT`) to
/// [`crate::kh_ctl`].
///
/// WHY IT LIVES HERE. `mame_sock.rs` gives a station exactly ONE control
/// socket (`SH_MAMECTL_SOCK`), and `scripts/serve/reset-tile.sh` sends
/// `LOADST golden` down that same socket the browser's pointer rides. Two
/// listeners would mean two sockets and a station that can be driven or reset
/// but not both, so this module owns the socket and `kh_ctl` owns the verbs:
/// the merge hook `kh_ctl` documents at its module top, taken.
///
/// Routing them through the ENGINE thread rather than the connection thread is
/// the point of the seam. The engine is the single applier of keys, buttons and
/// pointer counts, so a `LOADST` can never interleave with a half-drained
/// MOVEA burst — it is applied in wire order with everything ahead of it,
/// which is exactly what an `OK` promises the daemon.
struct MachinePtr(*mut Machine);
// SAFETY: the pointer is valid for the process lifetime (`main` parks after
// handing it over) and is dereferenced ONLY on the engine thread, which is the
// single applier for this module.
unsafe impl Send for MachinePtr {}
unsafe impl Sync for MachinePtr {}

/// The one port name this module accepts in `KEY <0|1> <port> <field>`.
/// Iris's keyboard is not a matrix, so there is exactly one.
pub const KBD_PORT: &str = "kbd";

const MAX_LINE: usize = 8192;

// ---------------------------------------------------------------------------
// env helpers
// ---------------------------------------------------------------------------

fn env_f64(name: &str, default: f64) -> f64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
}
fn env_i64(name: &str, default: i64) -> i64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
}
fn env_bool(name: &str, default: bool) -> bool {
    match std::env::var(name) {
        Ok(v) => matches!(v.trim(), "1" | "on" | "true" | "yes"),
        Err(_) => default,
    }
}

#[derive(Clone, Copy)]
struct Cfg {
    cal_x: i32,
    cal_y: i32,
    deadband: i32,
    move_step: i32,
    move_window: Duration,
    movea_rounds: u32,
    settle: Duration,
    accel_threshold: i32,
    gain_margin: f64,
    key_hold: Duration,
    key_gap: Duration,
    key_excl: bool,
    stat_period: Duration,
    trace: bool,
}

impl Cfg {
    fn from_env() -> Self {
        Self {
            cal_x: env_i64("IRIS_CTL_CAL_X", -31) as i32,
            cal_y: env_i64("IRIS_CTL_CAL_Y", -31) as i32,
            deadband: env_i64("IRIS_CTL_DEADBAND", 1).max(0) as i32,
            move_step: env_i64("IRIS_CTL_MOVE_STEP", 96).clamp(1, 255) as i32,
            move_window: Duration::from_millis(env_i64("IRIS_CTL_MOVE_WINDOW", 10).max(1) as u64),
            movea_rounds: env_i64("IRIS_CTL_MOVEA_ROUNDS", 80).max(1) as u32,
            settle: Duration::from_millis(env_i64("IRIS_CTL_SETTLE", 40).max(1) as u64),
            accel_threshold: env_i64("IRIS_CTL_ACCEL_THRESHOLD", 4).max(0) as i32,
            gain_margin: env_f64("IRIS_CTL_GAIN_MARGIN", 1.10).max(1.0),
            key_hold: Duration::from_millis(env_i64("IRIS_CTL_KEY_HOLD", 40).max(0) as u64),
            key_gap: Duration::from_millis(env_i64("IRIS_CTL_KEY_GAP", 40).max(0) as u64),
            key_excl: env_bool("IRIS_CTL_KEY_EXCL", false),
            stat_period: Duration::from_secs(env_i64("IRIS_CTL_STAT_PERIOD", 15).max(1) as u64),
            trace: env_bool("IRIS_CTL_TRACE", false),
        }
    }
}

// ---------------------------------------------------------------------------
// keymap: winit KeyCode <-> name
// ---------------------------------------------------------------------------

/// Every `KeyCode` `Ps2Controller::map_keycode_set1` can encode, under its
/// `winit` variant name. This table IS the contract with
/// `streamhost/stations/indyr4400/indy.keymap`; `scripts/dev/iris-keymap.py`
/// regenerates that file from the browser's XT set-1 scancodes against these
/// names, and `KEYDUMP` prints them so the generator can never drift from the
/// binary it will drive.
#[rustfmt::skip] // one row per key would be 100 lines of noise; the grid is the table
pub const NAMES: &[(&str, KeyCode)] = &[
    ("KeyA", KeyCode::KeyA), ("KeyB", KeyCode::KeyB), ("KeyC", KeyCode::KeyC),
    ("KeyD", KeyCode::KeyD), ("KeyE", KeyCode::KeyE), ("KeyF", KeyCode::KeyF),
    ("KeyG", KeyCode::KeyG), ("KeyH", KeyCode::KeyH), ("KeyI", KeyCode::KeyI),
    ("KeyJ", KeyCode::KeyJ), ("KeyK", KeyCode::KeyK), ("KeyL", KeyCode::KeyL),
    ("KeyM", KeyCode::KeyM), ("KeyN", KeyCode::KeyN), ("KeyO", KeyCode::KeyO),
    ("KeyP", KeyCode::KeyP), ("KeyQ", KeyCode::KeyQ), ("KeyR", KeyCode::KeyR),
    ("KeyS", KeyCode::KeyS), ("KeyT", KeyCode::KeyT), ("KeyU", KeyCode::KeyU),
    ("KeyV", KeyCode::KeyV), ("KeyW", KeyCode::KeyW), ("KeyX", KeyCode::KeyX),
    ("KeyY", KeyCode::KeyY), ("KeyZ", KeyCode::KeyZ),
    ("Digit0", KeyCode::Digit0), ("Digit1", KeyCode::Digit1), ("Digit2", KeyCode::Digit2),
    ("Digit3", KeyCode::Digit3), ("Digit4", KeyCode::Digit4), ("Digit5", KeyCode::Digit5),
    ("Digit6", KeyCode::Digit6), ("Digit7", KeyCode::Digit7), ("Digit8", KeyCode::Digit8),
    ("Digit9", KeyCode::Digit9),
    ("Minus", KeyCode::Minus), ("Equal", KeyCode::Equal),
    ("BracketLeft", KeyCode::BracketLeft), ("BracketRight", KeyCode::BracketRight),
    ("Backslash", KeyCode::Backslash), ("IntlBackslash", KeyCode::IntlBackslash),
    ("Semicolon", KeyCode::Semicolon), ("Quote", KeyCode::Quote),
    ("Comma", KeyCode::Comma), ("Period", KeyCode::Period), ("Slash", KeyCode::Slash),
    ("Backquote", KeyCode::Backquote), ("Space", KeyCode::Space),
    ("Enter", KeyCode::Enter), ("Backspace", KeyCode::Backspace),
    ("Escape", KeyCode::Escape), ("Tab", KeyCode::Tab),
    ("ShiftLeft", KeyCode::ShiftLeft), ("ShiftRight", KeyCode::ShiftRight),
    ("ControlLeft", KeyCode::ControlLeft), ("ControlRight", KeyCode::ControlRight),
    ("AltLeft", KeyCode::AltLeft), ("AltRight", KeyCode::AltRight),
    ("ArrowUp", KeyCode::ArrowUp), ("ArrowDown", KeyCode::ArrowDown),
    ("ArrowLeft", KeyCode::ArrowLeft), ("ArrowRight", KeyCode::ArrowRight),
    ("F1", KeyCode::F1), ("F2", KeyCode::F2), ("F3", KeyCode::F3), ("F4", KeyCode::F4),
    ("F5", KeyCode::F5), ("F6", KeyCode::F6), ("F7", KeyCode::F7), ("F8", KeyCode::F8),
    ("F9", KeyCode::F9), ("F10", KeyCode::F10), ("F11", KeyCode::F11), ("F12", KeyCode::F12),
    ("Numpad0", KeyCode::Numpad0), ("Numpad1", KeyCode::Numpad1), ("Numpad2", KeyCode::Numpad2),
    ("Numpad3", KeyCode::Numpad3), ("Numpad4", KeyCode::Numpad4), ("Numpad5", KeyCode::Numpad5),
    ("Numpad6", KeyCode::Numpad6), ("Numpad7", KeyCode::Numpad7), ("Numpad8", KeyCode::Numpad8),
    ("Numpad9", KeyCode::Numpad9),
    ("NumpadAdd", KeyCode::NumpadAdd), ("NumpadSubtract", KeyCode::NumpadSubtract),
    ("NumpadMultiply", KeyCode::NumpadMultiply), ("NumpadDivide", KeyCode::NumpadDivide),
    ("NumpadDecimal", KeyCode::NumpadDecimal), ("NumpadEnter", KeyCode::NumpadEnter),
    ("NumLock", KeyCode::NumLock), ("ScrollLock", KeyCode::ScrollLock),
    ("SuperLeft", KeyCode::SuperLeft), ("SuperRight", KeyCode::SuperRight),
    ("ContextMenu", KeyCode::ContextMenu),
    ("Home", KeyCode::Home), ("End", KeyCode::End),
    ("PageUp", KeyCode::PageUp), ("PageDown", KeyCode::PageDown),
    ("Insert", KeyCode::Insert), ("Delete", KeyCode::Delete),
];

/// Resolve a `KEY` field name. Fails CLOSED: an unknown name is `ERR nosuchkey`
/// and injects nothing, never a best-effort guess onto some other key.
pub fn keycode_by_name(name: &str) -> Option<KeyCode> {
    NAMES.iter().find(|(n, _)| *n == name).map(|(_, k)| *k)
}

// ---------------------------------------------------------------------------
// wire types
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct AckRef {
    conn: u64,
    seq: String,
}

impl AckRef {
    fn wants_reply(&self) -> bool {
        self.seq != "-"
    }
}

struct PendingCmd {
    conn: u64,
    seq: String,
    line: String,
    rx: Instant,
}

/// A key edge waiting on the hold/gap pacer.
struct KeyItem {
    key: KeyCode,
    down: bool,
    ack: AckRef,
}

/// A button edge waiting for the MOVEA flight to land (or for its own pacing).
struct BtnItem {
    btn: u8,
    down: bool,
    ack: AckRef,
}

/// A MOVEP delta being bled out at the pacing budget.
struct BleedItem {
    dx: i32,
    dy: i32,
    ack: AckRef,
}

/// A synthetic click: a queue of (button, state, hold-frames) edges.
struct ClickItem {
    btn: u8,
    steps: VecDeque<(bool, u32)>,
    ack: AckRef,
}

// ---------------------------------------------------------------------------
// the MOVEA flight
// ---------------------------------------------------------------------------

struct Flight {
    tx: i32,
    ty: i32,
    seq: String,
    rounds: u32,
    /// counts issued in the previous round, per axis, for gain learning
    last_issue: (i32, i32),
    last_read: (i32, i32),
    started: Instant,
}

/// Learned counts -> pixels gain, per axis. Starts at 1.0 (IRIX with
/// `xset m 1/1 0` is 1:1) and is corrected from what the cursor registers
/// actually did. Clamped so one anomalous round cannot wreck a flight.
#[derive(Clone, Copy)]
struct Gain {
    x: f64,
    y: f64,
}

impl Gain {
    fn new() -> Self {
        Self { x: 1.0, y: 1.0 }
    }
    fn learn(&mut self, issued: (i32, i32), moved: (i32, i32)) {
        for (iss, mov, g) in [
            (issued.0, moved.0, &mut self.x),
            (issued.1, moved.1, &mut self.y),
        ] {
            if iss.abs() < 8 {
                continue; // too small, or a 1:1 final-approach step, to measure a gain from
            }
            let observed = mov as f64 / iss as f64;
            if !observed.is_finite() {
                continue;
            }
            // EWMA, and never let a stalled round drive the gain to zero: a
            // round that observed nothing decays toward, not to, the floor.
            let next = 0.7 * *g + 0.3 * observed.clamp(0.05, 8.0);
            *g = next.clamp(0.1, 8.0);
        }
    }
}

// ---------------------------------------------------------------------------
// counters
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Counters {
    move_v: AtomicU64,
    movep: AtomicU64,
    movea: AtomicU64,
    movea_converged: AtomicU64,
    movea_giveup: AtomicU64,
    key: AtomicU64,
    btn: AtomicU64,
    errs: AtomicU64,
    gated: AtomicU64,
    /// reset-plane verbs served on this socket (`kh_ctl`'s)
    lifecycle: AtomicU64,
}

// ---------------------------------------------------------------------------
// server
// ---------------------------------------------------------------------------

struct Conn {
    id: u64,
    out: Mutex<UnixStream>,
}

struct Shared {
    cfg: Cfg,
    banner: String,
    /// The machine, for the lifecycle verbs `kh_ctl` owns. Only the engine
    /// thread dereferences it.
    machine: MachinePtr,
    surf_w: i32,
    surf_h: i32,
    ps2: Arc<Ps2Controller>,
    rex3: Option<Arc<Rex3>>,
    /// Last known `Rex3Screen::cursor_x_adjust` — the VT-timing X correction
    /// the compositor adds when it draws the glyph (`compositor.rs:128`).
    /// Cached, never re-read as 0: a `try_lock` that loses to a compose pass
    /// must not silently move the pointer 5 px. Seeded at init, refreshed
    /// lazily by the engine (it only changes on a mode switch).
    x_adjust: AtomicI32,
    inq: Mutex<VecDeque<PendingCmd>>,
    conns: Mutex<Vec<Arc<Conn>>>,
    counters: Counters,
    stop: AtomicBool,
}

impl Shared {
    fn send_to(&self, conn: u64, payload: &str) {
        let conns = self.conns.lock();
        for c in conns.iter() {
            if conn == 0 || c.id == conn {
                let mut s = c.out.lock();
                let _ = s.write_all(payload.as_bytes());
                let _ = s.flush();
            }
        }
    }
    fn reply_ok(&self, ack: &AckRef, data: &str) {
        if !ack.wants_reply() {
            return;
        }
        let line = if data.is_empty() {
            format!("{} OK\n", ack.seq)
        } else {
            format!("{} OK {}\n", ack.seq, data)
        };
        self.send_to(ack.conn, &line);
    }
    fn reply_err(&self, ack: &AckRef, code: &str, text: &str) {
        self.counters.errs.fetch_add(1, Ordering::Relaxed);
        if !ack.wants_reply() {
            return;
        }
        self.send_to(ack.conn, &format!("{} ERR {} {}\n", ack.seq, code, text));
    }
    fn reply_data(&self, ack: &AckRef, text: &str) {
        if !ack.wants_reply() {
            return;
        }
        self.send_to(ack.conn, &format!("{} D {}\n", ack.seq, text));
    }
    fn emit_ev(&self, line: &str) {
        self.send_to(0, &format!("EV {}\n", line));
    }
    fn trace(&self, line: &str) {
        if self.cfg.trace {
            eprintln!("CTLTRACE {}", line);
        }
    }
}

/// Handle returned to `main`; dropping it does not stop the threads (they live
/// for the process, exactly as the CI server's do), it only carries the path so
/// the socket file can be unlinked.
pub struct CtlServer {
    path: String,
    shared: Arc<Shared>,
}

impl CtlServer {
    pub fn path(&self) -> &str {
        &self.path
    }
    /// Test/ops hook: stop the engine thread and unlink the socket.
    pub fn shutdown(&self) {
        self.shared.stop.store(true, Ordering::Release);
        let _ = std::fs::remove_file(&self.path);
    }
}

/// Start the `mamectl/1` listener. `machine_ptr` must stay valid for the
/// process lifetime — the same contract `ci::start_server` takes.
///
/// # Safety
/// `machine_ptr` must point to a live `Machine` that outlives the process's
/// use of the returned server.
pub unsafe fn start_server(
    machine_ptr: *mut Machine,
    socket_path: &str,
) -> Result<Arc<CtlServer>, String> {
    let cfg = Cfg::from_env();
    let ps2 = (*machine_ptr).get_ps2();
    let rex3 = (*machine_ptr).get_rex3();

    // Clamp surface: IRIS_CTL_SCREEN wins, else the VC2-decoded geometry, else
    // the Indy's 1280x1024 default until the guest programs the timings.
    let (mut w, mut h) = (1280i32, 1024i32);
    let mut x_adjust = 0i32;
    if let Some(r) = rex3.as_ref() {
        let s = r.screen.lock();
        if s.width > 0 && s.height > 0 {
            w = s.width as i32;
            h = s.height as i32;
        }
        x_adjust = s.cursor_x_adjust;
    }
    if let Ok(v) = std::env::var("IRIS_CTL_SCREEN") {
        if let Some((a, b)) = v.split_once('x') {
            if let (Ok(a), Ok(b)) = (a.trim().parse::<i32>(), b.trim().parse::<i32>()) {
                if a > 0 && b > 0 {
                    w = a;
                    h = b;
                }
            }
        }
    }

    let mut caps = String::from("kbd,ptr,relatch");
    if rex3.is_some() {
        caps.push_str(",movea,cur");
    }
    // The reset plane rides this same socket, so it must ride this same banner:
    // a client that reads caps to decide whether it can reset the station has
    // to see the truth from the one connection it makes.
    caps.push(',');
    caps.push_str(crate::kh_ctl::caps_fragment());
    let banner = format!(
        "HELLO {} iris-{} indy caps={} screen={}x{}\n",
        PROTO_ID,
        env!("CARGO_PKG_VERSION"),
        caps,
        w,
        h
    );

    let shared = Arc::new(Shared {
        cfg,
        banner,
        machine: MachinePtr(machine_ptr),
        surf_w: w,
        surf_h: h,
        ps2,
        rex3,
        x_adjust: AtomicI32::new(x_adjust),
        inq: Mutex::new(VecDeque::new()),
        conns: Mutex::new(Vec::new()),
        counters: Counters::default(),
        stop: AtomicBool::new(false),
    });

    let _ = std::fs::remove_file(socket_path);
    let listener = UnixListener::bind(socket_path)
        .map_err(|e| format!("ctlsock: failed to bind {}: {}", socket_path, e))?;

    {
        let sh = shared.clone();
        thread::Builder::new()
            .name("iris-ctl-accept".into())
            .spawn(move || accept_loop(sh, listener))
            .map_err(|e| format!("ctlsock: accept thread: {}", e))?;
    }
    {
        let sh = shared.clone();
        thread::Builder::new()
            .name("iris-ctl-engine".into())
            .spawn(move || engine_loop(sh))
            .map_err(|e| format!("ctlsock: engine thread: {}", e))?;
    }

    eprintln!(
        "iris: mamectl/1 listening on {} (screen={}x{} deadband={} step={}/{}ms key={}/{}ms{})",
        socket_path,
        w,
        h,
        cfg.deadband,
        cfg.move_step,
        cfg.move_window.as_millis(),
        cfg.key_hold.as_millis(),
        cfg.key_gap.as_millis(),
        if cfg.key_excl { " excl" } else { "" }
    );

    Ok(Arc::new(CtlServer {
        path: socket_path.to_string(),
        shared,
    }))
}

fn accept_loop(shared: Arc<Shared>, listener: UnixListener) {
    let mut next_id: u64 = 1;
    for conn in listener.incoming() {
        if shared.stop.load(Ordering::Acquire) {
            return;
        }
        let Ok(stream) = conn else { continue };
        let id = next_id;
        next_id += 1;
        let Ok(rd) = stream.try_clone() else { continue };
        // A blocking write to a client that has stopped reading would freeze
        // the engine thread — and with it every key, button and pointer verb
        // for every other client. Bound it: a peer that cannot absorb an ack in
        // two seconds gets a dropped write, not a stalled emulator.
        let _ = stream.set_write_timeout(Some(Duration::from_secs(2)));
        let c = Arc::new(Conn {
            id,
            out: Mutex::new(stream),
        });
        {
            let mut s = c.out.lock();
            let _ = s.write_all(shared.banner.as_bytes());
            let _ = s.flush();
        }
        shared.conns.lock().push(c.clone());
        let sh = shared.clone();
        thread::Builder::new()
            .name("iris-ctl-conn".into())
            .spawn(move || {
                reader_loop(&sh, id, rd);
                sh.conns.lock().retain(|x| x.id != id);
            })
            .ok();
    }
}

/// Parse and enqueue ONLY. This thread never touches the machine.
fn reader_loop(shared: &Arc<Shared>, id: u64, stream: UnixStream) {
    let mut rd = BufReader::new(stream);
    let mut buf = String::new();
    loop {
        buf.clear();
        match rd.read_line(&mut buf) {
            Ok(0) | Err(_) => return,
            Ok(_) => {}
        }
        let line = buf.trim_end_matches(['\r', '\n']);
        if line.is_empty() {
            continue;
        }
        let cmd = if line.len() > MAX_LINE {
            PendingCmd {
                conn: id,
                seq: "0".into(),
                line: String::new(),
                rx: Instant::now(),
            }
        } else {
            let (tok, rest) = match line.split_once(' ') {
                Some((a, b)) => (a, b.to_string()),
                None => (line, String::new()),
            };
            let numeric = !tok.is_empty() && tok.bytes().all(|c| c.is_ascii_digit());
            if numeric || tok == "-" {
                PendingCmd {
                    conn: id,
                    seq: tok.to_string(),
                    line: rest,
                    rx: Instant::now(),
                }
            } else {
                PendingCmd {
                    conn: id,
                    seq: "0".into(),
                    line: String::new(),
                    rx: Instant::now(),
                }
            }
        };
        shared.inq.lock().push_back(cmd);
    }
}

// ---------------------------------------------------------------------------
// the engine thread — the ONLY caller of push_kb / push_mouse_input
// ---------------------------------------------------------------------------

struct Engine {
    shared: Arc<Shared>,
    buttons: u8,
    flight: Option<Flight>,
    gain: Gain,
    kq: VecDeque<KeyItem>,
    /// key currently held down, for `IRIS_CTL_KEY_EXCL`
    key_held: Option<KeyCode>,
    key_next_at: Instant,
    btnq: VecDeque<BtnItem>,
    bleedq: VecDeque<BleedItem>,
    bleed_at: Instant,
    clickq: VecDeque<ClickItem>,
    click_at: Instant,
    next_stat: Instant,
    /// earliest wall time the next MOVEA round may issue counts
    next_round: Instant,
    last_cur: (i32, i32),
}

fn engine_loop(shared: Arc<Shared>) {
    let cfg = shared.cfg;
    let now = Instant::now();
    let mut e = Engine {
        shared,
        buttons: 0,
        flight: None,
        gain: Gain::new(),
        kq: VecDeque::new(),
        key_held: None,
        key_next_at: now,
        btnq: VecDeque::new(),
        bleedq: VecDeque::new(),
        bleed_at: now,
        clickq: VecDeque::new(),
        click_at: now,
        next_stat: now + cfg.stat_period,
        next_round: now,
        last_cur: (0, 0),
    };
    let tick = cfg.move_window.min(Duration::from_millis(5));
    while !e.shared.stop.load(Ordering::Acquire) {
        // 1. drain every parsed line
        loop {
            let Some(cmd) = e.shared.inq.lock().pop_front() else {
                break;
            };
            e.exec(cmd);
        }
        // 2. run the pacers
        e.step_keys();
        e.step_clicks();
        e.step_bleed();
        e.step_flight();
        e.step_btns();
        e.step_stats();
        thread::sleep(tick);
    }
}

impl Engine {
    // ---- guest-state gates ------------------------------------------------

    fn kbd_ready(&self) -> bool {
        self.shared.ps2.input_ready().0
    }
    fn mouse_ready(&self) -> bool {
        self.shared.ps2.input_ready().1
    }

    // ---- the cursor reading ----------------------------------------------

    /// Read the guest's hardware-cursor position in SCREEN PIXELS from the VC2
    /// registers, applying the same `reg - 31 + cursor_x_adjust` arithmetic the
    /// compositor uses to draw the glyph — so what this returns is where the
    /// arrow is in the frame the visitor sees, which is the only definition
    /// that makes `cursor-locate.py` and the pointer proofs agree.
    fn reading(&self) -> Option<(i32, i32)> {
        let r = self.shared.rex3.as_ref()?;
        let (rx, ry) = {
            let vc2 = r.vc2.lock();
            (
                vc2.regs[VC2_REG_CURRENT_CURSOR_X as usize] as i32,
                // CURSOR_Y_LOC (0x03), NOT WORKING_CURSOR_Y (0x0d). The working
                // register is raster state: the refresh loop copies Y_LOC into
                // it at every VBLANK (`rex3.rs:4081`) and it drifts in between,
                // so a control loop reading it hunts against the scanline
                // rather than against the pointer. Measured on this rig: with
                // WORKING, Y took 49-80 rounds and landed 1-60 px out; with
                // Y_LOC, the same targets land in a handful of rounds. X has no
                // such twin — CURRENT_CURSOR_X is the position register.
                vc2.regs[VC2_REG_CURSOR_Y_LOC as usize] as i32,
            )
        };
        // `cursor_x_adjust` IS part of the answer: the compositor draws the
        // glyph at `reg - 31 + cursor_x_adjust` (`compositor.rs:128`), and on
        // the Indy's 1280x1024 VT timings that term is 5. Measured on the
        // framebuffer (rule 9): steering the register alone put the glyph 5 px
        // right of every commanded pixel; commanding `x - 5` put it exactly on
        // the pixel at 5 of 7 targets. Cached rather than read per verb, so a
        // lock lost to a compose pass cannot quietly shift the pointer.
        Some((
            rx + self.shared.cfg.cal_x + self.shared.x_adjust.load(Ordering::Relaxed),
            ry + self.shared.cfg.cal_y,
        ))
    }

    fn clamp_target(&self, x: i64, y: i64) -> (i32, i32) {
        (
            x.clamp(0, (self.shared.surf_w - 1) as i64) as i32,
            y.clamp(0, (self.shared.surf_h - 1) as i64) as i32,
        )
    }

    // ---- injection --------------------------------------------------------

    /// Forget everything this engine believes about the guest's pointer and
    /// buttons. Called after a restore or a machine reset — see the dispatch
    /// arm above.
    fn forget_guest_state(&mut self) {
        self.flight = None;
        self.bleedq.clear();
        self.clickq.clear();
        self.btnq.clear();
        self.gain = Gain::new();
        self.last_cur = (0, 0);
        // Buttons: the restored guest has none held. Tell the PS/2 device so
        // its own button byte matches, then clear ours.
        if self.buttons != 0 {
            self.buttons = 0;
            self.shared.ps2.push_mouse_input(0, 0, 0, 0);
        }
    }

    fn push_mouse(&self, dx: i32, dy: i32) {
        self.shared.ps2.push_mouse_input(self.buttons, dx, dy, 0);
    }

    // ---- verb dispatch ----------------------------------------------------

    fn exec(&mut self, cmd: PendingCmd) {
        let ack = AckRef {
            conn: cmd.conn,
            seq: cmd.seq,
        };
        if cmd.line.is_empty() {
            self.shared
                .reply_err(&ack, "badline", "empty or malformed request line");
            return;
        }
        let (verb, rest) = match cmd.line.split_once(' ') {
            Some((a, b)) => (a, b),
            None => (cmd.line.as_str(), ""),
        };
        let c = &self.shared.counters;
        match verb {
            "MOVE" => {
                let Some((dx, dy)) = two_longs(rest) else {
                    return self.shared.reply_err(&ack, "badarg", "MOVE dx dy");
                };
                c.move_v.fetch_add(1, Ordering::Relaxed);
                if !self.mouse_ready() {
                    c.gated.fetch_add(1, Ordering::Relaxed);
                    return self
                        .shared
                        .reply_err(&ack, "mouseoff", "guest AUX port disabled");
                }
                // MOVE applies immediately, unpaced — the ops-script verb.
                self.push_mouse(dx as i32, dy as i32);
                self.shared.reply_ok(&ack, "");
            }
            "MOVEP" => {
                let Some((dx, dy)) = two_longs(rest) else {
                    return self.shared.reply_err(&ack, "badarg", "MOVEP dx dy");
                };
                c.movep.fetch_add(1, Ordering::Relaxed);
                if !self.mouse_ready() {
                    c.gated.fetch_add(1, Ordering::Relaxed);
                    return self
                        .shared
                        .reply_err(&ack, "mouseoff", "guest AUX port disabled");
                }
                // Paced relative: an arbitrarily large jump bled out at
                // MOVE_STEP counts per window so no packet overflows and the
                // guest's 100 Hz sampler sees every count. Acks when DRAINED,
                // which is what makes it the calibration verb. Queued, never
                // slept on: this is the one thread that applies keys and button
                // edges too, and a 4000-count sweep must not freeze them.
                self.bleedq.push_back(BleedItem {
                    dx: dx as i32,
                    dy: dy as i32,
                    ack,
                });
            }
            "MOVEA" => {
                let Some((x, y)) = two_longs(rest) else {
                    return self.shared.reply_err(&ack, "badarg", "MOVEA x y");
                };
                c.movea.fetch_add(1, Ordering::Relaxed);
                if !self.mouse_ready() {
                    c.gated.fetch_add(1, Ordering::Relaxed);
                    return self
                        .shared
                        .reply_err(&ack, "mouseoff", "guest AUX port disabled");
                }
                if self.shared.rex3.is_none() {
                    return self.shared.reply_err(
                        &ack,
                        "nocursor",
                        "no REX3: MOVEA needs the VC2 cursor",
                    );
                }
                let (tx, ty) = self.clamp_target(x, y);
                // Latest-wins: a new target replaces the in-flight one, and the
                // old flight's EV is never emitted (the client already moved on).
                self.flight = Some(Flight {
                    tx,
                    ty,
                    seq: ack.seq.clone(),
                    rounds: 0,
                    last_issue: (0, 0),
                    last_read: self.reading().unwrap_or((0, 0)),
                    started: Instant::now(),
                });
                // MOVEA acks on ACCEPT; completion rides EV MOVEA.
                self.shared.reply_ok(&ack, "");
                self.shared
                    .trace(&format!("MOVEA seq={} tgt={},{}", ack.seq, tx, ty));
            }
            "DOWN1" | "UP1" | "DOWN2" | "UP2" | "DOWN3" | "UP3" => {
                let btn = verb.as_bytes()[verb.len() - 1] - b'1';
                let down = verb.starts_with('D');
                c.btn.fetch_add(1, Ordering::Relaxed);
                if !self.mouse_ready() {
                    c.gated.fetch_add(1, Ordering::Relaxed);
                    return self
                        .shared
                        .reply_err(&ack, "mouseoff", "guest AUX port disabled");
                }
                self.btnq.push_back(BtnItem { btn, down, ack });
            }
            "CLICK1" | "CLICK2" | "CLICK3" | "DCLICK1" => {
                let btn = if verb == "DCLICK1" {
                    0
                } else {
                    verb.as_bytes()[5] - b'1'
                };
                if !self.mouse_ready() {
                    c.gated.fetch_add(1, Ordering::Relaxed);
                    return self
                        .shared
                        .reply_err(&ack, "mouseoff", "guest AUX port disabled");
                }
                let mut steps = VecDeque::new();
                if verb == "DCLICK1" {
                    steps.push_back((true, 6));
                    steps.push_back((false, 8));
                }
                steps.push_back((true, 6));
                steps.push_back((false, 0));
                self.clickq.push_back(ClickItem { btn, steps, ack });
            }
            "KEY" => {
                // KEY <0|1> <port> <field>; the field is the rest of the line.
                let b = rest.as_bytes();
                if b.len() < 5 || (b[0] != b'0' && b[0] != b'1') || b[1] != b' ' {
                    return self
                        .shared
                        .reply_err(&ack, "badarg", "KEY <0|1> <port> <field>");
                }
                let down = b[0] == b'1';
                let Some(psp) = rest[2..].find(' ').map(|i| i + 2) else {
                    return self
                        .shared
                        .reply_err(&ack, "badarg", "KEY <0|1> <port> <field>");
                };
                let port = &rest[2..psp];
                let field = &rest[psp + 1..];
                if port != KBD_PORT {
                    return self.shared.reply_err(&ack, "nosuchport", port);
                }
                let Some(key) = keycode_by_name(field) else {
                    return self.shared.reply_err(&ack, "nosuchkey", field);
                };
                c.key.fetch_add(1, Ordering::Relaxed);
                if !self.kbd_ready() {
                    c.gated.fetch_add(1, Ordering::Relaxed);
                    return self.shared.reply_err(
                        &ack,
                        "kbdoff",
                        "guest KBD port disabled or not scanning",
                    );
                }
                self.kq.push_back(KeyItem { key, down, ack });
            }
            "KEYDUMP" => {
                for (n, _) in NAMES {
                    self.shared
                        .reply_data(&ack, &format!("{} | {}", KBD_PORT, n));
                }
                self.shared.reply_ok(&ack, &format!("{}", NAMES.len()));
            }
            "PING" => {
                self.shared.reply_ok(
                    &ack,
                    &format!(
                        "rx_us={} kq={} btnq={}",
                        cmd.rx.elapsed().as_micros(),
                        self.kq.len(),
                        self.btnq.len()
                    ),
                );
            }
            "CUR" => {
                // Read-only cursor probe with NO engine side effect: the
                // counterpart to MOVEP for calibration, because every MOVEA
                // runs a convergence and so cannot be the reader.
                match self.reading() {
                    Some((x, y)) => {
                        let (kb, ms) = self.shared.ps2.input_ready();
                        self.shared.reply_ok(
                            &ack,
                            &format!(
                                "x={} y={} trusted=1 gain={:.3},{:.3} kbd={} mouse={} xadj={}",
                                x,
                                y,
                                self.gain.x,
                                self.gain.y,
                                kb as u8,
                                ms as u8,
                                self.shared.x_adjust.load(Ordering::Relaxed)
                            ),
                        )
                    }
                    None => {
                        self.shared
                            .reply_err(&ack, "nocursor", "no REX3 in this configuration")
                    }
                }
            }
            "STAT" => {
                let line = self.stat_line();
                self.shared.reply_ok(&ack, &line);
            }
            "SYNC" => {
                // Everything ahead of this line has already been applied by the
                // time we get here (single engine thread, in-order drain), so a
                // SYNC ack is a real barrier.
                self.shared.reply_ok(&ack, "");
            }
            // ---- the reset plane, on this same socket ----------------
            // `SAVEST` `LOADST` `RESET` `FBSYNC` `CKPT` are `kh_ctl`'s, taken
            // through the merge hook it documents. They are applied HERE, on
            // the engine thread, so an `OK` still means "everything ahead of
            // this line has been applied and so has this" — the property
            // `mame_sock.rs` and `reset-tile.sh` both rely on.
            v if crate::kh_ctl::verb_owned(v) => {
                c.lifecycle.fetch_add(1, Ordering::Relaxed);
                // SAFETY: the pointer is valid for the process lifetime and
                // this is the single engine thread; every verb below stops the
                // machine's own threads before touching state.
                let m = unsafe { &mut *self.shared.machine.0 };
                let reply = crate::kh_ctl::dispatch(m, v, rest);
                // A restore rewinds the guest under us: the button mask, the
                // learned gain and any in-flight MOVEA describe a machine that
                // no longer exists, and steering by them would chase a cursor
                // that jumped. Drop the flight state and re-seed from the
                // registers on the next verb.
                if matches!(v, "LOADST" | "RESET") {
                    self.forget_guest_state();
                }
                match reply {
                    crate::kh_ctl::Reply::Ok(d) => self.shared.reply_ok(&ack, &d),
                    crate::kh_ctl::Reply::Err(code, text) => {
                        self.shared.reply_err(&ack, code, &text)
                    }
                }
            }
            // PAUSE / RESUME / EXIT are nobody's here: the daemon freezes this
            // station with SIGSTOP through SH_IDLE_PAUSE_PIDFILE and stops it
            // with the unit. Answering ERR rather than a silent OK keeps a
            // mis-wired station loud.
            _ => self.shared.reply_err(&ack, "badverb", verb),
        }
    }

    // ---- pacers -----------------------------------------------------------

    /// One window's worth of the head MOVEP bleed. Acks when the entry is
    /// fully drained.
    fn step_bleed(&mut self) {
        if Instant::now() < self.bleed_at {
            return;
        }
        let step = self.shared.cfg.move_step;
        let (sx, sy) = {
            let Some(item) = self.bleedq.front_mut() else {
                return;
            };
            let sx = item.dx.clamp(-step, step);
            let sy = item.dy.clamp(-step, step);
            item.dx -= sx;
            item.dy -= sy;
            (sx, sy)
        };
        self.push_mouse(sx, sy);
        let drained = self
            .bleedq
            .front()
            .map(|i| i.dx == 0 && i.dy == 0)
            .unwrap_or(false);
        if drained {
            let done = self.bleedq.pop_front().unwrap();
            self.shared.reply_ok(&done.ack, "");
        }
        self.bleed_at = Instant::now() + self.shared.cfg.move_window;
    }

    /// Key edge pacing. Unlike a scanned MAME matrix there is no field to hold
    /// across a scan — a PS/2 keyboard is a lossless queue of make/break bytes
    /// — so this module NEVER synthesises a release: the client's own `KEY 0`
    /// is the release, and **every edge acks the moment it is applied**, which
    /// is the contract `mame_sock.rs` is written against.
    ///
    /// What the pacing buys instead is spacing. A browser sends a line as one
    /// burst; dumped into the queue at zero spacing that arrives at the guest
    /// as an impossible typing speed, and X's autorepeat and IRIX's own
    /// keyboard driver both key off inter-edge timing. So a DOWN is followed by
    /// at least `key_hold` before the next edge and an UP by at least
    /// `key_gap` — the station's 40/40.
    fn step_keys(&mut self) {
        let cfg = self.shared.cfg;
        let now = Instant::now();
        if now < self.key_next_at {
            return;
        }
        let Some(item) = self.kq.front() else { return };
        // EXCL: never hold two keys at once. Off by default here (a PS/2 queue
        // has no matrix to alias into a chord) and available for a guest that
        // scans its own.
        if cfg.key_excl && item.down && self.key_held.is_some() {
            return;
        }
        let item = self.kq.pop_front().unwrap();
        if !self.kbd_ready() {
            self.shared.counters.gated.fetch_add(1, Ordering::Relaxed);
            self.shared.reply_err(
                &item.ack,
                "kbdoff",
                "guest KBD port disabled or not scanning",
            );
            return;
        }
        self.shared.ps2.push_kb(item.key, item.down);
        if item.down {
            self.key_held = Some(item.key);
            self.key_next_at = now + cfg.key_hold;
        } else {
            if self.key_held == Some(item.key) {
                self.key_held = None;
            }
            self.key_next_at = now + cfg.key_gap;
        }
        self.shared.trace(&format!(
            "KEY seq={} {:?} down={} applied",
            item.ack.seq, item.key, item.down
        ));
        self.shared.reply_ok(&item.ack, "");
    }

    fn step_clicks(&mut self) {
        if Instant::now() < self.click_at {
            return;
        }
        let Some(item) = self.clickq.front_mut() else {
            return;
        };
        let Some((down, frames)) = item.steps.pop_front() else {
            let done = self.clickq.pop_front().unwrap();
            self.shared.reply_ok(&done.ack, "");
            return;
        };
        let btn = item.btn;
        let bit = 1u8 << btn;
        if down {
            self.buttons |= bit;
        } else {
            self.buttons &= !bit;
        }
        self.push_mouse(0, 0);
        self.click_at = Instant::now() + Duration::from_millis(u64::from(frames) * 16);
    }

    /// One convergence round of the in-flight MOVEA.
    fn step_flight(&mut self) {
        let cfg = self.shared.cfg;
        if self.flight.is_some() && Instant::now() < self.next_round {
            // SETTLE. The emulated PS/2 mouse samples at the guest-programmed
            // rate (100 Hz) and the VC2 cursor Y latches at VBLANK, so counts
            // issued now are not visible in the registers for tens of ms. A
            // loop that re-reads faster than that reads its own past, keeps
            // correcting an error it has already fixed, and integrator-winds
            // the pointer into a screen edge — measured on this rig at a 5 ms
            // round: every target ran away to a corner and gave up at 80
            // rounds. One round per settle window is the fix.
            return;
        }
        let Some(mut f) = self.flight.take() else {
            return;
        };
        let Some((cx, cy)) = self.reading() else {
            self.shared
                .emit_ev(&format!("MOVEA seq={} err=nocursor", f.seq));
            return;
        };
        self.last_cur = (cx, cy);

        // Learn from what the previous round's counts actually did. A round
        // that issued counts and observed nothing decays the gain estimate
        // rather than zeroing it, so a transient stall cannot make the engine
        // slam the pointer on the next round.
        if f.rounds > 0 {
            self.gain
                .learn(f.last_issue, (cx - f.last_read.0, cy - f.last_read.1));
        }

        let ex = f.tx - cx;
        let ey = f.ty - cy;
        if ex.abs() <= cfg.deadband && ey.abs() <= cfg.deadband {
            self.shared
                .counters
                .movea_converged
                .fetch_add(1, Ordering::Relaxed);
            self.shared.emit_ev(&format!(
                "MOVEA seq={} x={} y={} rounds={} ms={}",
                f.seq,
                cx,
                cy,
                f.rounds,
                f.started.elapsed().as_millis()
            ));
            self.shared.trace(&format!(
                "MOVEA seq={} landed {},{} rounds={} ms={}",
                f.seq,
                cx,
                cy,
                f.rounds,
                f.started.elapsed().as_millis()
            ));
            return; // flight cleared: deferred button edges are now free
        }
        if f.rounds >= cfg.movea_rounds {
            self.shared
                .counters
                .movea_giveup
                .fetch_add(1, Ordering::Relaxed);
            self.shared.emit_ev(&format!(
                "MOVEA seq={} err=noconverge x={} y={} want={},{} rounds={}",
                f.seq, cx, cy, f.tx, f.ty, f.rounds
            ));
            return; // give up loudly; edges stop being deferred
        }

        // Size the issue against gain*margin, so a full-gain guest can never be
        // pushed past the target (the user rule: never extrapolate the cursor
        // ahead of real movement).
        // IRIX applies pointer acceleration only ABOVE a threshold (4 px by
        // default) — below it the guest is exactly 1:1. So the final approach
        // states the residual as counts directly and lands on the pixel, while
        // the long haul divides by the learned gain. This is why a 1 px
        // deadband is reachable here even though the desktop is running ~1.8x
        // acceleration: the loop never needs to undo the acceleration, only to
        // stop using it.
        let issue = |err: i32, g: f64| -> i32 {
            if err.abs() <= cfg.accel_threshold {
                return err;
            }
            let want = err as f64 / (g * cfg.gain_margin).max(0.05);
            let c = want.trunc() as i32;
            let c = if c.abs() <= cfg.accel_threshold {
                err.signum() * (cfg.accel_threshold + 1)
            } else {
                c
            };
            c.clamp(-cfg.move_step, cfg.move_step)
        };
        let sx = issue(ex, self.gain.x);
        let sy = issue(ey, self.gain.y);
        self.push_mouse(sx, sy);

        f.last_issue = (sx, sy);
        f.last_read = (cx, cy);
        f.rounds += 1;
        self.next_round = Instant::now() + cfg.settle;
        self.flight = Some(f);
    }

    /// Button edges are deferred behind an in-flight MOVEA, so a click always
    /// fires at the target and never at the pointer's old position.
    fn step_btns(&mut self) {
        if self.flight.is_some() {
            return;
        }
        while let Some(item) = self.btnq.pop_front() {
            let bit = 1u8 << item.btn;
            if item.down {
                self.buttons |= bit;
            } else {
                self.buttons &= !bit;
            }
            self.push_mouse(0, 0);
            self.shared.counters.btn.fetch_add(1, Ordering::Relaxed);
            self.shared.trace(&format!(
                "BTN seq={} btn={} down={}",
                item.ack.seq, item.btn, item.down
            ));
            self.shared.reply_ok(&item.ack, "");
        }
    }

    fn step_stats(&mut self) {
        if Instant::now() < self.next_stat {
            return;
        }
        self.next_stat = Instant::now() + self.shared.cfg.stat_period;
        // Refresh the cached VT-timing correction. try_lock: a compose pass in
        // flight just means we keep the value we already have.
        if let Some(r) = self.shared.rex3.as_ref() {
            if let Some(sc) = r.screen.try_lock() {
                self.shared
                    .x_adjust
                    .store(sc.cursor_x_adjust, Ordering::Relaxed);
            }
        }
        if !self.shared.conns.lock().is_empty() {
            let line = self.stat_line();
            self.shared.emit_ev(&format!("STATS {}", line));
        }
    }

    fn stat_line(&self) -> String {
        let c = &self.shared.counters;
        let (kb, ms) = self.shared.ps2.input_ready();
        format!(
            "move={} movep={} movea={} conv={} giveup={} key={} btn={} err={} gated={} \
             life={} cur={},{} gain={:.3},{:.3} buttons={:#04x} kq={} btnq={} kbd={} mouse={} screen={}x{} xadj={}",
            c.move_v.load(Ordering::Relaxed),
            c.movep.load(Ordering::Relaxed),
            c.movea.load(Ordering::Relaxed),
            c.movea_converged.load(Ordering::Relaxed),
            c.movea_giveup.load(Ordering::Relaxed),
            c.key.load(Ordering::Relaxed),
            c.btn.load(Ordering::Relaxed),
            c.errs.load(Ordering::Relaxed),
            c.gated.load(Ordering::Relaxed),
            c.lifecycle.load(Ordering::Relaxed),
            self.last_cur.0,
            self.last_cur.1,
            self.gain.x,
            self.gain.y,
            self.buttons,
            self.kq.len(),
            self.btnq.len(),
            kb as u8,
            ms as u8,
            self.shared.surf_w,
            self.shared.surf_h,
            self.shared.x_adjust.load(Ordering::Relaxed),
        )
    }
}

fn two_longs(s: &str) -> Option<(i64, i64)> {
    let mut it = s.split_whitespace();
    let a = it.next()?.parse::<i64>().ok()?;
    let b = it.next()?.parse::<i64>().ok()?;
    Some((a, b))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_name_round_trips() {
        for (n, k) in NAMES {
            assert_eq!(keycode_by_name(n), Some(*k), "name {n} does not resolve");
        }
        assert_eq!(keycode_by_name("NoSuchKey"), None);
    }

    #[test]
    fn names_are_unique() {
        let mut seen: Vec<&str> = NAMES.iter().map(|(n, _)| *n).collect();
        let before = seen.len();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(before, seen.len(), "duplicate name in NAMES");
    }

    #[test]
    fn two_longs_parses_the_wire() {
        assert_eq!(two_longs("640 512"), Some((640, 512)));
        assert_eq!(two_longs("-3 4"), Some((-3, 4)));
        assert_eq!(two_longs("640"), None);
        assert_eq!(two_longs(""), None);
    }

    #[test]
    fn gain_never_collapses_on_a_stalled_round() {
        let mut g = Gain::new();
        for _ in 0..20 {
            g.learn((100, 100), (0, 0)); // observed nothing at all
        }
        assert!(
            g.x >= 0.1 && g.y >= 0.1,
            "gain collapsed to {},{}",
            g.x,
            g.y
        );
    }

    #[test]
    fn gain_learns_a_real_ratio() {
        let mut g = Gain::new();
        for _ in 0..30 {
            g.learn((100, 100), (200, 50)); // guest doubles X, halves Y
        }
        assert!((g.x - 2.0).abs() < 0.1, "x gain {}", g.x);
        assert!((g.y - 0.5).abs() < 0.1, "y gain {}", g.y);
    }
}
