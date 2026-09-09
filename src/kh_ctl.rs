//! kernel-hive RESET plane — `mamectl/1` verbs `SAVEST` / `LOADST` / `RESET`
//! / `FBSYNC` / `CKPT`, plus the startup restore that replaces a cold boot.
//!
//! WHY THIS EXISTS. The `indyr4400` station's reset is a QEMU `loadvm golden`
//! today, because Iris runs inside a Debian kiosk. Host-native there is no
//! QEMU, and streamhost's reset path (`scripts/serve/reset-tile.sh`, the
//! `relaunch` branch) already knows how to reset a host-native emulator:
//! it speaks `mamectl/1` at the station's control socket and sends
//! `LOADST golden`. Iris already has a complete snapshot/rollback stack
//! (`Machine::save_snapshot` / `ci_restore` / `ci_rollback`, CHANGELOG
//! "System Snapshot"); this module is the adapter between the two, so the
//! station's reset becomes an in-process restore (~0.1 s) instead of a
//! ~7-minute cold boot, and streamhost needs no new code.
//!
//! ── MERGE HOOK (stream C owns the socket) ──────────────────────────────
//! The `mamectl/1` listener that carries the INPUT verbs (`MOVEA`, `DOWNn`,
//! `UPn`, `KEY`) is stream C's. It owns the socket, the banner, the seq
//! framing and the emulation-thread drain. To fold this module in, C routes
//! any verb it does not own to
//!
//!     kh_ctl::verb_owned(verb)                -> bool
//!     kh_ctl::dispatch(&mut Machine, verb, rest) -> Reply
//!
//! from its own drain (so an OK still means "the emulation thread applied
//! it"), and adds `caps_fragment()` to its HELLO caps list. Everything below
//! that pair is stream D's and needs no other seam.
//!
//! Until C's listener exists this module can carry the socket itself:
//! `IRIS_KH_CTL_SOCK=<path>` binds a minimal `mamectl/1` server that speaks
//! the same wire (banner, `<seq> VERB`, `<seq> OK|ERR`) and serves only the
//! reset verbs. `/root/mctl.py` drives it unchanged. When C lands, drop
//! `start_server` and keep `dispatch`.
//!
//! ── THE PROVENANCE TRIPLE (AGENTS.md rule 6) ──────────────────────────
//! A checkpoint, the emulator binary and the device set are ONE combination.
//! Iris's own `snapshot.toml` already refuses a restore whose cargo features,
//! CPU model or disk id/size differ (`machine.rs` `load_snapshot_inner`), and
//! *warns* on an `iris_git_rev` difference. A warning is not a guard: two
//! builds of different commits with the same feature set restore each other's
//! state silently, and in the lab the binary is one third of the checkpoint.
//!
//! So `SAVEST` writes a sidecar, `kh-provenance.toml`, next to the manifest,
//! recording the binary that captured the state (path, size, mtime, BLAKE3),
//! its feature set and the configured device set; `LOADST` refuses a restore
//! whose binary hash differs. `KH_PROVENANCE=warn` downgrades the refusal to
//! a log line (for a deliberate binary bump whose state is known good);
//! `KH_PROVENANCE=off` skips the check entirely. Both say so loudly.
//!
//! ── THE STALE-PAGE TRAP ────────────────────────────────────────────────
//! `docs/guests/nextstep.md` §5.3: after a restore, a frame publisher whose
//! private shadow already matches the restored pixels publishes nothing, and
//! the daemon streams the pre-restore picture forever under a live guest. So
//! every path here that changes machine state calls `fbsync()` unconditionally
//! afterwards — never conditionally, never "if the frame changed".
//! `fbsync()` is a no-op until stream A registers its republisher through
//! `set_fbsync_hook`.
//!
//! ── WRITE-TEMP-THEN-RENAME ─────────────────────────────────────────────
//! `docs/lab/checkpoint-guard.md` §"Runtimes covered, and refused" names the
//! shape a savestate runtime must use: write the new state under a temp name,
//! prove it, then `mv` it over the old one — an atomic rename with no window
//! at all. `SAVEST golden` therefore writes `saves/golden.new`, then renames
//! `golden` → `golden.prev` and `golden.new` → `golden`. A crash mid-save
//! leaves the old `golden` untouched. (The CAS chunk store under `saves/.cas`
//! is content-addressed and shared, so the rename moves only the manifests.)

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::OnceLock;
use std::thread;
use std::time::Instant;

use parking_lot::Mutex;

use crate::machine::Machine;

/// Protocol id — byte-identical to MAME's ctlsock so `mctl.py`, the daemon's
/// `mame_sock.rs` and `reset-tile.sh` need no special case.
const PROTO_ID: &str = "mamectl/1";

// ---------------------------------------------------------------------------
// Reply
// ---------------------------------------------------------------------------

/// One verb's answer, in ctlsock's vocabulary. `Err` codes are ctlsock's:
/// `badline | badverb | badarg | noport | unsupported | busy`.
pub enum Reply {
    Ok(String),
    Err(&'static str, String),
}

impl Reply {
    fn ok() -> Self {
        Reply::Ok(String::new())
    }
    /// Render onto the wire under `seq`. `seq == "-"` is fire-and-forget.
    pub fn render(&self, seq: &str) -> Option<String> {
        if seq == "-" {
            return None;
        }
        Some(match self {
            Reply::Ok(d) if d.is_empty() => format!("{} OK\n", seq),
            Reply::Ok(d) => format!("{} OK {}\n", seq, d),
            Reply::Err(code, text) => format!("{} ERR {} {}\n", seq, code, text),
        })
    }
}

// ---------------------------------------------------------------------------
// FBSYNC hook — stream A's republisher, registered at startup
// ---------------------------------------------------------------------------

type FbSyncHook = Box<dyn Fn() + Send + Sync + 'static>;
static FBSYNC_HOOK: OnceLock<FbSyncHook> = OnceLock::new();

/// Stream A calls this once, right after it installs the IFB1 publisher into
/// `rex3.renderer`. The hook must republish ONE WHOLE FRAME unconditionally
/// (full-surface dirty rect, seqlock bumped) regardless of what the
/// publisher's shadow believes. Registering twice is a no-op.
pub fn set_fbsync_hook(f: FbSyncHook) {
    let _ = FBSYNC_HOOK.set(f);
}

/// Republish one whole frame. Returns whether a publisher was registered —
/// reported on the wire so a station that silently has no frame plane is
/// visible in the ack, not only in a log nobody reads.
pub fn fbsync() -> bool {
    match FBSYNC_HOOK.get() {
        Some(f) => {
            f();
            true
        }
        None => false,
    }
}

// ---------------------------------------------------------------------------
// Provenance sidecar
// ---------------------------------------------------------------------------

const PROVENANCE_FILE: &str = "kh-provenance.toml";

/// Identity of the binary that captured (or is loading) a snapshot.
struct BinaryId {
    path: String,
    size: u64,
    blake3: String,
}

fn binary_id() -> Result<BinaryId, String> {
    let path = std::fs::read_link("/proc/self/exe")
        .map_err(|e| format!("readlink /proc/self/exe: {}", e))?;
    // A REPLACED binary reads back as "<path> (deleted)"; keep the raw string
    // so a mismatch says which one it was (DEBRIDGE-HANDOVER §Lessons 7).
    let disp = path.to_string_lossy().to_string();
    let bytes = std::fs::read(&path)
        .map_err(|e| format!("read {}: {}", disp, e))?;
    Ok(BinaryId {
        path: disp,
        size: bytes.len() as u64,
        blake3: blake3::hash(&bytes).to_hex().to_string(),
    })
}

/// The binary hash is computed once — the file is 64-70 MB and a `LOADST` on
/// the reset path must not pay 70 MB of I/O per press.
fn binary_id_cached() -> Result<&'static BinaryId, String> {
    static CACHE: OnceLock<Result<BinaryId, String>> = OnceLock::new();
    match CACHE.get_or_init(binary_id) {
        Ok(b) => Ok(b),
        Err(e) => Err(e.clone()),
    }
}

fn saves_dir(name: &str) -> PathBuf {
    // `Machine::save_snapshot` / `load_snapshot` resolve "saves/<name>"
    // relative to the PROCESS CWD. The launcher must therefore cd to the
    // station directory before exec'ing iris; this module uses the same
    // relative root deliberately so the two can never disagree.
    PathBuf::from("saves").join(name)
}

fn write_provenance(dir: &Path) -> Result<(), String> {
    let b = binary_id_cached()?;
    let features = crate::snapshot::enabled_features().join(",");
    let argv: Vec<String> = std::env::args().collect();
    let body = format!(
        "# kernel-hive provenance sidecar — written by SAVEST, checked by LOADST.\n\
         # AGENTS.md rule 6: checkpoint + binary + device set are ONE combination.\n\
         # A mismatch is refused unless KH_PROVENANCE=warn|off.\n\
         binary_path = \"{}\"\n\
         binary_size = {}\n\
         binary_blake3 = \"{}\"\n\
         features = \"{}\"\n\
         iris_git_rev = \"{}\"\n\
         argv = \"{}\"\n\
         created_at_unix = {}\n",
        b.path,
        b.size,
        b.blake3,
        features,
        option_env!("IRIS_GIT_REV").unwrap_or("unknown"),
        argv.join(" ").replace('"', "'"),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
    );
    std::fs::write(dir.join(PROVENANCE_FILE), body)
        .map_err(|e| format!("write {}: {}", PROVENANCE_FILE, e))
}

fn field<'a>(body: &'a str, key: &str) -> Option<&'a str> {
    body.lines()
        .find_map(|l| l.strip_prefix(key)?.trim_start().strip_prefix('=').map(str::trim))
        .map(|v| v.trim_matches('"'))
}

/// `off` = no check, `warn` = log and continue, anything else = refuse.
fn provenance_mode() -> String {
    std::env::var("KH_PROVENANCE").unwrap_or_else(|_| "strict".into())
}

/// Check `<snapshot>/kh-provenance.toml` against this binary. `Ok(note)` is a
/// line for the ack; `Err(msg)` refuses the restore.
fn check_provenance(dir: &Path, name: &str) -> Result<String, String> {
    let mode = provenance_mode();
    if mode == "off" {
        return Ok("prov=off".into());
    }
    let p = dir.join(PROVENANCE_FILE);
    let body = match std::fs::read_to_string(&p) {
        Ok(b) => b,
        Err(_) => {
            // A snapshot captured before this module existed, or one baked by
            // hand. Iris's own manifest still guards features/disks/CPU, so
            // this is a warning, not a refusal — but it is a LOUD one, because
            // the binary leg of the triple is missing.
            eprintln!(
                "KH-RESET: snapshot '{}' has no {} — the binary leg of the provenance \
                 triple is UNCHECKED for this restore (rule 6). Recapture with SAVEST.",
                name, PROVENANCE_FILE
            );
            return Ok("prov=absent".into());
        }
    };
    let want = match field(&body, "binary_blake3") {
        Some(h) => h,
        None => return Ok("prov=malformed".into()),
    };
    let have = binary_id_cached()?;
    if want == have.blake3 {
        return Ok("prov=ok".into());
    }
    let msg = format!(
        "provenance mismatch on '{}': snapshot was captured by binary {} ({} bytes, blake3 {}) \
         but this process is {} (blake3 {}). golden + binary + device set are ONE combination \
         (rule 6). Recapture the checkpoint, or set KH_PROVENANCE=warn if the state is known good.",
        name,
        field(&body, "binary_path").unwrap_or("?"),
        field(&body, "binary_size").unwrap_or("?"),
        &want[..want.len().min(16)],
        have.path,
        &have.blake3[..16],
    );
    if mode == "warn" {
        eprintln!("KH-RESET: {} [KH_PROVENANCE=warn — continuing anyway]", msg);
        return Ok("prov=warn".into());
    }
    Err(msg)
}

// ---------------------------------------------------------------------------
// The verbs
// ---------------------------------------------------------------------------

/// Verbs this module answers. Stream C's listener asks this before falling
/// through to `badverb`.
pub fn verb_owned(verb: &str) -> bool {
    matches!(verb, "SAVEST" | "LOADST" | "RESET" | "FBSYNC" | "CKPT")
}

/// The fragment stream C appends to its HELLO `caps=` list.
pub fn caps_fragment() -> &'static str {
    "savest,loadst,reset,fbsync"
}

/// Apply one verb. MUST be called on the thread that owns the machine
/// (stream C's emulation drain, or this module's own handler under its
/// mutex): an `OK` means the state change has completed, which is what
/// `reset-tile.sh` and `mame_sock.rs` both assume.
pub fn dispatch(m: &mut Machine, verb: &str, rest: &str) -> Reply {
    match verb {
        "SAVEST" => do_savest(m, rest.trim()),
        "LOADST" => do_loadst(m, rest.trim()),
        "RESET" => do_reset(m),
        "FBSYNC" => {
            let had = fbsync();
            Reply::Ok(format!("published={}", u8::from(had)))
        }
        "CKPT" => do_ckpt(rest.trim()),
        other => Reply::Err("badverb", other.to_string()),
    }
}

/// `SAVEST <name>` — capture a checkpoint under a temp name, stamp its
/// provenance, then rename it over the old one.
fn do_savest(m: &mut Machine, name: &str) -> Reply {
    if name.is_empty() || name.contains('/') || name.starts_with('.') {
        return Reply::Err("badarg", "SAVEST <name> (one path segment)".into());
    }
    let staging = format!("{}.new", name);
    let staging_dir = saves_dir(&staging);
    let _ = std::fs::remove_dir_all(&staging_dir);

    let t0 = Instant::now();
    if let Err(e) = m.save_snapshot(&staging) {
        let _ = std::fs::remove_dir_all(&staging_dir);
        return Reply::Err("busy", format!("save failed: {}", e));
    }
    if let Err(e) = write_provenance(&staging_dir) {
        let _ = std::fs::remove_dir_all(&staging_dir);
        return Reply::Err("busy", format!("provenance: {}", e));
    }
    // Promote. The previous state is kept one generation back so a bad bake
    // is one `mv` from being undone — the checkpoint-guard idiom.
    let final_dir = saves_dir(name);
    let prev_dir = saves_dir(&format!("{}.prev", name));
    if final_dir.exists() {
        let _ = std::fs::remove_dir_all(&prev_dir);
        if let Err(e) = std::fs::rename(&final_dir, &prev_dir) {
            return Reply::Err("busy", format!("park old '{}': {}", name, e));
        }
    }
    if let Err(e) = std::fs::rename(&staging_dir, &final_dir) {
        // Put the old one back rather than leaving the station with no golden.
        let _ = std::fs::rename(&prev_dir, &final_dir);
        return Reply::Err("busy", format!("promote '{}': {}", name, e));
    }
    let ms = t0.elapsed().as_millis();
    // A save stops and restarts the CPU and rewrites the framebuffer chunk
    // manifest; republish so a publisher shadow cannot go stale over it.
    let had = fbsync();
    Reply::Ok(format!(
        "ms={} name={} prev={}.prev fbsync={}",
        ms,
        name,
        name,
        u8::from(had)
    ))
}

/// `LOADST <name>` — restore. Prefers the in-memory rollback checkpoint when
/// it describes the SAME snapshot (Iris measures that at ~42 ms against a
/// disk restore's hundreds), and falls back to the disk path otherwise.
fn do_loadst(m: &mut Machine, name: &str) -> Reply {
    if name.is_empty() {
        return Reply::Err("badarg", "LOADST <name>".into());
    }
    let dir = saves_dir(name);
    if !dir.join("snapshot.toml").exists() && !dir.join("cpu.toml").exists() {
        return Reply::Err("badarg", format!("no snapshot at saves/{}", name));
    }
    let note = match check_provenance(&dir, name) {
        Ok(n) => n,
        Err(e) => return Reply::Err("badarg", e),
    };

    let t0 = Instant::now();
    // `ci_rollback` replays the in-memory image captured at the last
    // `ci_restore`. It is only the right answer when that restore was of THIS
    // snapshot — otherwise it would rewind to a different state under the same
    // name, which is the worst possible kind of correct-looking bug.
    let fast = m.last_restore_name().as_deref() == Some(name) && m.has_rollback_checkpoint();
    let (r, via) = if fast {
        (m.ci_rollback(), "rollback")
    } else {
        (m.ci_restore(name), "restore")
    };
    if let Err(e) = r {
        return Reply::Err("badarg", format!("{} failed: {}", via, e));
    }
    let ms = t0.elapsed().as_millis();
    // UNCONDITIONAL: see the stale-page trap at the top of this file.
    let had = fbsync();
    Reply::Ok(format!(
        "ms={} via={} {} fbsync={}",
        ms,
        via,
        note,
        u8::from(had)
    ))
}

/// `RESET` — roll back to the state the launcher restored at startup. This is
/// what the SPA's reset button and `labctl reset` reach, through
/// `reset-tile.sh`'s `relaunch` branch. No name, no disk read.
fn do_reset(m: &mut Machine) -> Reply {
    let t0 = Instant::now();
    let had_cp = m.has_rollback_checkpoint();
    if let Err(e) = m.ci_rollback() {
        return Reply::Err("badarg", format!("rollback failed: {}", e));
    }
    let ms = t0.elapsed().as_millis();
    let had = fbsync();
    Reply::Ok(format!(
        "ms={} via={} fbsync={}",
        ms,
        if had_cp { "rollback" } else { "disk" },
        u8::from(had)
    ))
}

/// `CKPT [name]` — report what the reset plane would do, without doing it.
/// The launcher and a human both need to answer "is there a checkpoint, does
/// it match this binary" without mutating the machine.
fn do_ckpt(name: &str) -> Reply {
    let name = if name.is_empty() { "golden" } else { name };
    let dir = saves_dir(name);
    let present = dir.join("snapshot.toml").exists();
    let prov = if !present {
        "absent".to_string()
    } else {
        match check_provenance(&dir, name) {
            Ok(n) => n.trim_start_matches("prov=").to_string(),
            Err(_) => "mismatch".to_string(),
        }
    };
    let b = binary_id_cached()
        .map(|b| b.blake3[..16].to_string())
        .unwrap_or_else(|_| "?".into());
    Reply::Ok(format!(
        "name={} present={} prov={} binary={} features={} cwd={}",
        name,
        u8::from(present),
        prov,
        b,
        crate::snapshot::enabled_features().join(","),
        std::env::current_dir()
            .map(|p| p.to_string_lossy().to_string())
            .unwrap_or_else(|_| "?".into()),
    ))
}

// ---------------------------------------------------------------------------
// Startup restore — what replaces `-loadvm golden`
// ---------------------------------------------------------------------------

/// Iris has no CLI flag that restores a snapshot at startup, so a host-native
/// station would cold-boot IRIX (~7 min: PROM, autoconfig relink, login) on
/// every launch. `IRIS_STATE=<name>` restores instead, in-process, before the
/// station's first visitor.
///
/// `IRIS_STATE=` (empty) or unset forces a cold boot — the deliberate rollback
/// lever, exactly as `IRIX_STATE=` is on the `irix` station.
///
/// The BOUNDED FALLBACK is deliberately loud and deliberately NOT silent
/// recovery: a failed restore logs `KH-RESET: cold boot` with the reason, and
/// the launcher counts the failures (see `kh-reset.sh`, `kh_reset_attempt`) so
/// two failed restore launches in a row make the third cold-boot on purpose
/// rather than by accident.
pub fn startup_restore(m: &mut Machine) {
    let name = match std::env::var("IRIS_STATE") {
        Ok(n) if !n.trim().is_empty() => n.trim().to_string(),
        _ => {
            eprintln!("KH-RESET: IRIS_STATE unset — cold boot (IRIX autoconfig, ~7 min)");
            return;
        }
    };
    let t0 = Instant::now();
    match dispatch(m, "LOADST", &name) {
        Reply::Ok(d) => eprintln!(
            "KH-RESET: restored '{}' at startup in {} ms ({})",
            name,
            t0.elapsed().as_millis(),
            d
        ),
        Reply::Err(code, msg) => eprintln!(
            "KH-RESET: startup restore of '{}' FAILED ({}: {}) — COLD BOOTING. \
             IRIX will redo its autoconfig relink (~7 min) because the COW overlay's \
             .dirty sidecar is only flushed on a clean exit.",
            name, code, msg
        ),
    }
}

// ---------------------------------------------------------------------------
// Standalone listener (temporary — stream C's listener supersedes it)
// ---------------------------------------------------------------------------

struct MachinePtr(*mut Machine);
unsafe impl Send for MachinePtr {}
unsafe impl Sync for MachinePtr {}

struct Server {
    machine: Arc<Mutex<MachinePtr>>,
    banner: String,
}

/// Bind `IRIS_KH_CTL_SOCK` and serve the reset verbs over `mamectl/1`.
///
/// # Safety
/// `machine_ptr` must stay valid for the process lifetime — pass the same
/// pointer `main` hands to `iris::ci::start_server`.
pub unsafe fn start_server(machine_ptr: *mut Machine) -> Result<(), String> {
    let path = match std::env::var("IRIS_KH_CTL_SOCK") {
        Ok(p) if !p.is_empty() => p,
        _ => return Ok(()),
    };
    let (w, h) = (*machine_ptr)
        .get_rex3()
        .map(|r| r.display_size())
        .unwrap_or((1280, 1024));
    let banner = format!(
        "HELLO {} iris-{} indy caps={} screen={}x{}\n",
        PROTO_ID,
        option_env!("IRIS_GIT_REV").unwrap_or("unknown"),
        caps_fragment(),
        w,
        h
    );
    let _ = std::fs::remove_file(&path);
    let listener = UnixListener::bind(&path).map_err(|e| format!("bind {}: {}", path, e))?;
    let server = Arc::new(Server {
        machine: Arc::new(Mutex::new(MachinePtr(machine_ptr))),
        banner,
    });
    eprintln!("KH-RESET: mamectl/1 reset socket listening at {}", path);
    thread::Builder::new()
        .name("kh-ctl-accept".into())
        .spawn(move || {
            for conn in listener.incoming().flatten() {
                let s = server.clone();
                thread::Builder::new()
                    .name("kh-ctl-conn".into())
                    .spawn(move || serve(s, conn))
                    .ok();
            }
        })
        .map_err(|e| format!("spawn kh-ctl-accept: {}", e))?;
    Ok(())
}

fn serve(server: Arc<Server>, stream: UnixStream) {
    let mut out = match stream.try_clone() {
        Ok(s) => s,
        Err(_) => return,
    };
    if out.write_all(server.banner.as_bytes()).is_err() {
        return;
    }
    let _ = out.flush();
    for line in BufReader::new(stream).lines().map_while(Result::ok) {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        // `<seq> VERB [args]`; seq "-" is fire-and-forget.
        let (seq, tail) = match line.split_once(' ') {
            Some((s, t)) => (s, t.trim()),
            None => (line, ""),
        };
        let (verb, rest) = match tail.split_once(' ') {
            Some((v, r)) => (v, r),
            None => (tail, ""),
        };
        if verb.is_empty() {
            if let Some(w) = Reply::Err("badline", line.to_string()).render(seq) {
                let _ = out.write_all(w.as_bytes());
            }
            continue;
        }
        let reply = if verb == "PING" {
            Reply::ok()
        } else if verb == "QUIT" {
            let _ = Reply::ok().render(seq).map(|w| out.write_all(w.as_bytes()));
            return;
        } else {
            let mut guard = server.machine.lock();
            // SAFETY: the pointer is valid for the process lifetime and this
            // mutex serialises every machine access from this module. Each
            // verb here stops the machine's threads itself before touching
            // state (save_snapshot / ci_restore / ci_rollback all do), so an
            // OK means applied — the property `mame_sock.rs` relies on.
            let m = unsafe { &mut *(guard.0) };
            dispatch(m, verb, rest)
        };
        if let Some(w) = reply.render(seq) {
            if out.write_all(w.as_bytes()).is_err() {
                return;
            }
            let _ = out.flush();
        }
    }
}
