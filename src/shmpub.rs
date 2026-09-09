//! IFB1 shared-memory frame publisher — the host-native frame plane.
//!
//! # Why this exists
//!
//! Upstream Iris has exactly one place that installs a `rex3::Renderer`:
//! `src/ui.rs`, on the windowed path. Under `--ci` (no window) `rex3.renderer`
//! stays `None`, `present()` is never called, and the "REX3 rendering to
//! offscreen buffer" banner in `main.rs` is not true — nothing composites and
//! `screen.rgba` stays zeroed.
//!
//! In the kernel-hive lab the emulator is not driven by a window: the streaming
//! daemon reads finished frames out of a file-backed mapping (`SH_CAPTURE=shm`)
//! and forwards them to the browser. Every layer in between — a guest X server,
//! llvmpipe, a D-Bus capture hop — is pure cost; deleting it is the whole point
//! of the host-native conversion (AGENTS.md rule 13).
//!
//! So this module is the missing renderer. On the no-window branch, when
//! `IRIS_SHM_PATH` is set, `install()` puts a `ShmPublisher` into
//! `rex3.renderer`. It drives the same CPU `SwCompositor` the windowed path and
//! `iris-gui` both use, and writes the composited frame straight into a mapped
//! file in the consumer's wire format.
//!
//! **With `IRIS_SHM_PATH` unset nothing in this module runs and the binary
//! behaves exactly as upstream.** That is deliberate: the live station keeps
//! running off the same source until cutover.
//!
//! # Wire format (IFB1)
//!
//! Byte-for-byte the format `streamhost/streamhost/src/capture/shm.rs`
//! consumes, which MAME's Newport device already produces for the sibling
//! `irix` station. Little-endian, one producer, many readers, readers never
//! write:
//!
//! ```text
//!   off  type  field
//!     0  u32   magic 'IFB1' (0x3142_4649)
//!     4  u32   version (1)
//!     8  u32   width
//!    12  u32   height
//!    16  u32   stride (bytes per row)
//!    20  u32   bpp (32)
//!    24  u64   sequence (seqlock: ODD while writing, EVEN when settled)
//!    32  u32   dirty_x0   36 u32 dirty_y0
//!    40  u32   dirty_x1   44 u32 dirty_y1   (EXCLUSIVE bounds)
//!    48        pad to 64
//!    64        height * stride bytes of pixels
//! ```
//!
//! ## Pixel format — no conversion happens here
//!
//! `SwCompositor`'s final store (`compositor.rs`) is
//! `buf[i] = 0xFF000000 | (r << 16) | (g << 8) | b`, i.e. the word is
//! `0xFFRRGGBB`, which on x86 is **B, G, R, X in memory** — byte-identical to
//! the BGRA the encoder wants and to MAME's `bitmap_rgb32`. At 1280x1024 a
//! per-pixel swap would be 1.3 M shuffles every frame, so it matters that there
//! is none.
//!
//! Two places in the tree say otherwise and are simply wrong: `disp.rs`'s
//! `save_screenshot` comment claims `0xFFBBGGRR` and its encoder pushes the
//! BLUE byte as the PNG's red channel (the RCtrl+PrintScreen path has R and B
//! swapped upstream), and `SwCompositor::pixels`' doc comment repeats the same
//! claim. `ci.rs`'s encoder is the correct one. This was confirmed on a real
//! frame, not taken on faith: IRIX's 4Dwm desktop is teal, and a swapped
//! channel turns it orange.
//!
//! ## Synchronisation
//!
//! A seqlock. `seq` goes odd with release ordering before any pixel is touched
//! and even with release ordering after the last one, so a reader that sees the
//! same even `seq` either side of its copy knows the copy is not torn. The
//! consumer bounds its retries (`MAX_TEARS`) and simply waits for the next
//! frame if it loses too often.
//!
//! ## Damage
//!
//! The consumer supports two modes. `SH_SHM_DAMAGE=1` (its default) ignores the
//! producer's rectangle and diffs the copy host-side to derive one.
//! `SH_SHM_DAMAGE=0` trusts the producer's rectangle as-is.
//!
//! This publisher produces a **real** scanline band, so the station runs
//! `SH_SHM_DAMAGE=0` — the `nextstep` precedent. It comes almost free: the
//! mapping is persistent, so a row whose pixels did not change is already
//! correct in the file and does not need copying at all. Deriving the band and
//! skipping the copy are the same pass. An idle IRIX desktop therefore costs a
//! row-compare over the frame and almost no writes.
//!
//! **An empty rectangle means "nothing changed" to the consumer, and it skips
//! the frame without copying** (`shm.rs`: `if dx1 <= dx0 || dy1 <= dy0`). So we
//! do not publish at all when no row moved — we do not even bump `seq`. The
//! first frame after a (re)map is exempt and always goes out whole, because a
//! daemon attaching to an already-idle guest (the IRIX login chooser is
//! perfectly static) would otherwise never see a frame and never come up.
//!
//! # FBSYNC — and the stale-page trap it exists for
//!
//! The `nextstep` conversion lost a day to this and it is guaranteed to bite
//! any publisher that keeps a private shadow of what the reader is showing:
//! after a savestate restore the emulator's framebuffer changes underneath us
//! while our shadow still believes the reader already has those pixels, so we
//! publish nothing and **the reader streams the pre-restore picture forever,
//! under a live guest**. Nothing in the logs is wrong; only the framebuffer is.
//!
//! [`request_fbsync`] is the cure and stream D's reset path calls it
//! unconditionally after every restore: it drops the shadow (forcing a whole
//! frame) and pokes REX3's `screenshot_pending`, which is the one flag that
//! makes the refresh loop composite even when it believes nothing is dirty.
//!
//! # Failure policy
//!
//! Unset knob = disabled. **Set knob that cannot be honoured = loud failure**,
//! never a silent fallback to no frames — the `MAME_SHM_PATH` rule. A station
//! whose frame plane quietly did not start looks identical to a wedged guest.

use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use crate::compositor::SwCompositor;
use crate::debug_overlay::DebugOverlay;
use crate::disp::{BarStats, Rex3Screen, StatusBar, StatusBarTexture};
use crate::rex3::{Renderer, Rex3};

/// Environment knob naming the file to publish into. Unset = publisher off.
pub const ENV_PATH: &str = "IRIS_SHM_PATH";
/// Optional A/B control: `IRIS_SHM_DAMAGE=0` marks every published frame
/// full-frame instead of deriving a scanline band. Diagnostic only.
pub const ENV_DAMAGE: &str = "IRIS_SHM_DAMAGE";

/// Fixed header size; pixels start here.
const HEADER: usize = 64;
/// 'IFB1' little-endian.
const MAGIC: u32 = 0x3142_4649;
const VERSION: u32 = 1;
const BPP: u32 = 32;
/// The compositor's buffer is always 2048 words per row regardless of the
/// decoded visible width.
const COMPOSITOR_STRIDE: usize = 2048;

/// Set by [`request_fbsync`]; consumed by the next `present()`.
static FBSYNC: AtomicBool = AtomicBool::new(false);
/// Frames actually published, for cadence measurement from outside.
static PUBLISHED: AtomicU64 = AtomicU64::new(0);
/// The REX3 the publisher is attached to, so `request_fbsync` can force a
/// composite pass. `OnceLock` because there is exactly one.
static REX3: std::sync::OnceLock<Arc<Rex3>> = std::sync::OnceLock::new();

/// Force one whole frame to be republished, unconditionally.
///
/// Call this after **every** savestate restore (see the module docs' stale-page
/// trap). Cheap and idempotent: it clears the publisher's shadow so the next
/// composite writes every row, and sets REX3's `screenshot_pending` so a
/// composite happens even if the refresh loop thinks nothing is dirty.
pub fn request_fbsync() {
    FBSYNC.store(true, Ordering::Release);
    if let Some(rex3) = REX3.get() {
        rex3.screenshot_pending.store(true, Ordering::Relaxed);
    }
}

/// Total frames published since start. A monotonically increasing counter that
/// stalls when the guest display is idle — that is correct, not a fault.
pub fn published_frames() -> u64 {
    PUBLISHED.load(Ordering::Relaxed)
}

/// Install the publisher into `rex3.renderer` if `IRIS_SHM_PATH` is set.
///
/// Returns `Ok(false)` when the knob is unset (publisher disabled, upstream
/// behaviour preserved), `Ok(true)` when installed, and `Err` when the knob is
/// set but the mapping could not be created — which the caller must treat as
/// fatal.
pub fn install(rex3: &Arc<Rex3>) -> std::io::Result<bool> {
    let path = match std::env::var(ENV_PATH) {
        Ok(p) if !p.is_empty() => p,
        _ => return Ok(false),
    };
    let derive_damage = std::env::var(ENV_DAMAGE).map(|v| v != "0").unwrap_or(true);

    // Fail here, at install time, rather than on the first frame: a station that
    // cannot publish must refuse to start, not stream a black screen.
    let pubr = ShmPublisher::new(path.clone(), derive_damage)?;
    let _ = REX3.set(rex3.clone());
    *rex3.renderer.lock() = Some(Box::new(pubr));
    eprintln!(
        "iris: shm frame plane -> {} (damage={})",
        path,
        if derive_damage { "derived" } else { "full-frame" }
    );
    Ok(true)
}

/// A live read-write mapping of the published file, sized from the geometry it
/// currently carries.
struct Mapping {
    ptr: *mut u8,
    len: usize,
    width: usize,
    height: usize,
    stride: usize,
}

// The pointer is a private MAP_SHARED mapping owned solely by this struct; the
// publisher lives on the single REX3-Refresh thread.
unsafe impl Send for Mapping {}

impl Drop for Mapping {
    fn drop(&mut self) {
        unsafe { libc::munmap(self.ptr as *mut libc::c_void, self.len) };
    }
}

impl Mapping {
    /// Create the file at `path`, size it for `width x height`, map it, and
    /// write the header with `seq` left ODD — the mapping is not yet valid for
    /// a reader and must not be read until the first frame settles it.
    fn create(path: &str, width: usize, height: usize) -> std::io::Result<Self> {
        let stride = width * 4;
        let len = HEADER + stride * height;

        // Write through a temp file and rename, so a consumer that is already
        // polling never maps a half-sized file. `shm.rs` guards against a short
        // mapping anyway, but the guard should never be what saves us.
        let tmp = format!("{path}.tmp{}", std::process::id());
        {
            let f = std::fs::File::create(&tmp)?;
            f.set_len(len as u64)?;
        }
        std::fs::rename(&tmp, path)?;

        let f = std::fs::OpenOptions::new().read(true).write(true).open(path)?;
        use std::os::fd::AsRawFd;
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                f.as_raw_fd(),
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error());
        }
        let m = Mapping {
            ptr: ptr as *mut u8,
            len,
            width,
            height,
            stride,
        };
        // Header first, geometry included; `seq` odd so a reader that maps us
        // between now and the first frame treats the pixels as in flight.
        m.put_hdr(0, MAGIC);
        m.put_hdr(1, VERSION);
        m.put_hdr(2, width as u32);
        m.put_hdr(3, height as u32);
        m.put_hdr(4, stride as u32);
        m.put_hdr(5, BPP);
        m.put_dirty(0, 0, 0, 0);
        m.seq_word().store(1, Ordering::Release);
        Ok(m)
    }

    fn put_hdr(&self, idx: usize, v: u32) {
        unsafe { std::ptr::write_volatile((self.ptr as *mut u32).add(idx), v) };
    }

    fn put_dirty(&self, x0: u32, y0: u32, x1: u32, y1: u32) {
        self.put_hdr(8, x0);
        self.put_hdr(9, y0);
        self.put_hdr(10, x1);
        self.put_hdr(11, y1);
    }

    fn seq_word(&self) -> &AtomicU64 {
        unsafe { &*(self.ptr.add(24) as *const AtomicU64) }
    }

    /// One row of pixels in the mapping, as bytes.
    fn row_mut(&self, y: usize) -> &mut [u8] {
        unsafe {
            std::slice::from_raw_parts_mut(self.ptr.add(HEADER + y * self.stride), self.stride)
        }
    }
}

/// The `Renderer` that composites and publishes. One per process, owned by
/// `rex3.renderer`, called on the REX3-Refresh thread.
pub struct ShmPublisher {
    path: String,
    map: Option<Mapping>,
    compositor: SwCompositor,
    /// Derive a real scanline band (default) or mark every frame full-frame.
    derive_damage: bool,
    /// True until the first frame settles the seqlock after a (re)map, or after
    /// an `FBSYNC`: the next publish writes every row and claims the whole
    /// frame as dirty.
    force_full: bool,
    seq: u64,
}

impl ShmPublisher {
    fn new(path: String, derive_damage: bool) -> std::io::Result<Self> {
        // Prove now that the path is writable — an unwritable station directory
        // must fail at start, not silently at the first composite.
        let probe = format!("{path}.probe{}", std::process::id());
        std::fs::File::create(&probe)?.write_all(b"")?;
        std::fs::remove_file(&probe)?;
        Ok(Self {
            path,
            map: None,
            compositor: SwCompositor::new(),
            derive_damage,
            force_full: true,
            seq: 0,
        })
    }

    /// Ensure the mapping matches `width x height`, remapping on any change.
    ///
    /// IRIX reprograms the VC2 partway through boot and the decoded visible
    /// rectangle moves; the consumer re-maps on any width/height/stride change
    /// and expects the file to have been re-sized first.
    fn ensure_map(&mut self, width: usize, height: usize) -> std::io::Result<()> {
        let ok = matches!(&self.map, Some(m) if m.width == width && m.height == height);
        if ok {
            return Ok(());
        }
        // Drop the old mapping before creating the new file so the rename never
        // races our own live mapping.
        let old = self.map.take().map(|m| (m.width, m.height));
        let m = Mapping::create(&self.path, width, height)?;
        match old {
            Some((ow, oh)) => eprintln!(
                "iris: shm geometry {ow}x{oh} -> {width}x{height} (stride {})",
                m.stride
            ),
            None => eprintln!("iris: shm first geometry {width}x{height} (stride {})", m.stride),
        }
        self.map = Some(m);
        self.force_full = true;
        Ok(())
    }
}

impl Renderer for ShmPublisher {
    fn present(
        &mut self,
        screen: &mut Rex3Screen,
        _overlay: &mut DebugOverlay,
        _status: &mut StatusBar,
        _sbtex: &mut StatusBarTexture,
        _stats: &BarStats,
        need_readback: bool,
        live_fb_rgb: Option<&[u32]>,
        live_fb_aux: Option<&[u32]>,
    ) {
        let width = screen.width;
        let height = screen.height;
        if width == 0 || height == 0 || width > COMPOSITOR_STRIDE || height > 1024 {
            return;
        }

        let fbsync = FBSYNC.swap(false, Ordering::AcqRel);
        if fbsync {
            self.force_full = true;
        }

        // Heartbeat frame with nothing but the emulator's own status bar to
        // redraw. The HUD is a separate GL texture and never reaches the
        // published surface, so there is genuinely nothing to publish — unless
        // an FBSYNC asked for a frame regardless.
        if screen.status_bar_only && !self.force_full {
            return;
        }

        if let Err(e) = self.ensure_map(width, height) {
            eprintln!("iris: shm publish {}: {e}", self.path);
            return;
        }
        let map = match &self.map {
            Some(m) => m,
            None => return,
        };

        // `fb_borrowed` means refresh() did not copy the framebuffer into the
        // screen caches and handed us live VRAM borrows instead. Passing them
        // through is not optional: ignoring them composites a stale snapshot.
        let src = screen.compositor_source_from(live_fb_rgb, live_fb_aux);
        self.compositor.compose_pixels(&src);
        drop(src);
        let pixels = self.compositor.pixels();

        // Copy only rows that actually changed, and let the same pass tell us
        // the dirty band. The mapping is persistent: an unchanged row is already
        // correct in the file. `dirty_y1` is exclusive.
        let full = self.force_full || !self.derive_damage;
        let mut first: Option<usize> = None;
        let mut last: usize = 0;

        map.seq_word().store(self.seq | 1, Ordering::Release);
        for y in 0..height {
            let src_row = &pixels[y * COMPOSITOR_STRIDE..y * COMPOSITOR_STRIDE + width];
            // SAFETY: `row_mut` is inside our own private mapping; the borrow
            // does not outlive this iteration and no other thread writes it.
            let dst_row = map.row_mut(y);
            let src_bytes = unsafe {
                std::slice::from_raw_parts(src_row.as_ptr() as *const u8, width * 4)
            };
            if !full && dst_row == src_bytes {
                continue;
            }
            dst_row.copy_from_slice(src_bytes);
            if first.is_none() {
                first = Some(y);
            }
            last = y;
        }

        match first {
            Some(y0) => {
                map.put_dirty(0, y0 as u32, width as u32, (last + 1) as u32);
                self.seq = self.seq.wrapping_add(2);
                map.seq_word().store(self.seq, Ordering::Release);
                PUBLISHED.fetch_add(1, Ordering::Relaxed);
            }
            None => {
                // Nothing moved. Settle the seqlock back on the value the reader
                // already has so it does no work at all, and leave the rectangle
                // empty. Never publish an empty rect as a new sequence: the
                // consumer would burn a wakeup to discover there was nothing.
                map.put_dirty(0, 0, 0, 0);
                map.seq_word().store(self.seq, Ordering::Release);
            }
        }
        self.force_full = false;

        // Screenshot readback. Upstream's `--ci` never populated `screen.rgba`
        // because no renderer existed, so `iris-ci screenshot` wrote a black
        // PNG; filling it here makes that verb work host-native for free.
        if need_readback {
            let n = COMPOSITOR_STRIDE * height;
            if screen.rgba.len() >= n {
                screen.rgba[..n].copy_from_slice(&pixels[..n]);
            }
        }
    }

    fn resize(&mut self, _width: usize, _height: usize) {
        // Geometry is taken from `screen` on every present, so there is nothing
        // to do here; but the next frame must be whole.
        self.force_full = true;
    }

    fn compositor_status(&self) -> String {
        format!("compositor=sw shm={} seq={}", self.path, self.seq)
    }
}
