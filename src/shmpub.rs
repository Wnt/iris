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
//! ## Pixel format — one channel swap, and it was measured, not reasoned
//!
//! `SwCompositor`'s final store (`compositor.rs`) is
//! `buf[i] = 0xFF000000 | (r << 16) | (g << 8) | b`. Read naively that says
//! the word is `0xFFRRGGBB`, i.e. B,G,R,X in memory, which is exactly what the
//! consumer wants — so this publisher was first written with no conversion at
//! all.
//!
//! **That is wrong, and the framebuffer said so.** The variables in that
//! expression are mis-named: what the compositor calls `r` carries blue all
//! the way through, so the word is `0xFFBBGGRR` and memory order is
//! **R, G, B, X**. Measured on the first published frame of an IRIX 6.5 boot:
//! the SGI background gradient came out orange instead of its blue, and a raw
//! sample at (20,20) read `AE CF FC FF` — (174, 207, 252) as R,G,B, a pale
//! blue, and nonsense the other way round.
//!
//! The layout is not a bug: the windowed GL path uploads that buffer as
//! `glow::RGBA`, and `iris-gui`'s `Frame` documents "Pixel order: R, G, B, A"
//! for the same bytes. Both agree with R,G,B,X. So the compositor stays as it
//! is and the swap happens here, on the way into the mapping — the one place
//! that actually wants B,G,R,X.
//!
//! It costs ~4 integer ops per pixel on rows that changed, and rows that did
//! not change are not touched at all (see Damage below), so an idle desktop
//! pays nothing. The upstream defect this DOES confirm is real:
//! `disp.rs::save_screenshot` (the RCtrl+PrintScreen path) pushes `px & 0xFF`
//! as the PNG's red channel, which for a `0xFFBBGGRR` word is the red byte and
//! therefore correct — while its own comment claims the opposite. `ci.rs`'s
//! encoder agrees with `save_screenshot`. Both are right; only the comments
//! lie.
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
//! # Geometry, and the two columns
//!
//! The visible rectangle is not a constant: it is decoded from the VC2 video
//! timings, and the framebuffer behind it is 2048x1024 words regardless. On
//! this IRIX 6.5 Indy the decode settles at **1282x1024** (`Rex3: Resolution
//! changed to 1282x1024 cursor_x_adjust=5`), which is what this publisher
//! declares by default.
//!
//! The station's registry declares 1280x1024, and a station's geometry is not
//! something to renegotiate for two columns — so `IRIS_SHM_GEOMETRY=1280x1024`
//! makes the publisher crop, once, on the producing side. That is deliberate:
//! the consumer has no crop knob and must never be asked to guess.
//!
//! Worth being accurate about what is discarded. Those two columns are **not
//! black padding** — measured at the IRIX login, columns 1278 through 1281 all
//! carry the identical desktop background word (0xFF7D9EC0), i.e. real decoded
//! picture, and the same is true of the rows behind them. They are overscan at
//! the right edge of a 1282-wide decode, so cropping them loses two columns of
//! border and nothing a visitor can name. It is still a crop, not a trim of
//! nothing, and the guest doc should say so.
//!
//! # The hardware cursor, and why it must be re-latched here
//!
//! `SwCompositor` places the cursor from `VC2_REG_WORKING_CURSOR_Y` (0x0D).
//! That register is raster state: the REX3 refresh loop re-latches it from
//! `VC2_REG_CURSOR_Y_LOC` (0x03) at VBLANK — but `Rex3Screen::refresh()`
//! snapshots the VC2 registers *before* that re-latch happens. So the snapshot
//! the compositor sees carries the PREVIOUS frame's working Y, and the drawn
//! glyph trails the registers by up to a frame: measured at 10-14 px low at
//! some positions while the registers themselves were exact.
//!
//! Invisible in a window (the eye does not mind a cursor one frame behind), but
//! it is a correctness bug here, because the published frame is the only
//! evidence the lab accepts that a pointer went where it was told, and a
//! closed-loop positioner reads the glyph back. So the publisher applies the
//! re-latch itself, on its own snapshot, immediately before compositing. It
//! touches only `Rex3Screen`'s cached copy; the device registers and the
//! windowed path are untouched.
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
/// Optional `WxH`: publish exactly this rectangle, cropped from the top-left of
/// whatever VC2 decodes, instead of the decoded rectangle itself. See
/// "Geometry" in the module docs.
pub const ENV_GEOMETRY: &str = "IRIS_SHM_GEOMETRY";

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
    // `WxH`, or a loud failure. A station that asked for a geometry and silently
    // got a different one is exactly the class of bug this whole plane exists to
    // make impossible.
    let crop = match std::env::var(ENV_GEOMETRY) {
        Ok(v) if !v.is_empty() => {
            let bad = || {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("{ENV_GEOMETRY}={v:?} is not WxH (e.g. 1280x1024)"),
                )
            };
            let (w, h) = v.split_once(['x', 'X']).ok_or_else(bad)?;
            let w: usize = w.trim().parse().map_err(|_| bad())?;
            let h: usize = h.trim().parse().map_err(|_| bad())?;
            if w == 0 || h == 0 || w > COMPOSITOR_STRIDE || h > 1024 {
                return Err(bad());
            }
            Some((w, h))
        }
        _ => None,
    };

    // Fail here, at install time, rather than on the first frame: a station that
    // cannot publish must refuse to start, not stream a black screen.
    let pubr = ShmPublisher::new(path.clone(), derive_damage, crop)?;
    let _ = REX3.set(rex3.clone());
    *rex3.renderer.lock() = Some(Box::new(pubr));
    match crop {
        Some((w, h)) => eprintln!(
            "iris: shm frame plane -> {path} (damage={}, geometry pinned to {w}x{h})",
            if derive_damage { "derived" } else { "full-frame" }
        ),
        None => eprintln!(
            "iris: shm frame plane -> {path} (damage={}, geometry follows the VC2 decode)",
            if derive_damage { "derived" } else { "full-frame" }
        ),
    }
    Ok(true)
}

/// `0xFFBBGGRR` (compositor, R,G,B,X in memory) -> `0xFFRRGGBB` (IFB1, B,G,R,X
/// in memory). Green and the pad byte stay put; red and blue trade places.
/// Written as plain integer arithmetic so LLVM can vectorise the row loop.
#[inline(always)]
fn swap_rb(w: u32) -> u32 {
    (w & 0xFF00_FF00) | ((w & 0x00FF_0000) >> 16) | ((w & 0x0000_00FF) << 16)
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
}

/// The `Renderer` that composites and publishes. One per process, owned by
/// `rex3.renderer`, called on the REX3-Refresh thread.
pub struct ShmPublisher {
    path: String,
    map: Option<Mapping>,
    compositor: SwCompositor,
    /// Derive a real scanline band (default) or mark every frame full-frame.
    derive_damage: bool,
    /// `IRIS_SHM_GEOMETRY`: publish exactly this rectangle instead of the
    /// decoded one, cropping from the top-left.
    crop: Option<(usize, usize)>,
    /// True until the first frame settles the seqlock after a (re)map, or after
    /// an `FBSYNC`: the next publish writes every row and claims the whole
    /// frame as dirty.
    force_full: bool,
    /// Our copy of the COMPOSITOR's last published pixels (tightly packed,
    /// UNSWAPPED). Diffing against this rather than against the mapping keeps
    /// an unchanged row down to one `memcmp`.
    shadow: Vec<u32>,
    seq: u64,
}

impl ShmPublisher {
    fn new(path: String, derive_damage: bool, crop: Option<(usize, usize)>) -> std::io::Result<Self> {
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
            crop,
            force_full: true,
            shadow: Vec::new(),
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
        let decoded_w = screen.width;
        let decoded_h = screen.height;
        if decoded_w == 0 || decoded_h == 0 || decoded_w > COMPOSITOR_STRIDE || decoded_h > 1024 {
            return;
        }
        // Publish the pinned rectangle if there is one, never more than what was
        // actually decoded (a crop must not invent pixels).
        let (width, height) = match self.crop {
            Some((w, h)) => (w.min(decoded_w), h.min(decoded_h)),
            None => (decoded_w, decoded_h),
        };

        // Re-latch the cursor's Y before compositing: `refresh()` snapshotted the
        // VC2 registers before the VBLANK handler copies CURSOR_Y_LOC into
        // WORKING_CURSOR_Y, and `SwCompositor` places the glyph from the latter.
        // Without this the drawn cursor trails the registers by a frame.
        screen.vc2_regs[crate::vc2::VC2_REG_WORKING_CURSOR_Y as usize] =
            screen.vc2_regs[crate::vc2::VC2_REG_CURSOR_Y_LOC as usize];

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
        //
        // The comparison is against our own shadow of the COMPOSITOR's pixels,
        // not against the mapping, so an unchanged row costs one `memcmp` and
        // never pays for the channel swap.
        let full = self.force_full || !self.derive_damage;
        if self.shadow.len() != width * height {
            self.shadow.resize(width * height, 0);
        }
        let mut first: Option<usize> = None;
        let mut last: usize = 0;

        map.seq_word().store(self.seq | 1, Ordering::Release);
        for y in 0..height {
            let src_row = &pixels[y * COMPOSITOR_STRIDE..y * COMPOSITOR_STRIDE + width];
            let sh_row = &mut self.shadow[y * width..(y + 1) * width];
            if !full && sh_row == src_row {
                continue;
            }
            sh_row.copy_from_slice(src_row);
            // SAFETY: the mapping is ours alone, `HEADER` and `stride` are both
            // multiples of 4 so the row is u32-aligned, and the borrow does not
            // outlive this iteration.
            let dst_row = unsafe {
                std::slice::from_raw_parts_mut(
                    map.ptr.add(HEADER + y * map.stride) as *mut u32,
                    width,
                )
            };
            for (d, &sp) in dst_row.iter_mut().zip(sh_row.iter()) {
                *d = swap_rb(sp);
            }
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
