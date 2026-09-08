//! Crash diagnostics — turn "iris just vanished with no message" into a log file.
//!
//! Two independent failure classes have bitten the CLI on Windows:
//!
//!  * **Rust panics.** The release profile is `panic = "abort"`, so a panic
//!    prints its message (the default hook still runs) and then `abort()`s with
//!    no unwinding and no backtrace unless `RUST_BACKTRACE` is set. If the
//!    panic happens on the stack of a Win32 callback (winit's window
//!    procedure), even the message can be lost in the noise and the exit code
//!    is `0xC000041D` (`STATUS_FATAL_USER_CALLBACK_EXCEPTION`).
//!
//!  * **Non-Rust faults inside a callback** — e.g. an access violation in a GL
//!    driver reached from `WM_SIZE`/`WM_PAINT`. 64-bit Windows does not run
//!    `SetUnhandledExceptionFilter` for these (the kernel's user-callback
//!    dispatcher swallows the original exception and re-raises it as
//!    `0xC000041D`), so a plain top-level filter never sees the real cause.
//!    This is the shape of issue #94.
//!
//! [`install`] wires up handlers for both. Everything is appended to
//! `iris-crash.log` in the working directory and echoed to stderr. All of it is
//! dormant until something actually crashes — no hot-path cost (see the note on
//! [`install`]).
//!
//! Limitation: the exception handler formats into a fixed stack buffer but
//! still writes the file through `std::fs`, so a *heap-corruption* or
//! *stack-overflow* fault may not manage to produce a report. Access
//! violations — the expected shape of #94 — are fine.

use std::sync::atomic::{AtomicBool, Ordering};

/// Install the panic hook and (on Windows) the exception handlers.
///
/// Call once, as early in `main` as possible.
///
/// ## Performance
///
/// Zero steady-state cost. The panic hook only runs on `panic!`; the Windows
/// vectored exception handler is only entered when the OS raises an exception,
/// which does not happen on any normal code path (it is *not* invoked for
/// `Result::Err`, `None`, integer overflow in release, etc.). No polling
/// thread, no allocation, no syscalls after `install` returns.
pub fn install() {
    install_panic_hook();
    #[cfg(windows)]
    {
        // Escape hatch: `IRIS_CRASH_DIAG=off` skips the Windows exception
        // handlers (the panic hook always stays). Only needed if some
        // dependency ever turns out to use first-chance access violations as
        // control flow and spams the log.
        if std::env::var("IRIS_CRASH_DIAG").as_deref() != Ok("off") {
            windows::install_exception_handlers();
        }
    }
}

/// Path we append crash reports to.
const LOG_PATH: &str = "iris-crash.log";

/// Deliberately crash, to prove the handlers are wired up. Driven by
/// `IRIS_CRASH_SELFTEST` from `main`:
///   * `panic`  — a normal `panic!` (exercises the panic hook)
///   * `segv`   — a null dereference (exercises the vectored handler)
///   * `thread` — a panic on a spawned, named thread
///
/// After running it, check that `iris-crash.log` gained an entry.
pub fn selftest(kind: &str) {
    match kind {
        "panic" => panic!("crash_diag self-test: deliberate panic on the main thread"),
        "thread" => {
            let h = std::thread::Builder::new()
                .name("selftest-victim".into())
                .spawn(|| panic!("crash_diag self-test: deliberate panic on a worker thread"))
                .unwrap();
            let _ = h.join();
            // If we get here the build is `panic = "unwind"` (the worker's
            // panic only killed the worker). A `panic = "abort"` release build
            // would have taken the whole process down at the panic above —
            // which is the point of this variant. Don't fall through into a
            // full emulator boot.
            eprintln!(
                "crash_diag: worker thread panicked and the process survived \
                 (panic=unwind build); a release build would have aborted here"
            );
            std::process::exit(70);
        }
        "segv" => unsafe {
            // Write through a null pointer -> STATUS_ACCESS_VIOLATION.
            let p: *mut u8 = std::ptr::null_mut();
            std::ptr::write_volatile(p, 1);
        },
        other => eprintln!("crash_diag: unknown IRIS_CRASH_SELFTEST={other:?} (want panic|thread|segv)"),
    }
}

fn install_panic_hook() {
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        // Guard against a panic *inside* the hook (e.g. from the backtrace
        // machinery) turning into an infinite recursion.
        static IN_HOOK: AtomicBool = AtomicBool::new(false);
        if IN_HOOK.swap(true, Ordering::SeqCst) {
            default_hook(info);
            return;
        }

        let thread = std::thread::current();
        let bt = std::backtrace::Backtrace::force_capture();
        let report = format!(
            "\n================ iris panic ================\n\
             when   : {}\n\
             thread : {}\n\
             location: {}\n\
             message: {}\n\
             backtrace:\n{bt}\n\
             ===========================================\n",
            timestamp(),
            thread.name().unwrap_or("<unnamed>"),
            info.location().map(|l| l.to_string()).unwrap_or_else(|| "<unknown>".into()),
            payload_str(info),
        );

        append_to_log(report.as_bytes());
        eprint!("{report}");
        use std::io::Write;
        let _ = std::io::stderr().flush();

        IN_HOOK.store(false, Ordering::SeqCst);
        default_hook(info);
    }));
}

fn payload_str<'a>(info: &'a std::panic::PanicHookInfo<'a>) -> &'a str {
    let p = info.payload();
    if let Some(s) = p.downcast_ref::<&str>() {
        s
    } else if let Some(s) = p.downcast_ref::<String>() {
        s.as_str()
    } else {
        "<non-string panic payload>"
    }
}

/// Best-effort "seconds since the Unix epoch" stamp. Avoids pulling a date
/// crate into the binary just for a log header; correlate with the log file's
/// mtime if you need wall-clock.
fn timestamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => format!("unix+{}.{:03}s", d.as_secs(), d.subsec_millis()),
        Err(_) => "unix+?".into(),
    }
}

/// Append bytes to `iris-crash.log`, creating it if needed. Silent on failure —
/// we are on a crash path and there is nothing useful to do if this fails.
fn append_to_log(bytes: &[u8]) {
    use std::io::Write;
    if let Ok(mut f) =
        std::fs::OpenOptions::new().create(true).append(true).open(LOG_PATH)
    {
        let _ = f.write_all(bytes);
        let _ = f.flush();
    }
}

#[cfg(windows)]
mod windows {
    use super::{append_to_log, LOG_PATH};
    use std::sync::atomic::{AtomicBool, Ordering};

    use core::fmt::Write as _;

    use windows_sys::Win32::Foundation::HMODULE;
    use windows_sys::Win32::System::Diagnostics::Debug::{
        AddVectoredExceptionHandler, RtlCaptureStackBackTrace, SetUnhandledExceptionFilter,
        EXCEPTION_POINTERS,
    };
    use windows_sys::Win32::System::LibraryLoader::{
        GetModuleFileNameW, GetModuleHandleExW, GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,
        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
    };
    use windows_sys::Win32::System::SystemInformation::GetTickCount64;

    // From <winnt.h>. windows-sys spreads these across modules / doesn't re-export
    // them all as consts, so spell them out.
    const EXCEPTION_CONTINUE_SEARCH: i32 = 0;
    const STATUS_ACCESS_VIOLATION: u32 = 0xC000_0005;
    const STATUS_IN_PAGE_ERROR: u32 = 0xC000_0006;
    const STATUS_ILLEGAL_INSTRUCTION: u32 = 0xC000_001D;
    const STATUS_PRIVILEGED_INSTRUCTION: u32 = 0xC000_0096;
    const STATUS_STACK_OVERFLOW: u32 = 0xC000_00FD;
    const STATUS_FATAL_USER_CALLBACK_EXCEPTION: u32 = 0xC000_041D;
    const STATUS_HEAP_CORRUPTION: u32 = 0xC000_0374;

    /// Codes worth a report. Deliberately excludes the subset of first-chance
    /// exceptions that normal code handles on purpose (`0xE06D7363` C++
    /// exceptions, `0x406D1388` the thread-name marker, DBG_* control codes,
    /// etc.) so a diagnostic build doesn't cry wolf.
    fn is_interesting(code: u32) -> bool {
        matches!(
            code,
            STATUS_ACCESS_VIOLATION
                | STATUS_IN_PAGE_ERROR
                | STATUS_ILLEGAL_INSTRUCTION
                | STATUS_PRIVILEGED_INSTRUCTION
                | STATUS_STACK_OVERFLOW
                | STATUS_FATAL_USER_CALLBACK_EXCEPTION
                | STATUS_HEAP_CORRUPTION
        )
    }

    fn code_name(code: u32) -> &'static str {
        match code {
            STATUS_ACCESS_VIOLATION => "ACCESS_VIOLATION",
            STATUS_IN_PAGE_ERROR => "IN_PAGE_ERROR",
            STATUS_ILLEGAL_INSTRUCTION => "ILLEGAL_INSTRUCTION",
            STATUS_PRIVILEGED_INSTRUCTION => "PRIVILEGED_INSTRUCTION",
            STATUS_STACK_OVERFLOW => "STACK_OVERFLOW",
            STATUS_FATAL_USER_CALLBACK_EXCEPTION => "FATAL_USER_CALLBACK_EXCEPTION",
            STATUS_HEAP_CORRUPTION => "HEAP_CORRUPTION",
            _ => "?",
        }
    }

    pub fn install_exception_handlers() {
        unsafe {
            // First arg != 0 => call us *first*, ahead of any other vectored
            // handler and, crucially, before the frame-based dispatch that on
            // x64 would otherwise swallow a fault that escaped a kernel
            // callback and re-raise it as 0xC000041D with the original cause
            // gone.
            let _ = AddVectoredExceptionHandler(1, Some(vectored_handler));
            let _ = SetUnhandledExceptionFilter(Some(unhandled_filter));
        }
    }

    /// Vectored handler: sees every exception first-chance. We report the
    /// interesting ones exactly once, then always return CONTINUE_SEARCH so the
    /// normal handling path is unchanged (if something legitimately handles the
    /// exception, we merely logged a spurious line; if nothing does, the
    /// process dies as before — but now with a report).
    unsafe extern "system" fn vectored_handler(info: *mut EXCEPTION_POINTERS) -> i32 {
        if info.is_null() {
            return EXCEPTION_CONTINUE_SEARCH;
        }
        let rec = (*info).ExceptionRecord;
        if rec.is_null() {
            return EXCEPTION_CONTINUE_SEARCH;
        }
        let code = (*rec).ExceptionCode as u32;
        if !is_interesting(code) {
            return EXCEPTION_CONTINUE_SEARCH;
        }

        // Only the first interesting fault gets a full dump. A second one is
        // very likely our own stack-walk touching bad memory.
        static DUMPED: AtomicBool = AtomicBool::new(false);
        if DUMPED.swap(true, Ordering::SeqCst) {
            return EXCEPTION_CONTINUE_SEARCH;
        }

        let fault_addr = (*rec).ExceptionAddress as usize;
        let (rw, data_addr) = if (*rec).NumberParameters >= 2
            && matches!(code, STATUS_ACCESS_VIOLATION | STATUS_IN_PAGE_ERROR)
        {
            let kind = match (*rec).ExceptionInformation[0] {
                0 => "read",
                1 => "write",
                8 => "execute (DEP)",
                _ => "?",
            };
            (kind, (*rec).ExceptionInformation[1])
        } else {
            ("", 0)
        };

        // No heap from here down — a heap-corruption or stack-overflow fault
        // can't afford an allocator call. Format into a fixed stack buffer.
        let mut buf = FixedBuf::<8192>::new();
        let _ = core::fmt::write(
            &mut buf,
            format_args!(
                "\n============ iris hardware exception ============\n\
                 os_tick_ms : {}  (GetTickCount64; correlate with the log mtime)\n\
                 code       : 0x{:08X}  {}\n\
                 fault addr : 0x{:016X}\n",
                GetTickCount64(),
                code,
                code_name(code),
                fault_addr,
            ),
        );
        if !rw.is_empty() {
            let _ = core::fmt::write(
                &mut buf,
                format_args!("access     : {rw} @ 0x{data_addr:016X}\n"),
            );
        }
        if code == STATUS_FATAL_USER_CALLBACK_EXCEPTION {
            let _ = core::fmt::write(
                &mut buf,
                format_args!(
                    "note       : this is the *re-raised* status; the original fault \
                     (in a wndproc / driver callback) should appear above as an earlier \
                     first-chance exception if it was one we recognise.\n"
                ),
            );
        }
        // Flush the header now, *before* the stack walk — if walking a corrupt
        // stack faults, the VEH re-enters, hits the DUMPED guard and bails, and
        // we'd otherwise have written nothing at all. The code + fault address
        // alone are already useful.
        emit(buf.as_bytes());

        let mut tail = FixedBuf::<8192>::new();
        let _ = tail.write_str("stack (return addresses, resolve with the matching .pdb / addr2line):\n");
        write_backtrace(&mut tail);
        let _ = tail.write_str("================================================\n");
        emit(tail.as_bytes());

        // Breadcrumb in case stderr is a pipe nobody is tailing.
        let _ = std::io::Write::write_all(
            &mut std::io::stderr(),
            format!("iris: wrote crash report to {LOG_PATH}\n").as_bytes(),
        );
        EXCEPTION_CONTINUE_SEARCH
    }

    /// Last-ditch top-level filter. Reached for a fault that was *not* inside a
    /// kernel callback (those never get here on x64). Complements the vectored
    /// handler rather than replacing it.
    unsafe extern "system" fn unhandled_filter(info: *const EXCEPTION_POINTERS) -> i32 {
        // Reuse the vectored path's formatting; it self-guards against a second
        // dump, so if the VEH already reported this fault we stay quiet.
        vectored_handler(info as *mut EXCEPTION_POINTERS);
        EXCEPTION_CONTINUE_SEARCH
    }

    /// Walk up to 62 frames and append `module.dll+0xoffset  (0xabsolute)` lines.
    unsafe fn write_backtrace(buf: &mut dyn core::fmt::Write) {
        let mut frames: [*mut core::ffi::c_void; 62] = [core::ptr::null_mut(); 62];
        let n = RtlCaptureStackBackTrace(0, frames.len() as u32, frames.as_mut_ptr(), core::ptr::null_mut());
        for &frame in frames.iter().take(n as usize) {
            let addr = frame as usize;
            let mut module: HMODULE = core::ptr::null_mut();
            let ok = GetModuleHandleExW(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
                    | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                frame as *const u16,
                &mut module,
            );
            if ok != 0 && !module.is_null() {
                let base = module as usize;
                let mut wide = [0u16; 260];
                let len = GetModuleFileNameW(module, wide.as_mut_ptr(), wide.len() as u32) as usize;
                let name = basename_utf8(&wide[..len.min(wide.len())]);
                let _ = core::fmt::write(
                    buf,
                    format_args!("  {}+0x{:X}  (0x{:016X})\n", name.as_str(), addr - base, addr),
                );
            } else {
                let _ = core::fmt::write(buf, format_args!("  ???+0x0  (0x{addr:016X})\n"));
            }
        }
        if n == 0 {
            let _ = buf.write_str("  <no frames captured>\n");
        }
    }

    /// Last path component of a UTF-16 path, lossily narrowed into a small
    /// fixed buffer (no heap).
    fn basename_utf8(wide: &[u16]) -> ArrString<128> {
        let start = wide
            .iter()
            .rposition(|&c| c == b'\\' as u16 || c == b'/' as u16)
            .map(|i| i + 1)
            .unwrap_or(0);
        let mut s = ArrString::<128>::new();
        for ch in char::decode_utf16(wide[start..].iter().copied()) {
            let c = ch.unwrap_or('\u{FFFD}');
            if c == '\0' {
                break;
            }
            s.push(c);
        }
        s
    }

    /// Write to both the log file and stderr. File first: on a hard crash the
    /// stderr handle may already be torn down.
    fn emit(bytes: &[u8]) {
        append_to_log(bytes);
        use std::io::Write;
        let _ = std::io::stderr().write_all(bytes);
        let _ = std::io::stderr().flush();
    }

    // ---- tiny no-alloc formatting helpers ---------------------------------

    struct FixedBuf<const N: usize> {
        data: [u8; N],
        len: usize,
    }
    impl<const N: usize> FixedBuf<N> {
        fn new() -> Self {
            Self { data: [0; N], len: 0 }
        }
        fn as_bytes(&self) -> &[u8] {
            &self.data[..self.len]
        }
    }
    impl<const N: usize> core::fmt::Write for FixedBuf<N> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let b = s.as_bytes();
            let take = b.len().min(N - self.len);
            self.data[self.len..self.len + take].copy_from_slice(&b[..take]);
            self.len += take;
            if take < b.len() {
                Err(core::fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    struct ArrString<const N: usize> {
        data: [u8; N],
        len: usize,
    }
    impl<const N: usize> ArrString<N> {
        fn new() -> Self {
            Self { data: [0; N], len: 0 }
        }
        fn push(&mut self, c: char) {
            let mut tmp = [0u8; 4];
            let s = c.encode_utf8(&mut tmp);
            let b = s.as_bytes();
            if self.len + b.len() <= N {
                self.data[self.len..self.len + b.len()].copy_from_slice(b);
                self.len += b.len();
            }
        }
        fn as_str(&self) -> &str {
            core::str::from_utf8(&self.data[..self.len]).unwrap_or("<bad-utf8>")
        }
    }
}
