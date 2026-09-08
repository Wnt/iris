# Windows: CLI vanishes with exit 0xC000041D and no message

**Keywords:** windows,winit,0.30,0.30.13,crash,STATUS_FATAL_USER_CALLBACK_EXCEPTION,0xC000041D,-1073740771,panic=abort,wndproc,window procedure,vectored exception handler,VEH,access violation,request_inner_size,SetWindowPos,WM_SIZE,re-entrant event handler,event loop,aspect ratio,issue #94,iris-crash.log,crash_diag
**Category:** gui

## Symptom

On some Windows machines the native (winit) CLI dies during startup — right
after `Rex3: Resolution changed to 1282x1024` — with exit code `0xC000041D`
(`-1073740771`, `STATUS_FATAL_USER_CALLBACK_EXCEPTION`) and **no** Rust panic
message, no backtrace, no window. `iris-gui` (eframe) is unaffected.
issue #94: ~90% of launches on one reporter's internal laptop panel, 0% on an
external monitor; unaffected by DPI scale.

## What that exit code means

`0xC000041D` is what 64-bit Windows raises when an exception escapes a
**user-mode callback invoked by the kernel** — a window procedure, hook proc,
`Enum*` callback. The kernel's callback dispatcher catches the original
exception, tears down the callback frame, and re-raises this fixed status. The
original cause (a Rust panic, or a native access violation in a driver) is gone
by the time any top-level `SetUnhandledExceptionFilter` runs.

Because the release profile is `panic = "abort"`, a Rust panic on a wndproc
stack aborts immediately — the default hook prints the message but it is easily
lost, and there is no unwind and no backtrace unless `RUST_BACKTRACE` is set.

## Getting a diagnosis

`src/crash_diag.rs` (installed first thing in `main`) exists for exactly this:

* **Panic hook** — fires even under `panic = "abort"`, before the abort. Writes
  message + thread + location + backtrace to `iris-crash.log`.
* **Vectored exception handler** (Windows) — runs *first-chance*, before the
  callback dispatcher swallows anything. Logs the real exception code, faulting
  address and a `module+offset` stack for access violations / illegal
  instructions / stack overflow / heap corruption / the fatal-callback status
  itself.

Ask a reporter to reproduce once and attach `iris-crash.log`. Symbolize the
`iris.exe+0xNNNN` frames against the matching build with
`addr2line -e iris.exe -f -C 0xNNNN` (or the `.pdb`).

Self-test the wiring: `IRIS_CRASH_SELFTEST=panic|thread|segv target/release/iris.exe`.
Escape hatch if the VEH ever gets noisy: `IRIS_CRASH_DIAG=off` (panic hook stays).

## What is NOT the cause (verified — don't re-investigate)

The obvious theory was re-entrancy: `WindowEvent::Resized`'s aspect-ratio lock
calls `Window::request_inner_size` from *inside* the winit event callback; on
Windows that reaches `SetWindowPos` synchronously, which re-enters the wndproc
with `WM_SIZE`.

* The **REX3-refresh-thread** `request_inner_size` in `GlRenderer::resize` is
  *not* it — cross-thread `SetWindowPos` uses `SWP_ASYNCWINDOWPOS`, so the
  resize is posted, never re-entrant. (Commit c2e085a's rationale is imprecise
  on this point.)
* The **event-thread** re-entrant `request_inner_size` is buffered, not
  crashed: vendored winit 0.30.13's `EventLoopRunner::should_buffer()` detects
  the taken handler and defers the nested `WindowEvent`. Forcing an *infinite*
  re-entrant `request_inner_size` from the `Resized` handler just ping-pongs
  the window ±1px forever without panicking.
* Could not reproduce on a desktop (single 2560×1440 @ 96 DPI) across ~130
  launches: delayed cross-thread resize, spammed resize, forced work-area
  clamp, forced infinite re-entrancy — all 0 crashes.
* The fix commits (c2e085a + 81c34ba) do **not** stop the reporter's crash.

## Upstream winit status

Not fixed. 0.30.13 is the last 0.30.x. `v0.31.0-beta.3` keeps the identical
`call_event_handler` re-entrancy assert (`"either event handler is re-entrant
(likely)…"`) and the identical `send_event` path that dispatches
`RedrawRequested` **directly, bypassing `should_buffer`** — the one genuine
re-entrancy hole. No changelog entry addresses Windows re-entrancy from
`request_inner_size`/`WM_SIZE`. Upstream's pattern for this class is deferral:
the `pending_drag` / `source_drag` fields were added so the blocking
`DoDragDrop` runs only after the app returns control to winit. Any iris fix
should follow suit — never call a synchronous window-mutating method from
inside a winit callback; queue it and apply it from outside dispatch.

## Working hypothesis

Given the panel dependency and that `panic = "unwind"` + `RUST_BACKTRACE=full`
produced *no* Rust output, the fault is most likely **not a Rust panic** but a
native access violation inside the wndproc — the GL ICD mishandling the HWND/DC
while `GlRenderer::ensure_init` creates the window surface on the REX3 thread at
the same moment the event thread services the mode-change resize. The VEH in
`crash_diag.rs` is what will confirm or refute this from a reporter's machine.
