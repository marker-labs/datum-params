//! C ABI over lazy evaluation so C++ (and other FFI languages) can load CAD
//! parameter values at startup without pre-generated artifacts.
//!
//! The compiled-artifact CLI and this C ABI call the same Rust `evaluate`
//! function, so those two modes agree by construction whenever the spec and CAD
//! sources are unchanged. The Python loader is a separate implementation and is
//! kept in step by conformance tests, not by construction.
//!
//! Conventions: status functions return 0 on success and -1 on error. String
//! functions return 0 after writing the NUL-terminated text; a positive return
//! value is the required buffer size in bytes including the NUL, meaning the
//! call wrote nothing and should be retried with a larger buffer (pass a null
//! buffer and length 0 to query the size). Errors are formatted with
//! `dparams_lazy_error`. A failed `dparams_lazy_eval` clears any active session,
//! so a caller can never fall back to values from an earlier evaluation. The
//! session and error slots are process-global: call these functions from one
//! thread at a time, and release the session with `dparams_lazy_release`.
use crate::Evaluation;
use std::{
    collections::BTreeMap,
    ffi::{c_char, CStr},
    fs,
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    ptr,
    sync::{Mutex, MutexGuard},
};

struct Session {
    evaluation: Evaluation,
    order: Vec<String>,
    hashes: BTreeMap<String, String>,
}

struct State {
    session: Option<Session>,
    error: Option<String>,
}

static STATE: Mutex<State> = Mutex::new(State { session: None, error: None });

fn lock() -> MutexGuard<'static, State> {
    // A panic while holding the lock poisons it; the state is still usable.
    STATE.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn set_error(message: String) {
    lock().error = Some(message);
}

/// Runs an exported body, converting a panic into a recorded error instead of
/// unwinding across the C boundary (which would abort the process).
fn guarded<T>(fallback: T, body: impl FnOnce() -> T) -> T {
    match catch_unwind(AssertUnwindSafe(body)) {
        Ok(value) => value,
        Err(_) => {
            set_error("internal panic in the Datum lazy evaluation core".into());
            fallback
        }
    }
}

fn cstr(pointer: *const c_char, role: &str) -> Result<String, String> {
    if pointer.is_null() {
        return Err(format!("{role} argument is null"));
    }
    let text = unsafe { CStr::from_ptr(pointer) };
    text.to_str()
        .map(str::to_owned)
        .map_err(|_| format!("{role} argument is not valid UTF-8"))
}

/// Writes `text` NUL-terminated into `buffer`. Returns 0 on success, or the
/// required byte length (including the NUL) when the buffer is null or too
/// small, in which case nothing is written.
fn write_string(buffer: *mut c_char, len: usize, text: &str) -> i32 {
    let bytes = text.as_bytes();
    let needed = bytes.len() as i32 + 1;
    if buffer.is_null() || len < bytes.len() + 1 {
        return needed;
    }
    unsafe {
        ptr::copy_nonoverlapping(bytes.as_ptr() as *const c_char, buffer, bytes.len());
        *buffer.add(bytes.len()) = 0;
    }
    0
}

fn with_session<T>(body: impl FnOnce(&Session) -> Result<T, String>, on_error: impl FnOnce() -> T) -> T {
    let mut state = lock();
    let outcome = match state.session.as_ref() {
        Some(session) => body(session),
        None => Err("no lazy evaluation is active; call dparams_lazy_eval first".into()),
    };
    match outcome {
        Ok(value) => {
            state.error = None;
            value
        }
        Err(message) => {
            state.error = Some(message);
            on_error()
        }
    }
}

/// Evaluates the spec against the saved CAD sources and installs the result as
/// the active lazy session. Returns 0 on success, -1 on error. A failed
/// evaluation clears the active session; the error text is available from
/// `dparams_lazy_error`.
#[no_mangle]
pub extern "C" fn dparams_lazy_eval(project: *const c_char, spec: *const c_char, worktree: *const c_char) -> i32 {
    guarded(-1, || {
        let outcome = (|| -> Result<Evaluation, String> {
            let project = cstr(project, "project")?;
            let spec = cstr(spec, "spec")?;
            let worktree = cstr(worktree, "worktree")?;
            crate::evaluate(Path::new(&project), Path::new(&spec), &worktree)
                .map_err(|error| format!("{error:#}"))
        })();
        match outcome {
            Ok(evaluation) => {
                let mut order: Vec<String> = evaluation.parameters.keys().cloned().collect();
                order.sort();
                let hashes = evaluation.sources_sha256.clone();
                let mut state = lock();
                state.session = Some(Session { evaluation, order, hashes });
                state.error = None;
                0
            }
            Err(message) => {
                let mut state = lock();
                state.session = None;
                state.error = Some(message);
                -1
            }
        }
    })
}

/// Count of parameters in the active session. -1 when no session is active.
#[no_mangle]
pub extern "C" fn dparams_lazy_count() -> i32 {
    guarded(-1, || with_session(|session| Ok(session.order.len() as i32), || -1))
}

/// Parameter name for an ordinal position in the active session.
#[no_mangle]
pub extern "C" fn dparams_lazy_name(index: i32, buffer: *mut c_char, len: usize) -> i32 {
    guarded(-1, || {
        with_session(
            |session| {
                let name = usize::try_from(index)
                    .ok()
                    .and_then(|index| session.order.get(index))
                    .ok_or("lazy session parameter index out of range")?;
                Ok(write_string(buffer, len, name))
            },
            || -1,
        )
    })
}

/// Parameter value by name into `*out`. 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn dparams_lazy_get(name: *const c_char, out: *mut f64) -> i32 {
    guarded(-1, || {
        if out.is_null() {
            set_error("output pointer is null".into());
            return -1;
        }
        with_session(
            |session| {
                let name = cstr(name, "parameter name")?;
                match session.evaluation.parameters.get(&name) {
                    Some(value) => {
                        unsafe { *out = *value };
                        Ok(0)
                    }
                    None => Err(format!("unknown lazy parameter {name}")),
                }
            },
            || -1,
        )
    })
}

/// SHA-256 of a recorded source (CAD or utility source) by its spec-relative name.
#[no_mangle]
pub extern "C" fn dparams_lazy_source_hash(name: *const c_char, buffer: *mut c_char, len: usize) -> i32 {
    guarded(-1, || {
        with_session(
            |session| {
                let name = cstr(name, "source name")?;
                match session.hashes.get(&name) {
                    Some(hash) => Ok(write_string(buffer, len, hash)),
                    None => Err(format!("unknown lazy source {name}")),
                }
            },
            || -1,
        )
    })
}

/// Selected worktree of the active session.
#[no_mangle]
pub extern "C" fn dparams_lazy_worktree(buffer: *mut c_char, len: usize) -> i32 {
    guarded(-1, || {
        with_session(|session| Ok(write_string(buffer, len, &session.evaluation.worktree)), || -1)
    })
}

/// Writes the machine-readable lazy-evidence record (evidence schema, mode,
/// values, bounds, source and spec SHA-256 hashes) to `path`, atomically via a
/// temporary file in the same directory. 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn dparams_lazy_evidence(path: *const c_char) -> i32 {
    guarded(-1, || {
        with_session(
            |session| {
                let path = cstr(path, "evidence path")?;
                let document = session.evaluation.evidence();
                let text = serde_json::to_string_pretty(&document).map_err(|error| error.to_string())?;
                let target = Path::new(&path);
                if let Some(parent) = target.parent() {
                    if !parent.as_os_str().is_empty() {
                        fs::create_dir_all(parent).map_err(|error| error.to_string())?;
                    }
                }
                let name = target
                    .file_name()
                    .map(|name| name.to_string_lossy().into_owned())
                    .unwrap_or_else(|| "evidence".into());
                let temporary = target.with_file_name(format!(".{name}.{}.tmp", std::process::id()));
                fs::write(&temporary, format!("{text}\n")).map_err(|error| error.to_string())?;
                if let Err(error) = fs::rename(&temporary, target) {
                    let _ = fs::remove_file(&temporary);
                    return Err(error.to_string());
                }
                Ok(0)
            },
            || -1,
        )
    })
}

/// Last error message, or an empty string when there is no recorded error.
/// Reading the error does not clear it; a successful session call or
/// `dparams_lazy_release` clears it.
#[no_mangle]
pub extern "C" fn dparams_lazy_error(buffer: *mut c_char, len: usize) -> i32 {
    guarded(-1, || {
        let message = lock().error.clone().unwrap_or_default();
        write_string(buffer, len, &message)
    })
}

/// Releases the active session and the recorded error. Values already copied
/// out stay valid; only session-based calls start failing.
#[no_mangle]
pub extern "C" fn dparams_lazy_release() {
    guarded((), || {
        let mut state = lock();
        state.session = None;
        state.error = None;
    })
}
