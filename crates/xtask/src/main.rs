//! Dev automation (`cargo xtask`). Full traceability engine lands in M1; the
//! argument parsing shell below grows as needed (kept dependency-free on purpose).

#![forbid(unsafe_code)]

use std::process::ExitCode;

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    match args.next().as_deref() {
        None => {
            eprintln!("usage: cargo xtask <traceability>");
            ExitCode::from(2)
        }
        Some(cmd) => {
            eprintln!("xtask '{cmd}' is implemented from milestone M1 onward (see PROGRESS.md)");
            ExitCode::SUCCESS
        }
    }
}
