//! `scenario-runner` CLI: run TOML closed-loop scenarios headlessly.
//!
//! Usage:
//!
//! ```text
//! scenario-runner [--scenario-dir DIR] [--out DIR] [--scenario NAME]...
//!                 [--no-csv] [--no-mcap] [--list]
//! ```
//!
//! Loads every `*.toml` in the scenario dir (sorted by file name), runs the
//! selected ones in virtual time, writes `report/<name>.csv` + `.mcap`, and a
//! `report/summary.md` table. Exit codes: 0 all checks pass, 1 check failure,
//! 2 usage/IO/parse errors.
//!
//! Hand-rolled argument parsing per ADR-0006 (consistent with `xtask`).

#![forbid(unsafe_code)]

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use scenario_runner::output::{self, McapLog};
use scenario_runner::runner;
use scenario_runner::scenario::Scenario;
use scenario_runner::{CheckOutcome, RunResult};

const USAGE: &str = "usage: scenario-runner [--scenario-dir DIR] [--out DIR] \
                     [--scenario NAME]... [--no-csv] [--no-mcap] [--list]";

struct Args {
    dir: PathBuf,
    out: PathBuf,
    only: Vec<String>,
    no_csv: bool,
    no_mcap: bool,
    list_only: bool,
}

enum ParseOutcome {
    Args(Box<Args>),
    Help(String),
    UsageError(String),
}

const DEFAULT_DIR: &str = "scenarios";
const DEFAULT_OUT: &str = "report";

/// Hand-rolled argument parsing (same style as the other workspace CLIs).
fn parse_args(iter: impl Iterator<Item = String>) -> ParseOutcome {
    let mut args = Args {
        dir: PathBuf::from(DEFAULT_DIR),
        out: PathBuf::from(DEFAULT_OUT),
        only: Vec::new(),
        no_csv: false,
        no_mcap: false,
        list_only: false,
    };
    let mut flag_iter = iter.into_iter().peekable();
    if flag_iter.peek().is_none() {
        return ParseOutcome::Args(Box::new(args));
    }
    while let Some(flag) = flag_iter.next() {
        match flag.as_str() {
            "--help" | "-h" => return ParseOutcome::Help(format!("scenario-runner\n{USAGE}\n")),
            "--scenario-dir" => match flag_iter.next() {
                Some(dir) => args.dir = PathBuf::from(dir),
                None => return ParseOutcome::UsageError("--scenario-dir needs a value".into()),
            },
            "--out" => match flag_iter.next() {
                Some(dir) => args.out = PathBuf::from(dir),
                None => return ParseOutcome::UsageError("--out needs a value".into()),
            },
            "--scenario" => match flag_iter.next() {
                Some(name) => args.only.push(name),
                None => return ParseOutcome::UsageError("--scenario needs a name".into()),
            },
            "--no-csv" => args.no_csv = true,
            "--no-mcap" => args.no_mcap = true,
            "--list" => args.list_only = true,
            other => {
                return ParseOutcome::UsageError(format!("unknown argument '{other}'\n{USAGE}"))
            }
        }
    }
    ParseOutcome::Args(Box::new(args))
}

/// `*.toml` scenario files of `dir`, sorted by file name.
fn collect_toml(dir: &Path) -> Result<Vec<PathBuf>, String> {
    let entries = std::fs::read_dir(dir)
        .map_err(|e| format!("cannot open scenario dir {}: {e}", dir.display()))?;
    let mut files = Vec::new();
    for entry in entries {
        let entry =
            entry.map_err(|e| format!("cannot read scenario dir {}: {e}", dir.display()))?;
        let path = entry.path();
        if path.extension().is_some_and(|ext| ext == "toml") {
            files.push(path);
        }
    }
    files.sort();
    Ok(files)
}

/// Load and parse one scenario file.
fn load(path: &Path) -> Result<Scenario, String> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// Run one scenario; returns the run plus its check results.
fn execute(
    scenario: &Scenario,
    no_csv: bool,
    no_mcap: bool,
    out_dir: &Path,
) -> (RunResult, Vec<CheckOutcome>) {
    let result = runner::run(scenario);
    let checks = runner::evaluate(scenario, &result);
    if !no_csv || !no_mcap {
        let _ = std::fs::create_dir_all(out_dir);
    }
    if !no_csv {
        let path = out_dir.join(format!("{}.csv", scenario.name));
        if let Err(e) = output::write_csv(&path, &result.rows) {
            eprintln!("warning: cannot write {}: {e}", path.display());
        }
    }
    if !no_mcap {
        let path = out_dir.join(format!("{}.mcap", scenario.name));
        match McapLog::create(&path) {
            Ok(mut log) => {
                for row in &result.rows {
                    if let Err(e) = log.write_row(row) {
                        eprintln!("warning: mcap write failed ({}): {e}", path.display());
                        break;
                    }
                }
                if let Err(e) = log.finish() {
                    eprintln!("warning: mcap finish failed ({}): {e}", path.display());
                }
            }
            Err(e) => eprintln!("warning: mcap create failed ({}): {e}", path.display()),
        }
    }
    (result, checks)
}

/// Render the per-check row of the summary (checkbox list with details).
fn check_line(check: &CheckOutcome) -> String {
    let mark = if check.passed { "✅" } else { "❌" };
    format!("  - {mark} `{}` — {}", check.id, check.detail)
}

/// Summary Markdown table over all executed scenarios.
fn summary_markdown(entries: &[(String, Vec<String>, usize, usize)]) -> String {
    let mut md = String::from(
        "# Scenario run report\n\n\
        | scenario | result | checks passed |\n|---|---|---|\n",
    );
    for (name, _lines, passed, total) in entries {
        let status = if passed == total && *total > 0 {
            "PASS"
        } else {
            "FAIL"
        };
        let _ = writeln!(md, "| {name} | {status} | {passed}/{total} |");
    }
    for (name, lines, passed, total) in entries {
        let status = if passed == total && *total > 0 {
            "PASS"
        } else {
            "FAIL"
        };
        let _ = writeln!(md, "\n## {name} — {status} ({passed}/{total} checks)");
        for line in lines {
            md.push_str(line);
            md.push('\n');
        }
    }
    md
}

fn main() {
    let argv = std::env::args().skip(1);
    match parse_args(argv) {
        ParseOutcome::Help(text) => {
            println!("{text}");
        }
        ParseOutcome::UsageError(msg) => {
            eprintln!("{msg}");
            std::process::exit(2);
        }
        ParseOutcome::Args(parsed) => {
            let code = run_all(&parsed);
            std::process::exit(code);
        }
    }
}

/// Main flow; returns the process exit code.
fn run_all(args: &Args) -> i32 {
    let files = match collect_toml(&args.dir) {
        Ok(files) => files,
        Err(e) => {
            eprintln!("error: {e}");
            return 2;
        }
    };
    if files.is_empty() {
        eprintln!(
            "error: no scenario (*.toml) files found in {}",
            args.dir.display()
        );
        return 2;
    }

    let mut scenarios = Vec::new();
    for path in files {
        match load(&path) {
            Ok(s) => scenarios.push(s),
            Err(e) => {
                eprintln!("error: {e}");
                return 2;
            }
        }
    }

    if args.list_only {
        for s in &scenarios {
            println!("{} — {}", s.name, s.description);
        }
        return 0;
    }

    // Filter by requested names (a name must exist — typos must fail loudly).
    let selected: Vec<&Scenario> = if args.only.is_empty() {
        scenarios.iter().collect()
    } else {
        let selected: Vec<&Scenario> = scenarios
            .iter()
            .filter(|s| args.only.contains(&s.name))
            .collect();
        for name in &args.only {
            if !selected.iter().any(|s| &s.name == name) {
                eprintln!("error: no scenario named '{name}' in the selected dir");
                return 2;
            }
        }
        selected
    };

    let mut entries = Vec::new();
    let mut all_passed = true;
    for scenario in &selected {
        let (result, checks) = execute(scenario, args.no_csv, args.no_mcap, &args.out);
        let passed = checks.iter().filter(|c| c.passed).count();
        if passed != checks.len() {
            all_passed = false;
        }
        let lines = checks.iter().map(check_line).collect();
        entries.push((scenario.name.clone(), lines, passed, checks.len()));
        println!(
            "{}: {} checks, {passed} passed ({} rows logged)",
            scenario.name,
            checks.len(),
            result.rows.len()
        );
    }

    let summary = summary_markdown(&entries);
    let _ = std::fs::create_dir_all(&args.out);
    let summary_path = args.out.join("summary.md");
    if let Err(e) = std::fs::write(&summary_path, summary) {
        eprintln!("warning: cannot write {}: {e}", summary_path.display());
    }

    i32::from(!all_passed)
}
