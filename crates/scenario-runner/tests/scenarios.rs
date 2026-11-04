//! M5 scenario acceptance tests: parsing, golden CSV capture, determinism,
//! check evaluation, and the REQ-WIP-020 worst-case-timing gate.

use std::collections::HashSet;
use std::path::PathBuf;

use scenario_runner::output::SnapshotRow;
use scenario_runner::runner;
use scenario_runner::scenario::Scenario;

fn scenarios_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// Load all `*.toml` scenarios from `scenarios/` (sorted). `Err` carries the
/// parser/IO text.
fn load_all() -> Result<Vec<Scenario>, String> {
    let mut paths = Vec::new();
    let entries = std::fs::read_dir(scenarios_dir()).map_err(|e| format!("read_dir: {e}"))?;
    for entry in entries {
        match entry {
            Ok(entry) if entry.path().extension().is_some_and(|e| e == "toml") => {
                paths.push(entry.path());
            }
            Err(e) => return Err(format!("entry: {e}")),
            _ => {}
        }
    }
    paths.sort();
    let mut scenarios = Vec::new();
    for path in paths {
        let text =
            std::fs::read_to_string(&path).map_err(|e| format!("read {}: {e}", path.display()))?;
        scenarios.push(toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?);
    }
    Ok(scenarios)
}

/// Load all scenarios and fold any load problem into `problems` (tests use
/// one `assert!(problems.is_empty(), …)` at the end, keeping the strict lints
/// happy without `unwrap`/`panic`).
fn collect_scenarios(problems: &mut Vec<String>) -> Vec<Scenario> {
    match load_all() {
        Ok(scenarios) if scenarios.len() == 6 => scenarios,
        Ok(scenarios) => {
            problems.push(format!("expected six scenarios, found {}", scenarios.len()));
            scenarios
        }
        Err(e) => {
            problems.push(e);
            Vec::new()
        }
    }
}

/// Scenario by name; missing names fold into `problems`.
fn collect_named<'a>(
    scenarios: &'a [Scenario],
    name: &str,
    problems: &mut Vec<String>,
) -> Option<&'a Scenario> {
    if let Some(scenario) = scenarios.iter().find(|s| s.name == name) {
        Some(scenario)
    } else {
        problems.push(format!("scenario '{name}' missing"));
        None
    }
}

/// Full generated CSV text (header + rows) for one named scenario.
fn baseline_csv(scenario: &Scenario) -> String {
    let result = runner::run(scenario);
    let csv = result
        .rows
        .iter()
        .map(SnapshotRow::csv_line)
        .collect::<String>();
    format!("{}\n{csv}", SnapshotRow::csv_header())
}

const GOLDEN_NAMES: [&str; 2] = ["high_rate_baseline", "low_rate_baseline"];

/// The two baselines are byte-identical to the committed golden CSVs →
/// verifies REQ-WIP-019 on the regression level.
#[test]
fn baselines_match_golden_csvs() {
    let mut problems = Vec::new();
    let scenarios = collect_scenarios(&mut problems);
    for name in GOLDEN_NAMES {
        let Some(scenario) = collect_named(&scenarios, name, &mut problems) else {
            continue;
        };
        let generated = baseline_csv(scenario);
        let golden_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/golden")
            .join(format!("{name}.csv"));
        match std::fs::read_to_string(&golden_path) {
            Ok(golden) if golden != generated => {
                problems.push(format!("{name}: generated CSV diverges from golden"));
            }
            Err(e) => problems.push(format!("{name}: golden read: {e}")),
            Ok(_) => {}
        }
    }
    assert!(problems.is_empty(), "golden CSV mismatch: {problems:?}");
}

/// Running the same scenario twice produces identical rows → REQ-WIP-019.
#[test]
fn scenario_runs_are_deterministic() {
    let mut problems = Vec::new();
    let scenarios = collect_scenarios(&mut problems);
    let csv_two_runs = collect_named(&scenarios, "low_rate_baseline", &mut problems).map(|s| {
        let a = runner::run(s);
        let b = runner::run(s);
        let csv = |rows: &[SnapshotRow]| rows.iter().map(SnapshotRow::csv_line).collect::<String>();
        (csv(&a.rows), csv(&b.rows))
    });
    match csv_two_runs {
        Some((a, b)) if a != b => {
            let first_diff = a
                .bytes()
                .zip(b.bytes())
                .position(|(a, b)| a != b)
                .unwrap_or(0);
            problems.push(format!("two runs diverge at byte {first_diff}"));
        }
        _ => {}
    }
    assert!(problems.is_empty(), "determinism: {problems:?}");
}

/// Every shipped scenario passes every check it declares (executable
/// acceptance of M5; failures print the per-check details).
#[test]
fn all_scenarios_pass_their_checks() {
    let mut problems = Vec::new();
    let scenarios = collect_scenarios(&mut problems);
    for scenario in &scenarios {
        if scenario.checks.is_empty() {
            continue;
        }
        let result = runner::run(scenario);
        let failed: Vec<String> = runner::evaluate(scenario, &result)
            .into_iter()
            .filter(|c| !c.passed)
            .map(|c| format!("{}: {}", c.id, c.detail))
            .collect();
        if !failed.is_empty() {
            problems.push(format!("{}: {failed:?}", scenario.name));
        }
    }
    assert!(problems.is_empty(), "scenario checks: {problems:?}");
}

/// All shipped scenarios parse and all names are unique.
#[test]
fn all_scenarios_parse_with_unique_names() {
    let mut problems = Vec::new();
    let scenarios = collect_scenarios(&mut problems);
    let names = scenarios
        .iter()
        .map(|s| s.name.clone())
        .collect::<HashSet<_>>();
    if names.len() != scenarios.len() {
        problems.push(format!(
            "duplicate scenario names among {:?}",
            scenarios.iter().map(|s| s.name.clone()).collect::<Vec<_>>()
        ));
    }
    assert!(problems.is_empty(), "scenario parse: {problems:?}");
}

/// CSV writer emits the header plus 16-column line bodies.
#[test]
fn csv_well_formed_columns() {
    let mut problems = Vec::new();
    let scenarios = collect_scenarios(&mut problems);
    if let Some(scenario) = collect_named(&scenarios, "low_rate_baseline", &mut problems) {
        let csv = baseline_csv(scenario);
        let bad: Vec<usize> = csv
            .lines()
            .enumerate()
            .filter(|(_, line)| line.split(',').count() != 16)
            .map(|(i, _)| i)
            .collect();
        if !bad.is_empty() {
            problems.push(format!("CSV line(s) without 16 columns at: {bad:?}"));
        }
    }
    assert!(problems.is_empty(), "CSV shape: {problems:?}");
}

// ------------------------------------------------------------- REQ-WIP-020

/// verifies: REQ-WIP-020 — worst-case `Ecu::tick()` (one `Wiper_Run10ms`
/// including the full BSW pipeline and 10 RK4 plant steps) stays below the
/// 1 ms host timing budget across 1000 consecutive 10 ms ticks.
#[test]
fn run10ms_worst_case_under_1ms() {
    use std::time::{Duration, Instant};

    let mut ecu = bsw_sim::Ecu::new();
    ecu.set_ignition(wiper_swc::Ignition::On);
    ecu.sensors_mut().stalk.mode = wiper_swc::StalkMode::Low;

    let mut worst = Duration::ZERO;
    for _ in 0..1000 {
        let start = Instant::now();
        ecu.tick();
        let elapsed = start.elapsed();
        if elapsed > worst {
            worst = elapsed;
        }
    }
    assert!(
        worst < Duration::from_millis(1),
        "worst 10 ms tick took {} µs (budget 1000 µs)",
        worst.as_micros()
    );
}
