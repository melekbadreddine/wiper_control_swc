# Scenario runner (`scenario-runner`)

Headless, deterministic closed-loop scenario execution for milestone M5: TOML
scenario files are applied onto [`bsw_sim::Ecu`] (see `docs/bsw-sim.md`), one
10 ms tick at a time; every tick is recorded as a CSV row and a JSON MCAP
message; the scenario's checks are evaluated against the run.

Everything in this document is re-checkable:

```console
$ make sim                                 # runs all scenarios (release build)
$ cargo test -p scenario-runner            # 6 acceptance tests incl. golden CSVs
```

## CLI

```console
$ ./target/release/scenario-runner --scenario-dir scenarios --out report/ \
      [--scenario NAME]... [--no-csv] [--no-mcap] [--list]
```

* Loads every `*.toml` in `--scenario-dir` (sorted by file name); `--scenario`
  selects a subset by the scenario `name` field (unknown names fail loudly).
* Writes `report/<name>.csv` (unless `--no-csv`), `report/<name>.mcap`
  (unless `--no-mcap`) and `report/summary.md`.
* Exit codes: `0` all checks pass, `1` check failure, `2` usage/IO/parse error.
* `make sim` = release build + run all; `make check` gates the workspace
  including the scenario acceptance tests.

## Scenario file schema (`scenarios/*.toml`)

```toml
name = "int_auto_rain"        # unique; report file names derive from it
description = "..."
duration_ms = 20000           # virtual runtime, 10 ms OS ticks

[setup]                       # applied at virtual time 0
ignition = "on"               # "off" / "acc" / "on"
[setup.stalk]
mode = "int"                  # "off" / "int" / "low" / "high"
sensitivity = 2               # level 1..=4
auto = true

[[events]]                    # applied before the first tick at ≥ at_ms
at_ms = 12000
[events.rain]
pct = 20                      # partial: absent fields keep their value

[[events]]
at_ms = 1500
fault = "stalk_offline"       # stalk_offline | rain_offline | speed_offline |
                              # blocked_linkage | unblock_linkage |
                              # park_stuck_on | park_switch_normal | clear_bus

[[checks]]
kind = "wipe_gap_after"
after_ms = 12000
min = 5500
max = 7500
```

Event fields: `stalk`, `rain`, `speed` (partial content), `ignition`,
`fault`, `battery_v`. Unknown fields in map sections are rejected
(`deny_unknown_fields`), so typos fail loudly; the `checks` enum is
internally tagged (`kind = ...`) — serde does not support
`deny_unknown_fields` there, see ADR-0006.

### Checks

| `kind` | Fields | Meaning |
|---|---|---|
| `wipe_rate` | `min`, `max` | completed wipes per minute over the whole run |
| `wipe_count` | `min`, `max` | completed wipes at end of run |
| `wipe_gap_after` | `after_ms`, `min`, `max` | gap between the first **two** completions that both end at ≥ `after_ms` (measures the pause *after* a mid-run change) |
| `final_fsm` | `value` | `parked` / `int_pause` / `sweeping` / `returning_to_park` / `not_sweeping` |
| `dtc` | `dtc`, `active`, `latched` | DTC state at end of run (booleans optional = don't-care) |
| `no_active_dtcs` | — | active DTC bitset empty at end of run |
| `arm_in_park` | — | final arm angle ≤ 2 deg (plant feedback) |

## Shipped scenarios

| File | Covers | Checks |
|---|---|---|
| `low_rate_baseline` | REQ-WIP-003 (LOW ≈45 cpm), clean diagnostics | wipe_rate 40–50, no_active_dtcs, wipe_count ≥ 6 |
| `high_rate_baseline` | REQ-WIP-004 (HIGH ≈65 cpm) | wipe_rate 60–73, no_active_dtcs |
| `int_auto_rain` | REQ-WIP-007 (auto continuous wipe when rain ≥ threshold), REQ-WIP-005 (rain below threshold → manual S2 pause 5 s) | wipe_count ≥ 8, wipe_gap_after 12000 → 5500–7500 ms, no_active_dtcs |
| `stalk_loss_fallback` | REQ-WIP-015 (stalk alive-supervision trip → LOW fallback, `InputTimeout`) | dtc InputTimeout active+latched, wipe_rate 30–50, wipe_count ≥ 3 |
| `undervoltage_recovery` | REQ-WIP-014 (8.5 V inhibit → DTC, 14 V recovery with 1 s debounce) | dtc Undervoltage inactive+latched, wipe_count ≥ 4, no_active_dtcs |
| `blocked_linkage_retry_latch` | REQ-WIP-012 (stall → retries → latch; ignition cycle clears the latch) | dtc MotorStall inactive+latched, wipe_count ≥ 3 |

Measured outcomes (see `make sim` output / `report/summary.md`): all six
pass their checks; **verified by `all_scenarios_pass_their_checks`** in the
test suite so `make check` fails if a check or a behavior regresses.

## Output formats

* **CSV** — 16 columns per 10 ms tick:
  `t_ms, wipe_count, fsm, wash_phase, motor_speed, duty_pct, pump_on,
  dtc_active_bits, dtc_latched_bits, arm_deg, arm_speed_dps, current_da,
  battery_v01, rain_pct, speed_kph, stalk_valid`.
  The two baselines are byte-stable and committed under
  `crates/scenario-runner/tests/golden/` (REQ-WIP-019 regression guard).
* **MCAP** — one message per tick on channel `/wiper/state`
  (`msg_encoding = "json"`, schema `wiper.JsonState` as a JSON-schema stub),
  `log_time = t_ms × 10⁶ ns`; hand-serialized JSON payload of the same row
  fields (no `serde_json`; ADR-0006). Openable by Foxglove Studio (M8).
* DTC bit order in `dtc_*_bits`: `MotorStall=0, ParkSwitchImplausible=1,
  Undervoltage=2, Overvoltage=3, InputTimeout=4, PumpOverrun=5`
  (`wiper_swc::Dtc::ALL` order).

## Known scope limits

In `docs/limitations.md`: integer/fixed-schema rows only (no dynamic
channels), single JSON channel per run, no MCAP compression features enabled
(`default-features = false`).
