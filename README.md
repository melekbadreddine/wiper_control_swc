# Automotive Wiper Control System (AUTOSAR Classic SWC)

A lightweight, fully-tested **AUTOSAR Classic 4.4 Application Software Component** (Atomic SWC)
modelling an automatic windshield wiper controller.

The project ships with **two independent validation paths**:

1. **Zero-dependency host execution** — a C test bench and an interactive CLI simulator that
   compile and run anywhere a C99 compiler exists. No MATLAB, no Simulink, no AUTOSAR toolchain.
2. **MATLAB / Simulink migration** — a defensive script that imports the ARXML into a Simulink
   model, attaches the 10 ms runnable logic, and configures the model for SIL.

The ARXML description is the single source of truth. The C implementation, the test bench, and the
generated Simulink logic all express the same state machine, and the host suite cross-checks them.

---

## Table of Contents

- [1. System Architecture](#1-system-architecture)
- [2. State Machine Logic](#2-state-machine-logic)
- [3. Repository Structure](#3-repository-structure)
- [4. Quick Start](#4-quick-start)
- [5. AUTOSAR Configuration Details](#5-autosar-configuration-details)
- [6. MATLAB and Simulink Migration Notes](#6-matlab-and-simulink-migration-notes)
- [7. Verification Summary](#7-verification-summary)
- [8. Known Limitations](#8-known-limitations)

---

## 1. System Architecture

The component is a single Atomic SWC with **two receiver ports** and **one sender port**. All
signal data is `uint8`.

| Direction | Port Name | Signal | Range | Description |
|:---------:|-----------|--------|-------|-------------|
| IN  | `RPort_StalkPosition` | Stalk Position | 0 – 3 | Driver-selected wiper mode |
| IN  | `RPort_RainIntensity` | Rain Intensity | 0 – 100 (%) | Sensor-reported rainfall level |
| OUT | `PPort_WiperSpeedCmd`  | Wiper Speed Cmd | 0 – 2 | Commanded wiper speed |

### Signal encoding

**`RPort_StalkPosition` (uint8)**

| Value | Meaning |
|:-----:|---------|
| `0` | OFF — driver selected wiper park |
| `1` | AUTO — automatic, rain-driven |
| `2` | LOW — manual low speed |
| `3` | HIGH — manual high speed |

**`RPort_RainIntensity` (uint8)** — percentage, `0` = dry, `100` = heaviest rain.

**`PPort_WiperSpeedCmd` (uint8)**

| Value | Meaning |
|:-----:|---------|
| `0` | OFF — wipers parked |
| `1` | LOW |
| `2` | HIGH |

### Data flow

```
  RPort_StalkPosition        RPort_RainIntensity
          (uint8)                    (uint8)
             |                          |
             +-----------+--------------+
                         |
                         v
            +---------------------------+
            |                           |
            |   Runnable_WiperControl   |   periodic, 10 ms
            |         _10ms             |   (PERIOD = 0.01 s)
            |                           |
            |     [ state machine ]     |
            +---------------------------+
                         |
                         v
            PPort_WiperSpeedCmd  (uint8)
```

---

## 2. State Machine Logic

The component implements four states, identified by the internal `Wc_State` value.

| State | ID | Output `WiperSpeedCmd` | Notes |
|-------|:--:|-----------------------|-------|
| OFF  | `0` | `0` | Wipers parked |
| AUTO | `1` | derived from rain | See threshold table below |
| LOW  | `2` | `1` | Manual latch |
| HIGH | `3` | `2` | Manual latch |

### AUTO thresholds

| Rain intensity | Wiper speed |
|:--------------:|:-----------:|
| `< 10 %`  | `0` (OFF) |
| `10 %` – `60 %` (inclusive) | `1` (LOW) |
| `> 60 %` | `2` (HIGH) |

The boundaries are exact as written: rain `9` → OFF, rain `10` → LOW, rain `60` → LOW, rain `61` → HIGH.

### Transition rules

| From | Condition | To |
|:----:|-----------|:--:|
| any | stalk `== 0` | OFF |
| OFF / AUTO | stalk `== 1` | AUTO |
| any | stalk `== 2` | LOW |
| any | stalk `== 3` | HIGH |
| any | stalk not in `0..3` | OFF |

### Manual latch behaviour

A manual LOW/HIGH selection **latches**: once the driver selects LOW or HIGH, the component stays
there even when the stalk is moved back to AUTO. The latch is released only when the stalk selects a
different value — in practice, OFF re-arms AUTO.

This means the driver always wins over the automatic mode for as long as the manual command is held.

### Fail-safe behaviour

| Fault | Result |
|-------|--------|
| Stalk value outside `0..3` | Command `0` (OFF) |
| `Rte_Read` of stalk returns an error | Command `0` (OFF) |
| `Rte_Read` of rain returns an error | Rain treated as `0` → AUTO commands OFF |

The component never leaves `PPort_WiperSpeedCmd` unwritten: exactly one write per runnable
invocation, which the test bench asserts.

---

## 3. Repository Structure

```
wiper_control_swc/
├── WiperControl_SWC.arxml      # AUTOSAR 4.4 SWC description (single source of truth)
├── WiperControl.c              # C state machine implementation
├── WiperControl.h              # Public interface, constants, runnable prototype
├── rte_mock/                   # Host stand-in RTE headers (signatures only)
│   ├── Std_Types.h
│   ├── Rte_Type.h
│   └── Rte_WiperControl_SWC.h
├── test_runner.c               # Non-interactive host test bench + mock RTE functions
├── cli_sim.c                   # Interactive terminal simulator + mock RTE functions
├── migrate_to_autosar.m        # MATLAB/Simulink ARXML import and SIL setup script
└── Makefile                    # Build, test, validate and migrate targets
```

| File | Purpose |
|------|---------|
| `WiperControl_SWC.arxml` | AUTOSAR 4.4 software-component description: ports, data types, runnable and 10 ms timing event. Consumed by the MATLAB importer and by any downstream AUTOSAR toolchain. |
| `WiperControl.c` / `.h` | The implementation. `WiperControl.h` also holds all tunable constants (speeds, thresholds, stalk encoding) and declares the runnable plus the `Wc_GetState()` diagnostic accessor. |
| `rte_mock/` | Header-only stand-ins for the RTE and standard types. They exist **only** so the SWC compiles on a workstation; on target these come from the RTE generator. The mock *functions* live in `test_runner.c` and `cli_sim.c`, not here. |
| `test_runner.c` | Automated regression bench. 7 scenarios producing **32 assertions**, including boundary-condition sweeps and a 249-combination exhaustive matrix compared against an independently written reference model. Exits non-zero on failure. |
| `cli_sim.c` | Interactive simulator for manually exploring the state machine. |
| `migrate_to_autosar.m` | MATLAB function that imports the ARXML, builds the Simulink model, attaches the logic and configures SIL. |
| `Makefile` | `all`, `run`, `cli`, `validate-xml`, `check`, `matlab-migrate`, `clean`. |

---

## 4. Quick Start

```bash
cd /home/melek/wiper_control_swc
```

### `make run` — automated test suite

Compiles and runs the full regression bench.

```
$ make run
gcc -Wall -Wextra -std=c99 -Irte_mock test_runner.c WiperControl.c -o wiper_sim

--- Scenario A: AUTO mode rain-intensity mapping ---
  [PASS] AUTO rain mapping                              stalk=1 rain=  0% -> speed=0
  [PASS] AUTO rain mapping                              stalk=1 rain=  9% -> speed=0
  [PASS] AUTO rain mapping                              stalk=1 rain= 10% -> speed=1
  [PASS] AUTO rain mapping                              stalk=1 rain= 60% -> speed=1
  [PASS] AUTO rain mapping                              stalk=1 rain= 61% -> speed=2
...
======================================================================
 Result: 32/32 passed, 0 failed
======================================================================
```

Exit code `0` on success, `1` on any failure — suitable for CI.

### `make cli` — interactive simulator

```
$ make cli
 WiperControl_SWC interactive simulator
 Runnable: Runnable_WiperControl_10ms (10 ms period)
======================================================================
 Stalk encoding: 0=OFF 1=AUTO 2=LOW 3=HIGH   (project assumption)
 Rain thresholds: <10% off, 10..60% low, >60% high
 Type 'h' for help, 'q' to quit.

> r 65
  Rain set to 65%
  ------------------ status ------------------
   Stalk Position : 0 (OFF)
   Rain Intensity : 65%
   Wiper Speed Cmd: 0 (OFF )
   Active State   : 0 (OFF )
   Cycles Run     : 1
  -------------------------------------------

> s 1
  Stalk set to AUTO
  ------------------ status ------------------
   Stalk Position : 1 (AUTO)
   Rain Intensity : 65%
   Wiper Speed Cmd: 2 (HIGH)
   Active State   : 1 (AUTO)
   Cycles Run     : 2
  -------------------------------------------
```

| Command | Effect |
|---------|--------|
| `s <0-3>` | Set stalk position, then run one cycle |
| `r <0-100>` | Set rain intensity, then run one cycle |
| `t` | Trigger a single cycle |
| `c <n>` | Run `n` consecutive cycles (max 10000) |
| `a` | AUTO sweep — one cycle per stalk at the current rain |
| `show` | Print status without running a cycle |
| `reset` | Restore power-on defaults |
| `h` / `?` | Help |
| `q` | Quit |

> The `Active State` readout comes from `Wc_GetState()`. It is worth watching at 0 % rain, where
> AUTO produces speed `0` while the state is still `AUTO` — output and state are not the same thing.

### `make validate-xml` — ARXML well-formedness

```
$ make validate-xml
[validate-xml] xmllint not found, falling back to python3
[validate-xml] OK - WiperControl_SWC.arxml is well-formed
```

Uses `xmllint --noout` when libxml2 is installed, otherwise falls back to Python's `xml.etree`.
Both check well-formedness only — **not** schema conformance against the AUTOSAR 4.4 XSD.

### `make check` — full host validation

Runs the test suite and the XML check together.

### `make matlab-migrate` — Simulink migration

```bash
make matlab-migrate                                  # uses `matlab` from PATH
make matlab-migrate MATLAB=/opt/R2023b/bin/matlab    # explicit interpreter
```

Fails fast with an actionable message if MATLAB is not found. See section 6.

### `make clean`

Removes `wiper_sim`, `wiper_cli`, object files, and generated `slprj/` and `codegen/` trees.

---

## 5. AUTOSAR Configuration Details

| Property | Value |
|----------|-------|
| Software Component Type | **Atomic SWC** (`APPLICATION-SW-COMPONENT-TYPE`) |
| ARXML schema | AUTOSAR 4.4 (`http://autosar.org/schema/r4.0`) |
| Runnable | `Runnable_WiperControl_10ms` |
| Trigger | Periodic **TimingEvent**, `PERIOD = 0.01` s |
| Minimum start interval | `0.0` s |
| Concurrency | `CAN-BE-INVOKED-CONCURRENTLY = false` |
| Multiple instantiation | `SUPPORTS-MULTIPLE-INSTANTIATION = false` |
| Data consistency | **EXPLICIT** on all three port data points |
| Data types | `uint8` (referenced from the platform base type) |

### Why `EXPLICIT` data consistency

The AUTOSAR default (implicit) would generate short RTE accessors. Setting `EXPLICIT` on the
data receiver/sender points produces the fully-qualified signatures the implementation uses:

```c
Std_ReturnType Rte_Read_RPort_StalkPosition_DE_StalkPosition(uint8 *const data);
Std_ReturnType Rte_Read_RPort_RainIntensity_DE_RainIntensity(uint8 *const data);
Std_ReturnType Rte_Write_PPort_WiperSpeedCmd_DE_WiperSpeedCmd(uint8 data);
```

The `_DE_<DataElement>` suffix is a direct consequence of that ARXML setting — change the
consistency and the generated names change with it.

### Public API

```c
FUNC(void,  WIPERCONTROL_CODE) Runnable_WiperControl_10ms(void);
FUNC(uint8, WIPERCONTROL_CODE) Wc_GetState(void);
```

`Wc_GetState()` is a read-only diagnostic accessor used by the CLI and test bench. It is additive
and has no side effects.

---

## 6. MATLAB and Simulink Migration Notes

`migrate_to_autosar.m` is a **function file**, invoked as `migrate_to_autosar` (optionally with
name–value arguments such as `'Logic','caller'` or `'Build',false`).

### Defensive architecture

The script is deliberately written so that it fails **loudly and specifically** rather than
crashing with an opaque `Undefined function` error:

| Technique | Purpose |
|-----------|---------|
| **Preflight checks** | Verifies the ARXML exists, Simulink is present and the AUTOSAR Blockset is installed *before* writing anything |
| **Dynamic importer resolution** | Probes for `autosar.ui.importer`, then `autosar.ui.simulinkImporter`, and reports which was used |
| **Isolated API-critical calls** | Every Blockset-specific call is confined to `localImportArxml`, `localAutosarTarget`, `localMapRunnable` and `localBuild` — a wrong API name is a one-line fix in one place |
| **Graceful degradation** | If the ARXML import fails, the script builds the port skeleton by hand from the ARXML contents and continues |
| **Guarded optional steps** | Missing runnable-mapping or SIL APIs produce a warning, not an abort |
| **Specific error IDs** | `AUTOSAR:BlocksetMissing`, `AUTOSAR:NoImporter`, `AUTOSAR:ImporterApiUnknown`, `AUTOSAR:BuildFailed` — each names the specific missing piece rather than failing generically |

### Timing synchronisation

The model is configured with a fixed-step discrete solver at `0.01` s and the chart sample time is
set to 10 ms. This **matches the `PERIOD = 0.01` already declared in `WiperControl_SWC.arxml`**,
so the model and the exported description agree on the runnable period.

> Note: `Stateflow.Chart.SampleTime` is expressed in **milliseconds**, unlike Simulink block
> parameters which use seconds. The script sets `ch.SampleTime = 10`.

### Logic generation

| Mode | Block | Notes |
|------|-------|-------|
| `stateflow` *(default)* | Stateflow chart | Self-contained; reproduces the four states and all rain thresholds. Preferred. |
| `caller` | C Caller | Binds to `WiperControl.c`. **Requires** an RTE implementation — see below. |

### C Caller caveat

`WiperControl.c` calls `Rte_Read_*` / `Rte_Write_*`, which exist only in a generated RTE or in the
host mock. A C Caller block bound to it directly will **not link** in SIL unless those symbols are
supplied. The script prints this warning when `caller` mode is selected. The Stateflow default
avoids the dependency entirely.

---

## 7. Verification Summary

| Check | Status |
|-------|--------|
| `WiperControl.c` compiles under `-Wall -Wextra -pedantic -Werror` | Clean |
| Automated test suite | **32/32 assertions pass** |
| Exhaustive matrix (5 stalk values × 101 rain values) | **249 combinations match** the reference model |
| ARXML well-formedness | Valid |
| Interactive CLI session | Verified end-to-end |

The exhaustive sweep compares the SWC against `ReferenceSpeed()`, a reference implementation
written independently of the component, so the test compares two separate expressions of the
requirement rather than the code against itself.

Coverage includes the exact threshold boundaries (9/10/60/61 %), manual-latch release and re-arm,
invalid stalk values, injected RTE read failures, and a 100-cycle drift check.

---

## 8. Known Limitations

Read this before treating the component as production-ready.

1. **The stalk encoding is a project assumption.** `0=OFF, 1=AUTO, 2=LOW, 3=HIGH` is *not*
   mandated by AUTOSAR. It is flagged as an assumption in `WiperControl.h` and in the CLI banner.
   **Confirm it against the real signal matrix** — if it is wrong, the table in
   `Wc_ProcessStalkPosition()` is the only place to change.

2. **The latch rule is an assumption.** LOW/HIGH latching over AUTO was inferred from the state
   model, not specified. Tests C and E encode it. If the intended behaviour is for AUTO to preempt
   the driver immediately, both the code and those tests need updating together.

3. **No init runnable.** The component defines only the 10 ms periodic runnable. A production
   Atomic SWC would normally also provide `Runnable_WiperControl_init` for one-time startup
   initialisation.

4. **XML well-formedness ≠ schema validity.** `validate-xml` does not validate against the
   AUTOSAR 4.4 XSD, and `/Platform/uint8` is a platform reference resolved only when the ARXML is
   merged with the platform description during RTE generation.

5. **`migrate_to_autosar.m` has never been executed.** No MATLAB was available on the authoring
   machine, and the AUTOSAR Blockset API names could not be verified against vendor documentation.
   Only the standard Simulink/Stateflow sections could be reasoned about with confidence. Review
   the `API-CRITICAL` blocks before the first run.

6. **Fail-safe defaults rely on local initialisation.** RTE read failures are handled by
   `(void)Rte_Read_...` with pre-initialised locals defaulting to the safe value. This is correct
   but relies on the SWC's own defaulting rather than on an explicit error branch.

---

## License

Provided as-is for engineering evaluation.
