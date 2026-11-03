# Safety analysis (HARA-lite)

> **Disclaimer.** This is an academic, lightweight hazard analysis and risk assessment
> done outside a real item definition, real OEM hazard catalogue, and real
> controllability studies. The ASIL ratings below are *assumed* to demonstrate the
> method, not certified claims. ISO 26262 expects workshops, DeM confirmation,
> validation measures, and more — those are out of scope and listed in
> `docs/limitations.md`.

## Item definition

Item: front windshield wiper control function (motor + washer pump control).
Element under analysis: the control SWC (this project). Vehicle maximum speed 250 km/h
(reference), urban/highway/expressway usage profile.

## Hazard analysis

| HE | Hazardous event | Operation sit. | Severity | Exposure | Controllability | ASIL calc (assumed) | Constraint |
|---|---|---|---|---|---|---|---|
| HE-01 | Unintended wiper stop in heavy rain at speed (blocked visibility) | City/highway in heavy rain | S2 (limited visibility, driver can pull over → moderate) | E4 (heavy rain driving is common) | C2 (driver notices, stops safely) | **A** | SG-001 |
| HE-02 | Unintended wiper activation at standstill with stalk OFF | Parking / still | S0/S1 | E4 | C1 | QM→A | SG-002 |
| HE-03 | Arm stops mid-windshield after `OFF` (blocked view) | Any | S2 | E4 | C1 (driver cannot fix) | **B** | SG-003 |
| HE-04 | Washer fluid on glass at high speed without rain (blind) | Highway | S1 | E2 | C1 | QM | REQ-WIP-010/016 |
| HE-05 | Motor stall fire/overheat | Any | S1 | E1 | C2 | QM | REQ-WIP-012 |

Severity/exposure/controllability values follow the ISO 26262‑3 annex scales as taught
in automotive safety engineering courses; the combination S2·E4·C2 ⇒ ASIL A, S2·E4·C1 ⇒
ASIL B (per the standard's lookup — reproduced here for the exercise).

→ Highest assumed ASIL of the item: **ASIL B** (HE-03), overall item treated ASIL A/B
mix; this drives the academic "ASIL A (documented as assumption)" statement of the
master prompt and the safe-state definition below.

## Safe state and derived technical constraints

Safe state: motor de-energized with arm in park; wiping only per explicit demand.
Derived constraints (traced back into requirements):

1. SG-001 ⇒ REQ-WIP-015 fallback to `LOW` on sensor loss (motion continuity),
   REQ-WIP-012 stall retry chain with latching.
2. SG-003 ⇒ REQ-WIP-008 park discipline (no stop outside park), REQ-WIP-013 park
   switch plausibility monitoring.
3. SG-002 ⇒ REQ-WIP-001 ignition gating.

## Safety mechanisms summary

| Mechanism | Detected fault | Reaction | Requirement |
|---|---|---|---|
| Stall monitor (I > 8 A ∧ no motion 500 ms) | blocked linkage / blade ice | motor off, retry chain, latch | REQ-WIP-012 |
| Park plausibility | stuck park switch | switch assumed failed → angle-based park | REQ-WIP-013 |
| Voltage supervision | under/over voltage | inhibit outputs | REQ-WIP-014 |
| Input timeouts | CAN/sensor loss | degraded mode (`LOW` / manual INT timing) | REQ-WIP-015 |
| Pump over-run watch | driver holds WASH | max 10 s + 3 s off interval | REQ-WIP-010/016 |
