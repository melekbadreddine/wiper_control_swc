# AUTOSAR Classic vs Adaptive — platform choice analysis

The windscreen wiper control function is embedded in the **Classic Platform** corner
of AUTOSAR. The analysis below is the criteria table that justifies it, and the
boundary where an **Adaptive** ECU could legitimately interact with this function.

| Criterion | Classic Platform | Adaptive Platform | Wiper function | Verdict |
|---|---|---|---|---|
| ECU target hardware | MCU (e.g. ARM Cortex-M/R), flash+RAM in MB | SoC (Cortex-A), GB memory | Microcontroller-class function, driver at the edge | Classic |
| Timing model | Hard real-time, static schedule, 10 ms tick | Soft real-time, POSIX processes | Deterministic 10 ms control loop | Classic |
| Configuration | Completely static at build time (ARXML) | Dynamic service discovery, apps at runtime | Fixed function, strictly sensor→actuator | Classic |
| Communication | Signal-based (CAN/LIN, DBC/ARXML PDU mapping) | Service-oriented (SOME/IP, DDS) | Plain CAN signals, no service discovery | Classic |
| Safety integrity | Functionally safe stack up to ASIL D patterns (memory partition via MPU, protected scheduler) | Designed for safety+security on SoC, isolation heavier | Mixed ASIL A/B academic assumption | Classic |
| Memory/footprint | < 1 MB typical | > 100 MB runtime | Trivial | Classic |
| Determinism of execution | Runnable model (init/run cyclic, data-received) | Async, libraries, reunions with time-triggered optional | Runnable model fits directly | Classic |

Where Adaptive fits: a camera-based rain detection service (image processing is
Adaptive territory) would publish rain intensity as a **service** to this Classic ECU
over the vehicle network (Adaptive↔Classic signal/service mapping is outside this
project's simulation but documented). Visualization, recording, and driver preferences
(cloud OTA profiles) would also live on Adaptive/Offboard.

Conclusion: implement the wiper control as a Classic-style SWC (runnables, ports,
ARXML) and expose an optional Adaptive-side client only via ROS 2 adapters (this
project's ROS 2 layer plays the "research/hil lane", not a safety path).
