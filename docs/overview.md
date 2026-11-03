# Overview

The project recreates a model-based development workflow for an automotive ECU
function without MATLAB/Simulink:

```mermaid
flowchart LR
    R[Requirements\nREQ-WIP-*] --> A[Architecture\nAUTOSAR-style ports/runnables]
    A --> M[Executable model\nwiper-swc (no_std Rust)]
    M --> S[Closed-loop simulation\nplant-sim + bsw-sim]
    S --> V[Verification\nunit/property/scenario/golden tests]
    V -->|failed| R
    V -->|passed| X[ARXML, Docker, ROS 2, Foxglove]
```

See the chapters in the table of contents for details. Every claim in the docs is
backed by a committed command output (see `docs/verification.md` once milestone M5+
ships real runs).
