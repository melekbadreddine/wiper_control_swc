# wiper-control-swc

Windshield wiper control software component modeled after **AUTOSAR Classic Platform**,
written in **Rust**, simulated as a closed loop (plant model + ECU) and visualized in
**Foxglove Studio** with a 3D URDF car, live plots and gauges. ROS 2 (Jazzy) adapters
are provided; the ECU logic itself is ROS-independent (hexagonal architecture).

> Work in progress — see `PROGRESS.md` for the milestone checklist and `AGENTS.md`
> for project conventions. Documentation site: `make docs` (mdBook under `docs/`).

## Quick start (native)

```bash
make check   # fmt + clippy (warnings as errors) + all tests
make sim     # run headless closed-loop scenarios
```

## Quick start (Docker)

```bash
make demo    # compose up ECU + plant + ROS bridge; connect Foxglove to ws://localhost:8765
```

## License

MIT — see `LICENSE`.
