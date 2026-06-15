# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
cmake -S . -B build
cmake --build build

# If CMake cannot find SFML automatically
cmake -S . -B build -DSFML_DIR=/path/to/SFML/lib/cmake/SFML
cmake --build build
```

Requirements: C++17 compiler, CMake, SFML 3.

## Running

```bash
# GUI mode
./build/simulation

# GUI with JSON config
./build/simulation --input-json examples/run_config.example.json

# Headless mode — used by the Python optimizer internally
./build/simulation --headless --input-json examples/run_config.example.json

# Run optimizer
make optimize
```

There are no automated tests — correctness is validated by running simulations.

## Architecture

The project simulates particles moving through an S-shaped corridor toward a target zone, driven by an external field and interacting via Lennard-Jones potential.

### Layer separation

**`simulation_core` (SFML-free static library)**
- `SimulationCore` — owns particles, corridor, zones, stats, and executes physics steps
- `Particle`, `Corridor`, `Zone` — data and geometry
- `PhysicsForces` — `getPotentialForce()` (external field) and `computePairForce()` (Lennard-Jones)
- `SimulationStats` — tracks per-particle target hits, computes `t80` and `quality`
- `RunConfig` — all runtime parameters for one simulation run
- `HeadlessSimulationRunner` — runs `SimulationCore` to completion, returns `HeadlessRunResult`
- `JsonRunIO` — loads partial/full `RunConfig` from JSON, serializes `HeadlessRunResult` to JSON

**`simulation_gui` (SFML-dependent static library)**
- `Simulation` — SFML window, event loop, rendering
- `CameraController`, `VisualConfig` — GUI-only concerns

**`simulation` executable** — `main.cpp` does CLI argument parsing, selects GUI or headless path.

### Data flow

```
RunConfig (JSON or defaults)
    → SimulationCore::reset()
    → SimulationCore::step() × N
    → SimulationStats (t80, quality)
    → JsonRunIO::serializeHeadlessResultJson()
    → stdout / --output-json file
```

### Field model

The external field (`Config::Field::Params`) is a piecewise force that drives particles through corridor segments (start → top horizontal → right vertical → bottom → target). Each segment has a drive force and a centering force. Segment boundaries are defined as constants in `Config::Field` (e.g. `TOP_SEGMENT_X_MAX`, `RIGHT_SEGMENT_X_MIN`). `getPotentialForce()` in `src/PhysicsForces.cpp` selects the active segment by particle position.

### Coordinate system

Origin is top-left, X right, Y down (SFML convention). World positions use `Config::World::grid()` helpers that multiply by `GRID_UNIT = 20.0f`.

### Quality metric

`SimulationStats::computeQuality()`:
- If 80% of particles reached `TargetZone`: `quality = t80`
- Otherwise: `quality = maxSimulationTime + penaltyWeight * missingTargetFraction`

Lower is better. The Python optimizer minimizes this value.

### JSON config format

JSON is partial-friendly — unspecified fields take defaults from `RunConfig`. `apiVersion: 1`. See `examples/run_config.json` and `examples/optimizer_config.json` for the full schema.

### Key files

| File | Purpose |
|------|---------|
| `include/Config.h` | All compile-time constants (particle size, LJ params, field defaults, corridor geometry) |
| `include/RunConfig.h` | Runtime parameters passed to a simulation run |
| `include/SimulationCore.h` | Central physics engine |
| `include/SimulationStats.h` | t80 / quality computation |
| `src/PhysicsForces.cpp` | Force field and Lennard-Jones implementation |
| `src/JsonRunIO.cpp` | JSON serialization/deserialization |
| `scripts/gradient_descent.py` | Numerical optimizer driving headless runs |
| `docs/potentials.md` | Physics formulas |
