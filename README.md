# ADS Geometric Fault Injector

This module injects controlled geometric faults into waypoint-based roads to test ADS robustness under map and geometry corruption scenarios.

Its main aim is to provide a reproducible and configurable way to stress-test driving behavior before full closed-loop evaluation, so that failures can be analyzed systematically instead of relying on ad-hoc scenario selection.

# Core objectives:

- Create realistic geometric perturbations that emulate map corruption and localization inconsistencies
- Expose weak points in path tracking, curve handling, and recovery behavior
- Run attacks with deterministic seeds for repeatable experiments
- Keep perturbed roads valid enough for simulation while remaining challenging

Attack logic in brief:

- Each attack operates directly on waypoint geometry using a dedicated fault model and severity control
- F1 displaces a subset of points (sparse targeted corruption)
- F2 injects a smooth lateral curvature bump in a selected road window
- F3 removes waypoint segments, prioritizes difficult regions, then repairs excessive gaps to preserve runnability
- F4 adds a distributed Gaussian perturbation to all waypoints (global noise model)

## What It Contains

- `__init__.py`: public exports for package-style imports.
- `fault_injector.py`: core fault taxonomy, config model, injector logic, and validation helpers.
- `integrate.py`: integration notes/examples for plugging the injector into a generator pipeline.
- `requirements.txt`: minimal runtime dependency list.

## Active Fault Set

The active attack taxonomy is now:

- `F1_waypoint_displacement`
- `F2_curvature_injection`
- `F3_waypoint_dropout`
- `F4_noise_injection`

## Core API

### `FaultConfig`
Defines one attack configuration:

- `fault_type`: which attack to run
- `severity`: main intensity control
- `seed`: reproducible random behavior
- `affected_fraction`: F1 only
- `window_size`, `inject_at`: F2 only
- `contiguous`, `dropout_blocks`: F3 only

### `FaultInjector`
Main methods:

- `inject(road) -> perturbed_road`

Input road format is a list of waypoints, typically `[x, y, z, width]` (x/y are modified; other fields are preserved).

## Attack Definitions

### F1 - Waypoint Displacement (GPS Spoofing Proxy)
Randomly selects a fraction of waypoints and shifts each by a fixed magnitude in random directions.

- Severity meaning: displacement in meters.
- Useful for: localized coordinate corruption.

![F1 attack visualization placeholder](assets/F1_wp_displace.gif)

### F2 - Curvature Injection (Map Geometry Corruption)
Applies a smooth lateral sinusoidal bump over a contiguous window to create an artificial sharp curve.

- Severity meaning: max lateral bump amplitude in meters.
- Placement: `peak`, `entry`, or `random`.
- Useful for: testing turn-following fragility and control stability.

![F2 attack visualization placeholder](assets/F2_curv_inject.gif)

### F3 - Waypoint Dropout (Map Data Loss)
Removes a fraction of interior waypoints while preserving first/last points.

Updated behavior:

- Random mode: scattered removals.
- Contiguous mode: one or more contiguous missing blocks (`dropout_blocks`).
- Contiguous dropout prefers high-curvature regions to create stronger disturbances.
- Boundary kinks are added around dropped runs to increase local difficulty.
- Gap-repair logic reinserts points as needed so resulting roads stay within validation gap limits.

- Severity meaning: fraction of interior points removed.
- Useful for: sparse/corrupted map segments and interpolation stress.

![F3 attack visualization placeholder](assets/F3_wp_drop.gif)

### F4 - Noise Injection (Sensor/Map Noise Proxy)
Adds zero-mean Gaussian noise to x/y of all waypoints.

- Severity meaning: Gaussian sigma in meters.
- Useful for: low-to-high amplitude global perturbation.

![F4 attack visualization placeholder](assets/F4_noise_inject.gif)

## Minimal Usage

Install dependencies first:

```bash
pip install -r requirements.txt
```

```python
from fault_injector import FaultConfig, FaultInjector, FaultType

road = [
    [100.0, 100.0, 0.0, 3.0],
    [100.5,  99.6, 0.0, 3.0],
    [101.0,  99.2, 0.0, 3.0],
]

cfg = FaultConfig(
    fault_type=FaultType.WAYPOINT_DROPOUT,
    severity=0.10,
    seed=42,
    contiguous=True,
    dropout_blocks=2,
)

injector = FaultInjector(cfg)
perturbed = injector.inject(road)
```

Package-style import also works:

```python
from fault_inject import FaultConfig, FaultInjector, FaultType
```

## Reproducibility

- Injection is deterministic for a fixed config seed.
- Repeated calls use deterministic per-call stream offsets to avoid identical randomness on every call.

## Validation Helper

Use `validate_road(road, original_len)` for basic sanity checks:

- minimum waypoint count
- excessive dropout detection
- max gap threshold
- near-duplicate waypoint detection

## Run Unit Tests

From repository root:

```bash
python3 -m unittest fault_inject.tests.test_fault_injector -v
```

## Typical Severity Bands

Approximate guidance from the module presets:

- Mild
- Moderate
- Strong
- Severe
- Extreme

Check `SEVERITY_SWEEPS` and `SEVERITY_LABELS` in `fault_injector.py` for exact values.

## Usage

```bash

python3 main.py --fault-type F1_waypoint_displacement --fault-severity 0.5 --fault-label moderate

python3 main.py --fault-type F2_curvature_injection --fault-severity 1.2 --fault-label strong --fault-inject-at peak --fault-window-size 12

python3 main.py --fault-type F3_waypoint_dropout --fault-severity 0.10 --fault-label strong --fault-contiguous-dropout

python3 main.py --fault-type F4_noise_injection --fault-severity 0.1 --fault-label moderate

```
