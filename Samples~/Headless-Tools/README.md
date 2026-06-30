# Headless Tools

This folder contains standalone Python tools for inspecting QuadSim runs when Unity is headless, minimized, or running faster than real time.

These tools are intentionally separate from PX4-specific files. They can be used by PX4 workflows, geometric-controller workflows, Optuna studies, and future automation.

## Current tools

```text
headless-tools/
  live_viewer.py
```

## Live viewer

Run:

```bash
python headless-tools/live_viewer.py --port 14660
```

Recommended PX4/trajectory view:

```bash
python headless-tools/live_viewer.py --port 14660 --range 24 --zmin 0 --zmax 25
```

Debug mode:

```bash
python headless-tools/live_viewer.py --port 14660 --debug
```

## Packet types

The viewer accepts UDP JSON.

### Path

```json
{
  "type": "path",
  "points": [[0, 0, 3], [1, 0, 3], [2, 1, 3]]
}
```

### Sample

```json
{
  "type": "sample",
  "source": "my_experiment",
  "trial": 0,
  "t_rel": 1.0,
  "actual": [0.0, 0.0, 3.0],
  "target": [1.0, 0.0, 3.0],
  "err": 1.0,
  "speed": 0.5
}
```

`actual` and `target` may also be objects:

```json
"actual": {"x": 0.0, "y": 0.0, "z": 3.0}
```

## Coordinate convention

The viewer is generic and assumes:

```text
FLU, z up
```

Do not mix NED and FLU in the same visualization stream unless the script clearly converts or labels the data.

## Design expectations for future tools

Tools in this folder should be:

- Standalone Python scripts.
- Safe to run separately from Unity.
- Optional for core simulation.
- Non-blocking for experiments.
- Useful for headless or faster-than-real-time runs.
- Documented with `--help`.

Good future candidates:

```text
log_replay.py
csv_plotter.py
trajectory_preview.py
trial_dashboard.py
telemetry_recorder.py
```
