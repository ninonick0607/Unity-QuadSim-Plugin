# Headless Tools

The `headless-tools/` directory contains standalone Python utilities that are useful when QuadSim runs without an interactive Unity window.

These tools are not PX4-specific. They are intended for any workflow where the sim runs headless or faster than real time:

- Geometric-controller experiments.
- PX4 SITL experiments.
- Optuna sweeps.
- Regression tests.
- Future dataset-generation or replay workflows.

## Directory

```text
headless-tools/
  README.md
  live_viewer.py
```

Future tools can be added here without tying them to PX4:

```text
headless-tools/
  live_viewer.py
  log_replay.py
  csv_plotter.py
  trajectory_preview.py
  trial_dashboard.py
```

## Live viewer

`live_viewer.py` is a standalone UDP 3D viewer.

Run:

```bash
python headless-tools/live_viewer.py --port 14660
```

Useful options:

```bash
python headless-tools/live_viewer.py \
  --port 14660 \
  --range 24 \
  --zmin 0 \
  --zmax 25 \
  --draw-hz 20
```

Controls:

```text
left-drag   rotate
scroll      zoom
right-drag  pan
```

## Packet format

The viewer listens for UDP JSON packets.

### Planned path packet

```json
{
  "type": "path",
  "points": [[0, 0, 2], [1, 0, 2], [2, 0, 2]]
}
```

### Sample packet

```json
{
  "type": "sample",
  "source": "experiment_name",
  "trial": 0,
  "t_rel": 1.25,
  "actual": [0.1, 0.2, 2.9],
  "target": [0.0, 0.0, 3.0],
  "err": 0.25,
  "speed": 1.4
}
```

The viewer accepts either list-style positions:

```json
"actual": [x, y, z]
```

or object-style positions:

```json
"actual": {"x": 0.0, "y": 0.0, "z": 3.0}
```

Unknown fields are ignored, so specialized experiments can include extra metadata.

## Coordinate convention

The generic viewer assumes:

```text
FLU, z up
```

That means:

```text
x = forward
y = left
z = up
```

If a PX4 script uses NED internally, it should convert or label the data before sending it to the generic viewer.

## Using from the Python SDK

For normal QuadSim SDK experiments, use `UdpViz`:

```python
from quadsim import UdpViz

viz = UdpViz(source="my-run")
viz.path(planned_points)

# inside the loop
viz.sample(
    t,
    pos,
    target=target_pos,
    err=err,
    speed=speed,
    trial=trial_id,
)
```

Start the viewer in another terminal:

```bash
python headless-tools/live_viewer.py --port 14660
```

## Using from PX4 tools

The PX4 offboard validation script can stream to the viewer:

```bash
python test_offboard_figure8_3d.py --viz-udp 127.0.0.1:14660
```

Then run:

```bash
python headless-tools/live_viewer.py --port 14660
```

## Design rule

Headless tools should be:

- Optional.
- Fire-and-forget.
- Non-blocking.
- Usable without Unity Editor.
- Usable with or without PX4.
- Safe to disable during timing-sensitive studies.

Avoid making control loops depend on viewers. The sim and controller should keep running even if no viewer is open.
