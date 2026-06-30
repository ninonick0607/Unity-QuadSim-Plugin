# PX4 SITL Tools

This sample contains Python helper scripts for running PX4 SITL with QuadSim.

QuadSim provides the simulated plant. PX4 provides the flight controller. The bridge connects the two in lockstep.

## Contents

```text
PX4 SITL Tools/
  README.md
  requirements.txt
  PX4_VERSION.md
  px4_bridge.py
  px4_confirm.py
  test_offboard_figure8_3d.py
```

Generic visualization tools are not stored here. They live at the repo root:

```text
headless-tools/
  live_viewer.py
```

## Install dependencies

From this folder:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

If the SDK is not available from PyPI yet, install it locally:

```bash
pip install -e /path/to/QuadSimLib
```

## Run order

Use four terminals.

### Terminal 1 — QuadSim

Run QuadSim in the Editor or as a headless build.

Headless example:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556
```

### Terminal 2 — Bridge

```bash
python px4_bridge.py --speed 0
```

If PX4 is the TCP server and waits for simulator connection, run:

```bash
python px4_bridge.py --px4-client
```

### Terminal 3 — PX4 SITL

From your PX4-Autopilot checkout:

```bash
make px4_sitl none_iris
```

### Terminal 4 — Confirm

```bash
python px4_confirm.py
```

Full-loop takeoff confirmation:

```bash
python px4_confirm.py --takeoff 3.0
```

## Optional live viewer

From the repo root:

```bash
python headless-tools/live_viewer.py --port 14660 --range 24 --zmin 0 --zmax 25
```

## Optional offboard figure-eight

From this folder:

```bash
python test_offboard_figure8_3d.py --speed 10.0 --viz-udp 127.0.0.1:14660
```

This script:

- Connects to PX4 on the onboard/offboard MAVLink UDP link.
- Reads `LOCAL_POSITION_NED` and `ATTITUDE`.
- Commands local-position setpoints and yaw.
- Logs actual vs. target tracking.
- Saves CSV and plot files.
- Streams optional live samples to `headless-tools/live_viewer.py`.

## Ports

| Link | Default |
|---|---:|
| QuadSim RPC | `5555` |
| QuadSim telemetry | `5556` |
| PX4 simulator/HIL TCP | `4560` |
| PX4 onboard/offboard UDP | `14540` |
| Headless viewer UDP | `14660` |

## Bridge options

```bash
python px4_bridge.py --help
```

Common options:

```bash
python px4_bridge.py --speed 0
python px4_bridge.py --speed 1.0
python px4_bridge.py --px4-client
python px4_bridge.py --no-gps
python px4_bridge.py --hz 250
```

Use `--no-gps` only for attitude-only bring-up. Do not expect normal takeoff or position control without GPS/local position.

## Validation flow

Run this sequence after changing sensors, frames, motor mapping, or PX4 versions:

1. Start QuadSim.
2. Start bridge.
3. Start PX4.
4. Run `px4_confirm.py`.
5. Confirm heartbeat.
6. Confirm sane attitude.
7. Confirm arming.
8. Watch bridge for nonzero actuators.
9. Run `px4_confirm.py --takeoff 3.0`.
10. Run offboard figure-eight only after takeoff works.

## Common failure hints

### PX4 waits for simulator

Run bridge as client:

```bash
python px4_bridge.py --px4-client
```

### No attitude in `px4_confirm.py`

PX4 is not receiving/fusing simulated sensors. Check the bridge and PX4 TCP connection.

### Arming fails

Usually PX4 prearm/EKF/home readiness. Run with GPS enabled and wait for EKF to settle.

### Motors go nonzero but drone flips

Likely motor order mismatch. Check `MOTOR_MAP` in `px4_bridge.py`.

### QGroundControl datalink failsafe at high speed

Run real time:

```bash
python px4_bridge.py --speed 1.0
```

or adjust PX4 datalink-loss parameters for headless testing.

## More docs

See:

```text
Documentation~/px4-sitl.md
Documentation~/networking.md
Documentation~/troubleshooting.md
Documentation~/headless-tools.md
```
