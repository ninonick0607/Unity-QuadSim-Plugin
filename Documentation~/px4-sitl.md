# PX4 SITL Integration

This guide explains how to run **PX4 SITL** against QuadSim using the external Python bridge.

QuadSim acts as the simulated plant. PX4 owns the flight controller. The bridge is the lockstep conductor between the two systems:

```text
QuadSim Unity Runtime  <-- ZMQ/MessagePack RPC -->  px4_bridge.py  <-- MAVLink TCP 4560 --> PX4 SITL
                                                              |
                                                              +-- MAVLink UDP 14540 --> offboard / confirm scripts / QGroundControl
```

## What this mode is for

Use PX4 SITL mode when you want to validate QuadSim as a realistic external simulator for the PX4 autopilot stack.

This path is useful for:

- PX4 EKF bring-up using QuadSim IMU/GPS/barometer/magnetometer data.
- Validating motor passthrough and actuator feedback.
- Testing PX4 modes such as arming, takeoff, position control, and offboard control.
- Running hardware-deployment-style experiments where the controller consumes PX4 state rather than QuadSim ground truth.
- Future controller integrations where PX4 remains the inner flight controller and external logic sends setpoints.

## What this mode is not

PX4 SITL mode is not the same as QuadSim's Python geometric-controller lockstep path or the built in PID cascade.

In the Python geometric-controller path:

```text
Python controller -> QuadSim command -> QuadSim physics step -> sensors
```

In the PX4 SITL path:

```text
QuadSim sensors -> PX4 EKF/control -> PX4 actuator outputs -> QuadSim motors -> QuadSim physics step
```

PX4 owns the control loop. QuadSim is the plant.

## Files

The PX4-specific scripts live in the PX4 tools sample:

```text
Samples~/PX4 SITL Tools/
  README.md
  requirements.txt
  px4_bridge.py
  px4_confirm.py
  test_offboard_figure8_3d.py
  PX4_VERSION.md
```

Generic headless visualization tools live outside PX4:

```text
headless-tools/
  README.md
  live_viewer.py
```

The live viewer is intentionally not PX4-specific. It can be reused by PX4 experiments, geometric-controller experiments, Optuna studies, and future headless tooling.

## Dependencies

Create a Python environment for the PX4 tools:

```bash
python -m venv .venv
source .venv/bin/activate
pip install pymavlink numpy matplotlib quadsim-sdk
```

If `quadsim-sdk` is not installed from PyPI yet, install your local SDK:

```bash
pip install -e /path/to/QuadSimLib
```

PX4 tools require:

- `pymavlink` for MAVLink communication with PX4.
- `quadsim-sdk` for ZMQ/MessagePack RPC into QuadSim.
- `numpy` and `matplotlib` for validation scripts and plots.

## Required QuadSim scene setup

The Unity scene must have:

- Drone in FRD frame, you can change this when selecting the drone prefab
- Enable noise in QuadSimEnvironment config 
- A `SimRoot` with the RPC adapter enabled.
- A drone prefab with `QuadPawn`, sensors, and passthrough motor control available.
- IMU, GPS, barometer, and magnetometer outputs enabled.
- Geodetic GPS fields populated by the Unity-side GPS sensor.
- A deterministic physics loop, usually 250 Hz.
- RPC port available, usually `5555`.
- Telemetry port available, usually `5556`.

For PX4 bring-up, the scene should start the drone near the ground and should not require UI interaction.

## Run order

Use this order for a clean PX4 SITL session.

### Terminal 1 — Start QuadSim

Run QuadSim either in the Unity Editor or as a built executable.

For a headless build:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556
```

### Terminal 2 — Start the bridge

From the PX4 tools folder:

```bash
python px4_bridge.py --speed 0
```

`--speed 0` means uncapped. The bridge runs as fast as the PX4/QuadSim handshake allows.

For real-time pacing, use:

```bash
python px4_bridge.py --speed 1.0
```

### Terminal 3 — Start PX4 SITL

From your PX4-Autopilot checkout:

```bash
make px4_sitl none_iris
```

If PX4 prints something like:

```text
Waiting for simulator to connect on TCP port 4560
```

rerun the bridge as a TCP client:

```bash
python px4_bridge.py --px4-client
```

### Terminal 4 — Confirm the loop

From the PX4 tools folder:

```bash
python px4_confirm.py
```

For full takeoff confirmation:

```bash
python px4_confirm.py --takeoff 3.0
```

### Optional — Run the live viewer

From repo root:

```bash
python headless-tools/live_viewer.py --port 14660 --range 24 --zmin 0 --zmax 25
```

### Optional — Run the offboard figure-eight validation

From the PX4 tools folder:

```bash
python test_offboard_figure8_3d.py --speed 10.0 --viz-udp 127.0.0.1:14660
```

The figure-eight script connects to PX4 on the onboard/offboard UDP link, sends PX4 local-position setpoints, logs position/yaw tracking, saves CSV/plots, and can stream live path data to the generic headless viewer.

## Bridge behavior

`px4_bridge.py` performs the lockstep handshake:

1. Read QuadSim sensor snapshot.
2. Send `HIL_SENSOR` and optionally `HIL_GPS` to PX4.
3. Wait for `HIL_ACTUATOR_CONTROLS`.
4. Convert PX4 actuator outputs to QuadSim motor order.
5. Apply motors in QuadSim passthrough mode.
6. Step QuadSim physics.
7. Use the returned sensors for the next PX4 tick.

The bridge does not implement a flight controller. It only forwards sensors and actuators.

## Speed and lockstep

Lockstep means PX4 and QuadSim wait for each other every tick. It does **not** necessarily mean real time.

The bridge supports:

```bash
python px4_bridge.py --speed 0      # uncapped, fastest possible
python px4_bridge.py --speed 1.0    # real-time cap
python px4_bridge.py --speed 5.0    # cap at 5x real time
```

The speed is a cap, not a guaranteed target. The actual speed is limited by:

- MAVLink round-trip time.
- QuadSim RPC round-trip time.
- PX4 EKF/control processing.
- Python scheduling overhead.
- Console print spam.
- QGroundControl or other attached consumers.

PX4 lockstep cannot batch IMU ticks because EKF2 expects the sensor stream tick-by-tick.

## Motor ordering

PX4 actuator output order may not match QuadSim's motor order.

The bridge currently maps PX4 controls to QuadSim motors with:

```python
MOTOR_MAP = [2, 0, 1, 3]
```

QuadSim expects:

```text
send_motors(fl, fr, bl, br)
```

If the vehicle flips instantly on arm, the first thing to suspect is motor ordering.

## GPS and EKF

For normal takeoff/offboard validation, run the bridge with GPS enabled.

Do **not** use `--no-gps` unless doing attitude-only bring-up.

If GPS fields are zero or missing, the bridge will warn that the snapshot has no geodetic GPS. PX4 may still produce attitude, but takeoff and position control will usually fail or refuse to arm.

## PX4 parameters

The helper scripts set several headless-friendly parameters automatically, including RC/datalink failsafe behavior. This is useful for headless or faster-than-real-time experiments where no real RC or GCS is attached.

Typical parameters include:

```text
NAV_RCL_ACT=0
NAV_DLL_ACT=0
COM_RC_IN_MODE=4
```

For QGroundControl sessions at faster-than-real-time speeds, PX4 can interpret GCS heartbeats as slow in simulation time. Either run at real time or adjust PX4 datalink-loss behavior.

## Recommended validation stages

Use this sequence when changing anything in the PX4 path:

1. QuadSim starts and RPC is reachable.
2. Bridge connects to QuadSim.
3. PX4 connects to bridge on TCP 4560.
4. `px4_confirm.py` receives heartbeat on UDP 14540.
5. `px4_confirm.py` receives sane `ATTITUDE`.
6. PX4 arms.
7. Bridge logs actuator outputs going nonzero.
8. Drone reacts in QuadSim.
9. Optional takeoff reaches target altitude.
10. Optional offboard figure-eight completes and saves plots.

Each stage isolates one part of the loop.
