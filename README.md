<p align="center">
  <h1 align="center">QuadSim Unity Plugin</h1>
  <p align="center">
    Installable Unity quadrotor simulation package with Python SDK control.
    <br />
    Deterministic stepping, headless RPC, sensors, control allocation, and sample scenes.
  </p>
</p>

<p align="center">
  <a href="#getting-started">Getting Started</a> •
  <a href="#scene-setup">Scene Setup</a> •
  <a href="#python-sdk">Python SDK</a> •
  <a href="#control-overview">Control Overview</a> •
  <a href="#headless--lockstep">Headless / Lockstep</a> •
  <a href="#sensors--frames">Sensors & Frames</a> •
  <a href="#drone-configuration">Drone Configuration</a> •
  <a href="#architecture">Architecture</a>
</p>

---

## What is QuadSim?

QuadSim is a research-oriented quadrotor simulation platform built in Unity for controls research, autonomy development, headless tuning, and sim-to-real workflows.

This repository is the **Unity runtime/plugin**. It provides the simulation scene components, drone prefab, sensors, controller plumbing, control allocation, physics stepping, and RPC bridge needed to run QuadSim inside a Unity project.

The Python package is separate: **`quadsim-sdk`** connects to a running QuadSim scene over ZeroMQ and is where user-facing Python control APIs live.

```text
Unity QuadSim Plugin       ← simulator runtime, prefabs, physics, sensors, RPC
quadsim-sdk / QuadSimLib   ← Python interface, high-level flight, raw modes, lockstep control
```

Use this README to install and run the Unity simulator. Use the Python SDK docs for detailed control-mode usage.

- Python SDK repo: [github.com/ninonick0607/QuadSimLib](https://github.com/ninonick0607/QuadSimLib)
- Detailed Python control modes: [QuadSimLib/docs/control_modes.md](https://github.com/ninonick0607/QuadSimLib/blob/main/docs/control_modes.md)

---

## Key Features

- Installable Unity Package Manager package.
- Drop-in `SimRoot` + `QuadPawn` scene setup.
- Deterministic fixed-step physics using scripted simulation.
- Headless Linux execution with RPC ports configurable from the command line.
- External Python control over ZeroMQ / MessagePack.
- Composite lockstep stepping for faster-than-real-time studies.
- Explicit control authority between UI, Internal C#, and External Python.
- Cascaded controller modes: position, velocity, angle, and rate.
- Low-level access: direct motors, allocated wrench, and direct rigidbody wrench.
- Sensor stack: IMU, GPS, barometer, magnetometer.
- FLU output frame for Python-facing robotics workflows.
- YAML-based drone model and controller configuration.

---

## Getting Started

### Requirements

- Unity 6 / Unity 6000.x
- Git
- Git LFS if your project pulls large mesh or texture assets
- Python 3.8+ for Python SDK control
- A QuadSim scene containing `SimRoot`, `ExternalRpcAdapter`, and at least one configured `QuadPawn` for RPC control

### Install the Unity Package

In Unity:

1. Open **Window → Package Manager**.
2. Click **+**.
3. Choose **Add package from git URL...**.
4. Paste:

```text
https://github.com/ninonick0607/Unity-QuadSim-Plugin.git#v0.1.0
```

For active development, you can omit the tag and install the latest `main` branch:

```text
https://github.com/ninonick0607/Unity-QuadSim-Plugin.git
```

For reproducible research or lab machines, prefer a tagged version like `#v0.1.0`.

### Import the Sample Scene

After the package installs:

1. Open **Package Manager**.
2. Select **QuadSim**.
3. Open the **Samples** section.
4. Import **QuadSim Scene**.
5. Open the imported scene and press Play.

The sample scene contains the demo environment and required simulation objects. Runtime scripts, prefabs, models, and third-party plugins remain inside the package under `Runtime/`.

---

## Scene Setup

QuadSim is designed to drop into a Unity scene with a small number of required objects.

### `SimRoot`

`SimRoot` is the simulation backbone. It should contain or reference:

| Component | Purpose |
|----------|---------|
| `SimulationManager` | Owns scripted fixed-step physics, pause/resume, and stepping |
| `SimCoreApi` | Global simulation API facade |
| `ControlAuthorityManager` | Tracks whether UI, Internal, or External owns control |
| `ExternalRpcAdapter` | ZeroMQ server for Python clients |
| `DroneManager` | Spawns and registers configured drones |

### `QuadPawn`

`QuadPawn` is the drone vehicle. It owns the Rigidbody, motor transforms, controller, sensors, command proxy, and per-drone API.

A typical drone prefab includes:

| Component | Purpose |
|----------|---------|
| `Rigidbody` | Physical body simulated by Unity |
| `CascadedController` | Position → velocity → angle → rate controller |
| `SensorManager` | IMU, GPS, barometer, magnetometer |
| `FlightCommandProxy` | Current command and goal mode |
| `ModeCoordinator` | Mode transitions and routing |
| `QuadSimApi` | Per-drone API used by C# and RPC |

`DroneManager` can spawn a configured `QuadPawn` automatically. For most users, the sample scene is the fastest starting point.

---

## Python SDK

Install the SDK separately from the Unity package:

```bash
pip install quadsim-sdk
```

From source:

```bash
git clone https://github.com/ninonick0607/QuadSimLib.git
cd QuadSimLib
pip install -e .
```

Start the Unity scene or headless build first, then verify the connection:

```python
from quadsim import QuadSim

with QuadSim() as sim:
    print(sim.get_status())
```

Basic flight:

```python
from quadsim import QuadSim

with QuadSim() as sim:
    drone = sim.drone()

    drone.takeoff(altitude=3.0)
    drone.fly_to(x=5, y=0, z=3)
    drone.hover(duration=2.0)
    drone.yaw_to(heading_deg=180)
    drone.land()
```

For detailed mode-by-mode Python usage, see the SDK docs:

```text
QuadSimLib/docs/control_modes.md
```

---

## Control Overview

QuadSim separates **who owns control** from **what kind of command is being sent**.

### Control Sources

Only one source has authority at a time.

| Source | Used for |
|--------|----------|
| `UI` | Keyboard/gamepad/manual testing |
| `Internal` | C# scripts running inside Unity |
| `External` | Python SDK over ZeroMQ RPC |

The authority system prevents silent takeovers. External Python commands only apply when the Python client is connected and has authority.

### Control Interfaces

This README intentionally keeps the mode list short. The detailed Python command syntax lives in the SDK docs.

| Interface | What you command | Typical use |
|----------|------------------|-------------|
| High-level SDK flight | `takeoff`, `fly_to`, `hover`, `land` | Scripted demos, simple autonomy |
| Position mode | FLU position + yaw | Waypoints and navigation |
| Velocity mode | FLU velocity + yaw rate | Outer-loop guidance |
| Angle mode | Roll/pitch angle + yaw rate + throttle/altitude stage | Manual-style stabilized control |
| Rate mode | Body rates + throttle | Inner-loop controller work |
| Passthrough motors | Four normalized motor commands | PX4 bridge, mixer tests, actuator studies |
| Allocated wrench | Body torques + thrust through allocator | Geometric controller and physical feasibility tests |
| Direct wrench | Body force/torque applied to Rigidbody | Idealized controller sanity checks |
| Acceleration helper | Desired local acceleration + yaw rate through SDK helper | Policy/guidance interfaces that do not want to command torques |

Important distinction:

- **Allocated wrench** goes through motor allocation and saturation.
- **Direct wrench** bypasses the allocator and applies the requested force/torque directly to the Rigidbody.
- **Passthrough** bypasses the cascaded controller but still goes through the actuation model.

Detailed examples for switching modes from Python are in:

- [Python SDK control modes](https://github.com/ninonick0607/QuadSimLib/blob/main/docs/control_modes.md)

---

## Headless / Lockstep

QuadSim can run without a Unity window for tuning, regression tests, and faster-than-real-time studies.

Build your scene for Linux, then run:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556
```

The simulator uses scripted physics stepping, so Python can pause the sim and advance it deterministically:

```python
from quadsim import QuadSim

CONTROL_HZ = 50.0

with QuadSim() as sim:
    drone = sim.drone()
    sim.pause()

    status = sim.get_status()
    steps_per_tick = max(1, round((1.0 / CONTROL_HZ) / status.fixed_dt))

    sensors = drone.get_sensors()
    for _ in range(1000):
        command = my_controller(sensors)
        sensors = drone.step_with_command(command, count=steps_per_tick)

    sim.resume()
```

At the default 250 Hz physics rate, `count=5` gives a 50 Hz controller update.

Why lockstep matters:

- Physics advances only when requested.
- Trials are reproducible.
- Headless studies can run faster than real time.
- Composite step calls reduce RPC round trips.

---

## Sensors & Frames

Everything exposed to Python uses **FLU** unless a lower-level interface explicitly says otherwise:

| Axis | Direction |
|------|-----------|
| `X` | Forward |
| `Y` | Left |
| `Z` | Up |

Common fields:

| Field | Frame |
|-------|-------|
| `gps_position` | World FLU, z-up |
| `imu_orientation` | Body orientation exposed in SDK convention |
| `imu_attitude` | Roll, pitch, yaw |
| `imu_vel` | Body-frame FLU velocity |
| `imu_ang_vel` | Body-frame FLU angular velocity |
| `imu_accel` | Body-frame FLU acceleration/specific force depending on sensor setting |
| `gps_vel_ned` | NED velocity for PX4 bridge use |
| `baro_*` | Barometer pressure, temperature, altitude |
| `mag_*` | Magnetometer field and heading |

The frame conversion boundary belongs in the Unity adapter and sensor stack. Python users should send and read FLU values unless they are intentionally writing a PX4/NED bridge.

---

## Drone Configuration

Drone parameters are loaded from YAML/model configuration and applied to the spawned `QuadPawn`.

Typical configuration includes:

```yaml
name: "StandardDrone"

physical_properties:
  mass_kg: 0.73
  com_offset: { x: 0, y: 0, z: 0 }
  inertia: { x: 0.00791, y: 0.00791, z: 0.01231 }

propulsion:
  max_rpm: 10000
  thrust_coef: 0.025
  torque_coef: 0.0005
  propellers:
    - { id: 0, x:  0.17, y: 0, z:  0.17, dir:  1 }
    - { id: 1, x:  0.17, y: 0, z: -0.17, dir: -1 }
    - { id: 2, x: -0.17, y: 0, z:  0.17, dir: -1 }
    - { id: 3, x: -0.17, y: 0, z: -0.17, dir:  1 }

controller:
  position_gains:
    x: { p: 1.0, i: 0.0, d: 0.0 }
  velocity_gains:
    x: { p: 2.0, i: 0.1, d: 0.0 }
  angle_gains:
    roll: { p: 6.0, i: 0.0, d: 0.0 }
  rate_gains:
    roll: { p: 0.1, i: 0.0, d: 0.0 }
```

For packaged builds, runtime-readable configs should live in Unity-accessible package resources or `StreamingAssets`, depending on how the project loads drone models/configs.

---

## Visualization

### In Unity

The sample scene can visualize the drone, ground plane, telemetry HUD, and optional ghost/setpoint markers.

### Headless UDP Viewer

For headless runs, stream the state from Python to a separate UDP viewer:

```python
from quadsim import UdpViz

viz = UdpViz(source="study-0")
viz.path(planned_points)

# inside your control loop
viz.sample(t, pos, target=target_pos, err=err)
```

Run the viewer in a second terminal:

```bash
python live_viewer.py --port 14660
```

The viewer uses FLU `[x, y, z]` directly.

---

## Architecture

```text
Python SDK
  └── ZeroMQ / MessagePack RPC
        └── ExternalRpcAdapter
              └── SimCoreApi / QuadSimApi
                    ├── ControlAuthorityManager
                    ├── SimulationManager
                    └── QuadPawn
                          ├── SensorManager
                          ├── CascadedController
                          ├── ControlAllocator
                          └── Rigidbody
```

### Simulation Timing

`SimulationManager` sets Unity physics to scripted mode and advances the sim through a fixed-step pipeline:

```text
PrePhysicsStep → Physics.Simulate(dt) → PostPhysicsStep
```

The default base rate is 250 Hz. Controllers, sensors, telemetry, and external clients can run at subrates using deterministic tick timing.

### RPC

The external interface uses two sockets:

| Socket | Default port | Purpose |
|--------|--------------|---------|
| REQ/REP | `5555` | Commands, queries, mode changes, stepping |
| PUB/SUB | `5556` | Telemetry streaming |

The headless executable can override both ports:

```bash
-rpcPort 5555 -telemetryPort 5556
```

---

## Project Structure

Package-oriented layout:

```text
Unity-QuadSim-Plugin/
├── package.json
├── Runtime/
│   ├── DroneCore/
│   ├── RobotCore/
│   ├── SimCore/
│   ├── Prefabs/
│   ├── Configs/
│   └── ThirdParty/
├── Samples~/
│   └── QuadSim Scene/
├── Documentation~
└── README.md
```

Development repository layout may contain additional editor-only scenes, experiments, assets, and test scripts.

---

## Related

- **Python SDK / QuadSimLib**: [github.com/ninonick0607/QuadSimLib](https://github.com/ninonick0607/QuadSimLib)
- **Unity plugin**: [github.com/ninonick0607/Unity-QuadSim-Plugin](https://github.com/ninonick0607/Unity-QuadSim-Plugin)
- **Research controller repo**: `Resnet-GeometricController`

---

## License

MIT
