<p align="center">
  <h1 align="center">QuadSim Unity Plugin</h1>
  <p align="center">
    Installable Unity quadrotor simulation package with Python SDK control.
    <br />
    Deterministic stepping, headless RPC, sensors, acceleration commands, control allocation, and sample scenes.
  </p>
</p>

<p align="center">
  <a href="#getting-started">Getting Started</a> •
  <a href="#quick-example">Quick Example</a> •
  <a href="#stepped-control-lockstep">Stepped Control</a> •
  <a href="#acceleration-control">Acceleration Control</a> •
  <a href="#wrench-control">Wrench Control</a> •
  <a href="#headless-visualization">Visualization</a> •
  <a href="#api-reference">API Reference</a> •
  <a href="#project-structure">Project Structure</a>
</p>

---

## What is QuadSim?

QuadSim is a research-oriented quadrotor simulation platform built in Unity, designed for controls research, autonomy development, headless tuning, and sim-to-real workflows. This repository is the installable Unity plugin/package. It contains the Unity runtime, prefabs, sample scene content, RPC adapter, sensors, and controller plumbing needed to run QuadSim inside another Unity project.

The Python SDK is installed separately and connects to a running QuadSim Unity scene over ZeroMQ so you can command drones from Python.

**Two layers, one object:**

```text
QuadSim       ← sim/world entry point, owns the connection
  └─ Drone    ← everything about the drone (low + high level)
```

You get `takeoff()` and `send_command()` on the same `Drone` object. No juggling between classes to switch from scripted flight to raw control to deterministic stepping.

---

## Getting Started

### Requirements

- Unity 6 / Unity 6000.x
- Git installed on the machine running Unity
- Git LFS installed if your project pulls large mesh/texture assets through Git
- Python 3.8+ if you want to control QuadSim from the Python SDK
- A QuadSim scene with `SimRoot`, `ExternalRpcAdapter`, and at least one `QuadPawn` if you want RPC control

### Install the Unity Package

This repo is now distributed as a Unity Package Manager package. You do **not** need to clone the full Unity development project into your own project.

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

The imported sample contains the demo environment and scene assets. The reusable QuadSim runtime, prefabs, models, scripts, and third-party plugins remain inside the package under `Runtime/`.

### Headless Build

To run QuadSim headless, import the sample scene or create a scene that contains:

- `SimRoot`
- `SimulationManager`
- `SimCoreApi`
- `ControlAuthorityManager`
- `ExternalRpcAdapter`
- `DroneManager`
- at least one configured `QuadPawn`

Build that scene for Linux, then run:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556
```

### Install the Python SDK

The Unity package provides the simulator runtime. The Python SDK is installed separately:

```bash
pip install quadsim-sdk
```

From source:

```bash
git clone https://github.com/ninonick0607/QuadSimLib.git
cd QuadSimLib
pip install -e .
```

This installs `pyzmq` and `msgpack` automatically.

### Verify Connection

Start the Unity scene or headless build first, then run:

```python
from quadsim import QuadSim

sim = QuadSim()
sim.connect()
print(sim.get_status())
sim.disconnect()
```

If it prints status info, you're connected.

---

## Quick Example

```python
from quadsim import QuadSim

with QuadSim() as sim:
    drone = sim.drone()

    drone.takeoff(altitude=3.0)
    drone.fly_to(x=5, y=0, z=3)
    drone.hover(duration=2.0)
    drone.yaw_to(heading_deg=180)
    drone.fly_path([(5, 5, 3), (0, 5, 3), (0, 0, 3)])
    drone.land()
```

The high-level methods run a Python-side control loop at wall-clock 50 Hz. That's perfect for scripted missions — but for controls research you usually want **deterministic, faster-than-real-time stepping**, which is what the next section is about.

---

## Stepped Control (Lockstep)

The high-level loops above advance the sim in real time (Python sleeps between ticks). For tuning, learning, and reproducible experiments you instead take control of *time itself*: pause the sim, then advance physics one control tick at a time and read the result back.

The SDK exposes this through **composite step commands** that do three things in a single RPC round trip:

1. apply your command,
2. step physics `count` sub-steps,
3. return the post-step sensor snapshot.

```python
from quadsim import QuadSim

with QuadSim() as sim:
    drone = sim.drone()
    sim.pause()                       # you now own the clock

    sensors = drone.get_sensors()
    for _ in range(2500):             # 50 s @ 50 Hz — runs as fast as the host allows
        tx, ty, tz, thrust = my_controller(sensors)        # your control law
        sensors = drone.step_with_wrench(tx, ty, tz, thrust, count=5)

    sim.resume()
```

`count` is the number of physics sub-steps per control tick. With physics at 250 Hz and control at 50 Hz, `count=5` gives one 50 Hz control update per call.

### Why use it

- **Determinism.** Physics only advances when you call `step_*`. Identical inputs and seed produce bit-identical runs — exactly what you want for gain tuning, regression tests, and reproducing a bug.
- **Speed.** Nothing is tied to the wall clock, so headless runs go *faster than real time*. A 150 s trajectory can complete in a few seconds.
- **Fewer round trips.** The composite call is one REQ/REP instead of three (`send_command` + `step` + `get_sensor_data`). Over thousands of ticks per trial that adds up — it's the difference that makes Optuna sweeps practical.

### Reading controller internals

The step returns measured state (sensors). Controller outputs — including the allocator's per-motor forces — live in telemetry, fetched separately to keep measured state and controller state cleanly split:

```python
sensors = drone.step_with_wrench(tx, ty, tz, thrust, count=5)
tel = drone.get_telemetry()
print(tel.motor_thrusts)              # (FL, FR, BL, BR) per-motor force, Newtons
print(tel.motors)                     # same four, normalized [0, 1]
```

---

## Acceleration Control

Acceleration control is a higher-level command interface for outer-loop controllers, policies, planners, and guidance laws. Instead of commanding body torques or motor outputs directly, you command a desired **body/local-frame acceleration** and yaw rate:

```text
(ax, ay, az, yaw_rate)
```

All acceleration commands use the SDK's normal FLU convention:

| Field | Meaning | Units |
|------|---------|-------|
| `ax` | forward acceleration | m/s² |
| `ay` | left acceleration | m/s² |
| `az` | up acceleration | m/s² |
| `yaw_rate` | yaw rate command | deg/s |

The Unity cascade treats acceleration as an intermediate setpoint:

```text
acceleration command → desired roll/pitch + thrust → rate controller → motors
```

So acceleration mode is physically different from wrench mode. It still uses the onboard cascade and motor path, but bypasses the position and velocity PID stages. This is useful when your Python code owns the outer loop but you still want Unity's attitude/rate/motor stack to handle stabilization and actuation.

### One-shot acceleration command

```python
drone.send_acceleration(
    ax=1.0,        # accelerate forward
    ay=0.0,
    az=0.0,
    yaw_rate=0.0,
)
```

### Lockstep acceleration command

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
        ax, ay, az, yaw_rate = my_outer_loop(sensors)
        sensors = drone.step_with_acceleration(
            ax=ax,
            ay=ay,
            az=az,
            yaw_rate=yaw_rate,
            count=steps_per_tick,
        )
```

### Examples

```python
# Slight upward acceleration: above-hover thrust, no intentional tilt.
sensors = drone.step_with_acceleration(ax=0.0, ay=0.0, az=1.0, yaw_rate=0.0, count=5)

# Forward acceleration: the cascade should pitch/tilt the vehicle forward.
sensors = drone.step_with_acceleration(ax=1.0, ay=0.0, az=0.0, yaw_rate=0.0, count=5)

# Left acceleration while yawing.
sensors = drone.step_with_acceleration(ax=0.0, ay=1.0, az=0.0, yaw_rate=20.0, count=5)
```

> Acceleration commands are local/body-frame FLU commands. The C# RPC adapter converts them into Unity's internal frame and `Axis4` layout. Normal SDK users should send FLU values and should not pre-transform them.

---

## Wrench Control

A *wrench* is a body force + torque command: `(Mx, My, Mz, thrust)` in FLU — three body torques (Nm) and a thrust along body-up (N). There are two paths, and they answer different research questions.

### `wrench` — through the allocator (default)

The wrench is routed through the effectiveness-matrix **control allocator**: it's projected onto the real per-motor `[0..1]` feasible set, run through the actuation model, then applied. This is the physically faithful path — it respects motor limits and exposes saturation. Use it when you want the allocator in the loop (the usual case for flight-controller research).

```python
drone.send_wrench(tx=0.0, ty=0.1, tz=0.0, thrust=7.2)        # one-shot
sensors = drone.step_with_wrench(0.0, 0.1, 0.0, 7.2, count=5)  # apply + step + read
```

### `wrench_bypass` — direct to the rigid body

The force and torque are applied **straight to the Unity Rigidbody**, bypassing the allocator and actuation model entirely. No motor limits, no mixing — the exact wrench you ask for. Use it for idealized studies, sanity checks, or isolating controller behavior from actuation dynamics.

```python
drone.send_wrench_bypass(tx=0.0, ty=0.1, tz=0.0, thrust=7.2)
sensors = drone.step_with_wrench_bypass(0.0, 0.1, 0.0, 7.2, count=5)
```

> For hover, `thrust ≈ mass × g`. Torques are FLU body-frame; the C# adapter handles the FLU→Unity conversion.

### Raw motors

To skip wrench entirely and command the four rotors directly (throttle fractions `[0,1]`):

```python
drone.send_motors(fl=0.5, fr=0.5, bl=0.5, br=0.5)   # passthrough mode
```

---

## Mixing High-Level and Low-Level

```python
import time

with QuadSim() as sim:
    drone = sim.drone()

    drone.takeoff(3.0)                       # scripted

    drone.set_mode("velocity")               # raw velocity control
    for _ in range(100):
        drone.send_command(vx=1.0, vy=0, vz=0, yaw_rate=10)
        time.sleep(0.02)

    drone.hover(duration=2.0)                 # back to high-level
    drone.land()
```

## Async Flight with Polling

```python
import time

with QuadSim() as sim:
    drone = sim.drone()
    drone.takeoff(3.0)

    future = drone.fly_to_async(x=10, y=0, z=3, speed=2.0)
    while not future.done:
        pos = drone.get_position()
        print(f"Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        time.sleep(0.5)

    drone.land()
```

---

## Environment / Disturbance

Toggle the scene's wind/disturbance field from Python — useful for adaptive-control studies where you train or evaluate under a disturbance:

```python
drone.set_wind(enabled=True)                 # turn the WindModule field on
drone.set_wind(enabled=True, wind_speed=8.0) # ... and override its speed (m/s)
drone.set_wind(enabled=False)                # nominal plant
```

With no `wind_speed`, each `WindModule` keeps its scene-configured speed. Wind is plant configuration — it needs a connected client but not drone authority.

---

## Headless Visualization

When you run headless (sim stepped faster than real time, no Unity window in view) you can still watch the flight live. The SDK ships a fire-and-forget UDP sender, `UdpViz`, and a standalone viewer, `live_viewer.py`.

**In your script** — stream live state:

```python
from quadsim import UdpViz

viz = UdpViz(source="my-run")
viz.path(planned_points)                     # optional: static ghost of the planned path

# inside the control loop:
viz.sample(t,
           pos,                              # FLU [x, y, z]
           target=target_pos,               # optional setpoint
           err=err, speed=speed, trial=trial_id)
```

**In a second terminal** — run the viewer:

```bash
python live_viewer.py --port 14660
```

It draws the flown path, the setpoint trail, current markers, and the planned-path ghost. Everything is **FLU, z up** — positions are sent and plotted as-is, no frame flips. If no viewer is listening, packets are silently dropped and your run is unaffected.

> `viz.sample(...)` only has the data the *script* holds (actual + target), so it's an explicit call in your loop rather than something scraped from the command stream. The viewer accumulates the path on its own from the sample stream; the one-shot `viz.path(...)` is the only thing it can't infer.

For the in-Unity visualizer (when you *are* watching the scene), `drone.send_viz(pos, vel, bz)` pushes a desired-pose ghost over the RPC link instead.

---

## Coordinate Frame

Everything the Python SDK sees is **FLU** (Forward-Left-Up):

| Axis | Direction | Example |
|------|-----------|---------|
| **X** | Forward | `fly_to(x=5, ...)` moves forward |
| **Y** | Left | `fly_to(y=3, ...)` moves left |
| **Z** | Up | `fly_to(z=3, ...)` = 3 meters altitude |

**Z is always altitude.** This applies to positions, velocities, accelerations, and all sensor readings. The C# adapter handles all conversions to/from Unity's internal frame automatically.

Acceleration, velocity, and wrench commands sent from the SDK are expressed in the selected client/output frame. For normal Python SDK usage, that frame is FLU, so users should send commands directly in FLU and should not pre-transform them.

---

## API Overview

### QuadSim — Sim / World

| Method | Description |
|--------|-------------|
| `connect()` | Connect to Unity server |
| `disconnect()` | Clean disconnect |
| `drone()` | Get a Drone handle |
| `get_status()` | Sim time, pause state, authority |
| `pause()` / `resume()` | Pause/resume simulation |
| `step(count)` | Advance N physics steps while paused |
| `set_time_scale(scale)` | Speed up / slow down sim |
| `reset()` | Reset entire simulation |

### Drone — Flight Control

**High-level (blocking):**

| Method | Description |
|--------|-------------|
| `takeoff(altitude, speed)` | Climb and stabilize |
| `land(speed)` | Descend to ground |
| `hover(duration)` | Hold position for N seconds |
| `fly_to(x, y, z, speed, yaw)` | Fly to FLU position |
| `fly_path(waypoints, speed)` | Follow waypoint sequence |
| `yaw_to(heading_deg)` | Rotate to heading |

All have `_async` variants that return a `Future`.

**Low-level commands:**

| Method | Description |
|--------|-------------|
| `set_mode(mode)` | Set goal mode (rate, angle, velocity, acceleration, position, wrench, …) |
| `set_controller(ctrl)` | Switch controller (cascade, geometric) |
| `send_command(...)` | Send raw Axis4 command with named args |
| `send_acceleration(ax, ay, az, yaw_rate)` | Body/local FLU acceleration + yaw-rate command |
| `send_motors(fl, fr, bl, br)` | Raw rotor throttles `[0,1]` (passthrough) |
| `send_wrench(tx, ty, tz, thrust)` | Body wrench through the allocator |
| `send_wrench_bypass(tx, ty, tz, thrust)` | Body wrench straight to the Rigidbody |
| `set_wind(enabled, wind_speed)` | Toggle the disturbance field |

**Stepped (lockstep) — apply + step + read in one round trip:**

| Method | Returns | Description |
|--------|---------|-------------|
| `step_with_acceleration(ax, ay, az, yaw_rate, count)` | `SensorData` | Acceleration command, step `count`, read back |
| `step_with_wrench(tx, ty, tz, thrust, count)` | `SensorData` | Allocated wrench, step `count`, read back |
| `step_with_wrench_bypass(tx, ty, tz, thrust, count)` | `SensorData` | Direct wrench, step `count`, read back |

**Sensors & telemetry:**

| Method | Description |
|--------|-------------|
| `get_sensors()` | Full sensor snapshot (IMU + GPS) |
| `get_telemetry()` | Controller state, motor outputs, per-motor forces |
| `get_position()` / `get_attitude()` / `get_velocity()` | Quick reads |
| `is_airborne()` | Altitude check |

**Resets:**

| Method | Description |
|--------|-------------|
| `reset()` | Full drone reset |
| `reset_pose(x, y, z, ...)` | Teleport to a pose |
| `reset_rotation()` | Level the drone |
| `reset_physics()` | Zero velocities |
| `reset_controller()` | Clear PID integrators |

**Streaming (PUB/SUB):**

| Method | Description |
|--------|-------------|
| `subscribe_sensors(callback, hz)` | Push-based sensor data |
| `subscribe_telemetry(callback, hz)` | Push-based telemetry |
| `subscribe(callback, topics, hz)` | Raw topic subscription |
| `unsubscribe()` | Stop streaming |

**Visualization:**

| Method | Description |
|--------|-------------|
| `send_viz(pos, vel, bz)` | Push a desired-pose ghost to the in-Unity visualizer |

---

## API Reference

### send_command Layout

What `x`, `y`, `z`, `w` mean depends on the active mode:

| Mode | x | y | z | w |
|------|---|---|---|---|
| **Rate** | roll rate °/s | pitch rate °/s | yaw rate °/s | throttle [0,1] |
| **Angle** | roll ° | pitch ° | yaw rate °/s | throttle [0,1] |
| **Velocity** | vx m/s | vy m/s | vz m/s (up+) | yaw rate °/s |
| **Acceleration** | ax m/s² | ay m/s² | az m/s² (up+) | yaw rate °/s |
| **Position** | x m | y m | z m (altitude) | yaw ° |
| **Wrench** | Mx Nm | My Nm | Mz Nm | thrust N |
| **WrenchBypass** | Mx Nm | My Nm | Mz Nm | thrust N |

Named aliases map directly: `roll`→x, `pitch`→y, `yaw`/`yaw_rate`→z/w, `vx`→x, `vy`→y, `vz`→z, `ax`→x, `ay`→y, `az`→z, `throttle`→w. For acceleration, prefer `send_acceleration` / `step_with_acceleration`. For wrench, prefer the dedicated `send_wrench` / `send_wrench_bypass` and their `step_with_*` forms over `send_command(..., mode="wrench")`.

### Acceleration mode

| Mode string | Path | Use when |
|-------------|------|----------|
| `"acceleration"` | body/local acceleration setpoint → attitude/thrust cascade → motors | Your Python controller owns the outer-loop acceleration command |
| `"accel"` | alias for `"acceleration"` | Short-form mode string |

Acceleration commands are FLU by default: `+x` forward, `+y` left, `+z` up. The command is converted by the C# adapter before entering the Unity controller, so Python users should not convert it themselves.

### Wrench modes

| Mode string | Path | Use when |
|-------------|------|----------|
| `"wrench"` | Effectiveness-matrix allocator → per-motor `[0,1]` → actuation model | You want a physically faithful plant (motor limits, saturation) |
| `"wrench_bypass"` | Force/torque applied directly to the Rigidbody | You want the exact commanded wrench, no actuation dynamics |

### SensorData Fields

```python
sensors = drone.get_sensors()
sensors.gps_position        # (x, y, z) FLU, Z = altitude
sensors.imu_attitude        # (roll, pitch, yaw) degrees
sensors.imu_vel             # (vx, vy, vz) m/s
sensors.imu_accel           # (ax, ay, az) m/s²
sensors.imu_ang_vel         # (wx, wy, wz) rad/s
sensors.imu_orientation     # (qx, qy, qz, qw) quaternion
sensors.imu_valid           # bool
sensors.gps_valid           # bool
```

### Telemetry Fields

```python
telem = drone.get_telemetry()
telem.drone_id              # str
telem.mode                  # "rate", "angle", "velocity", "acceleration", "position", "wrench", ...
telem.controller            # "cascade", "geometric"
telem.motors                # (FL, FR, BL, BR) in [0, 1]
telem.motor_thrusts         # (FL, FR, BL, BR) per-motor force in Newtons
telem.desired_rates_deg     # (roll, pitch, yaw) °/s
telem.desired_angles_deg    # (roll, pitch, yaw) °
telem.desired_vel           # (vx, vy, vz) m/s
telem.external_cmd          # (x, y, z, w) raw Axis4
```

`motor_thrusts` is the allocator's per-motor output in Newtons — `motors × max_thrust`. Compare it against the rotor's max thrust to read saturation directly.

### SimStatus Fields

```python
status = sim.get_status()
status.is_paused            # bool
status.time_scale           # float
status.sim_time             # float (seconds)
status.fixed_dt             # float — physics step size; control count = (1/control_hz)/fixed_dt
status.authority            # "UI", "Internal", "External"
status.client_connected     # bool
```

### Visualization — UdpViz

```python
from quadsim import UdpViz

viz = UdpViz(addr=("127.0.0.1", 14660), enabled=True, source="my-run")
viz.path(points)                                   # one-shot planned-path ghost
viz.sample(t, pos, target=None, *, err=0.0, speed=0.0, trial=-1, **extra)
viz.close()
```

Positions are FLU `[x, y, z]`, z up. Extra keyword args (e.g. `phi_norm=...`) ride along in the packet for specialized viewers; `live_viewer.py` ignores unknown keys.

### Tuning Parameters

Set on the `Drone` object before or during flight:

```python
drone.position_tolerance = 0.5          # meters — arrival threshold
drone.altitude_tolerance = 0.3          # meters — takeoff/land threshold
drone.landing_altitude = 0.15           # below this = landed
drone.default_speed = 2.0               # m/s fallback
drone.control_loop_hz = 50.0            # SDK tick rate
drone.default_leg_timeout = 30.0        # seconds per waypoint
drone.use_velocity_mode_navigation = False  # velocity-mode fly_to fallback
```

### Exceptions

```python
from quadsim import QuadSimError, ConnectionError, CommandError, TimeoutError, ProtocolError
```

| Exception | When |
|-----------|------|
| `ConnectionError` | Not connected, connection denied |
| `CommandError` | Authority rejected, no drone, invalid mode |
| `TimeoutError` | RPC timeout, flight command timeout |
| `ProtocolError` | Wire-level serialization issues |

All inherit from `QuadSimError`.

---

## Project Structure

```text
QuadSimLib/
├── quadsim/                    # SDK package
│   ├── __init__.py             # Public exports: QuadSim, Drone, UdpViz, Future, types, exceptions
│   ├── sim.py                  # QuadSim class — sim/world entry point
│   ├── drone.py                # Drone class — all drone control (low + high level + stepped)
│   ├── viz.py                  # UdpViz — fire-and-forget UDP sender for headless runs
│   ├── _transport.py           # Internal ZMQ/MessagePack transport (not user-facing)
│   ├── _control_loops.py       # Internal control loop logic for high-level commands
│   ├── types.py                # SensorData, Telemetry, SimStatus dataclasses
│   ├── exceptions.py           # QuadSimError hierarchy
│   └── future.py               # Async Future handle
├── live_viewer.py              # Standalone live 3D viewer (run alongside any headless session)
├── Examples/
│   ├── flight_demo.py          # Full flight demo
│   └── async_demo.py           # Async/Future usage demo
├── Testing/                    # Low-level integration tests (Phase 9)
│   ├── quadsim_test_client.py
│   ├── quadsim_test_commands.py
│   └── test_high_level.py
└── pyproject.toml              # Package config — pip install -e .
```

---

## How It Works

The SDK communicates with QuadSim's Unity runtime over ZeroMQ:

- **REQ/REP** (port 5555) — commands, queries, mode changes, and composite step calls
- **PUB/SUB** (port 5556) — streaming telemetry

Messages are serialized with MessagePack. A background heartbeat thread keeps the connection alive. All of this is managed internally by `_transport.py` — you never touch sockets directly.

Two control styles share this one path:

- **High-level methods** (`takeoff`, `fly_to`, …) run Python-side control loops at wall-clock 50 Hz — inspectable and modifiable without recompiling Unity.
- **Stepped methods** (`step_with_acceleration`, `step_with_wrench`, …) pause the sim and advance physics deterministically, one composite round trip per control tick. This is the lockstep path used for tuning, learning, and reproducible experiments.

For live visibility during headless runs, `quadsim.viz.UdpViz` streams state over UDP to `live_viewer.py` — a separate process, so it never blocks or perturbs the sim loop.

---

## Related

- **Unity-QuadSim-Plugin** — Installable Unity package: [github.com/ninonick0607/Unity-QuadSim-Plugin](https://github.com/ninonick0607/Unity-QuadSim-Plugin)
- **QuadSimLib / quadsim-sdk** — Python SDK used to control the simulator from Python.

---

## License

MIT
