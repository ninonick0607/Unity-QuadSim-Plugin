# Troubleshooting

This document lists common QuadSim, PX4 SITL, and headless-tool failures and how to debug them.

## Unity package import issues

### Package installs but sample is not visible

Check that sample folders are under:

```text
Samples~/
```

and that `package.json` has a `samples` entry if you want the sample to appear in Package Manager.

Example:

```json
{
  "samples": [
    {
      "displayName": "QuadSim Scene",
      "description": "Minimal QuadSim scene with SimRoot, drone prefab, and RPC enabled.",
      "path": "Samples~/QuadSim Scene"
    },
    {
      "displayName": "PX4 SITL Tools",
      "description": "Python helper scripts for PX4 SITL integration.",
      "path": "Samples~/PX4 SITL Tools"
    }
  ]
}
```

### Scripts are missing on prefabs

This usually means Unity lost script GUIDs.

Fixes:

- Move `.cs` files with their `.meta` files.
- Do not recreate script `.meta` files unless necessary.
- Avoid copying scripts without metas after prefabs already reference them.
- Reimport the package after moving files.

### Duplicate assembly errors

If the same DLL exists in both the package and `Assets/`, Unity may report duplicate assemblies.

Common cause:

```text
Assets/Packages/NetMQ...
Packages/com.nino.quadsim/Runtime/Plugins/ThirdParty/NetMQ.dll
```

Fix:

- Keep runtime DLLs in `Runtime/Plugins/ThirdParty/`.
- Move old NuGet extraction folders outside `Assets/`.
- Only include one copy of each DLL.

## QuadSim RPC issues

### Python cannot connect to QuadSim

Check that QuadSim is running with the RPC adapter enabled.

For headless:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556
```

Then try a minimal Python connection:

```python
from quadsim import QuadSim

with QuadSim() as sim:
    print(sim.get_status())
```

Likely causes:

- Wrong RPC host or port.
- Unity scene did not include `ExternalRpcAdapter`.
- QuadSim not running.
- Firewall or WSL2 networking issue.
- A stale process is holding the port.

### RPC works but stepping is slow

Likely causes:

- Excessive Unity `Debug.Log` calls at physics rate.
- Running in Editor instead of headless build.
- Visual rendering, UI graphs, or scene effects active.
- Python doing heavy plotting/logging inside the control loop.
- Too many separate RPC calls instead of composite `step_with_command`.

Use composite stepping wherever possible.

## PX4 bridge issues

### PX4 says "Waiting for simulator to connect"

PX4 is acting as the TCP server and is waiting for the bridge to connect.

Run:

```bash
python px4_bridge.py --px4-client
```

### Bridge waits forever for PX4 heartbeat

Likely causes:

- PX4 has not started.
- Wrong TCP server/client mode.
- Wrong `--px4-port`.
- Network boundary issue.

Try both modes:

```bash
python px4_bridge.py
python px4_bridge.py --px4-client
```

### Bridge prints no actuator output

PX4 may still be booting, EKF may not be ready, or the vehicle is not armed.

Run:

```bash
python px4_confirm.py
```

If Stage 1 heartbeat passes but Stage 2 attitude fails, PX4 is not fusing the simulated sensors.

If Stage 2 passes but arming fails, it is probably a PX4 prearm/EKF/home issue.

### `px4_confirm.py` receives no heartbeat

The command/confirmation script uses the PX4 onboard/offboard UDP link, usually:

```text
udpin:0.0.0.0:14540
```

Try:

```bash
python px4_confirm.py --url udpin:0.0.0.0:14540
```

Also check whether QGroundControl or another MAVLink router is changing the port topology.

### Stage 2 attitude is wild

If roll/pitch/yaw are unstable or nonsensical while the drone is level on the ground, suspect:

- IMU frame mismatch.
- Gyro sign mismatch.
- Accelerometer sign mismatch.
- Gravity/specific-force convention mismatch.
- Quaternion or body-frame conversion bug.

At rest, the bridge prints the IMU acceleration sanity check. If the sign is wrong, PX4 EKF can initialize upside down or diverge.

### Arming fails

Common causes:

- GPS/home not ready.
- EKF not initialized.
- Prearm checks failing.
- RC input/failsafe configuration.
- Running the bridge with `--no-gps` and then expecting takeoff/position modes.

Try:

```bash
python px4_confirm.py
```

Wait longer for EKF/GPS origin. Run the bridge without `--no-gps`.

### Motors go nonzero but the drone flips instantly

First suspect motor mapping.

The bridge maps PX4 actuator channels to QuadSim motor order. QuadSim expects:

```text
FL, FR, BL, BR
```

If the drone flips instantly, the mapping is probably wrong for the PX4 airframe/mixer being used.

Debug method:

1. Keep the vehicle low or restrained in sim.
2. Log motor outputs.
3. Apply small asymmetric commands.
4. Verify roll, pitch, yaw responses.
5. Update `MOTOR_MAP`.

### Drone climbs/falls incorrectly

Likely causes:

- Total thrust scale mismatch.
- Motor curve mismatch.
- Mass mismatch.
- PX4 airframe parameters do not match QuadSim drone.
- IMU acceleration sign/convention issue.
- Motors are mapped but not physically equivalent to the PX4 mixer assumptions.

Check:

- QuadSim drone mass.
- Rotor max thrust.
- PX4 airframe config.
- Hover throttle estimate.
- Per-motor thrust limits.

### Takeoff command is accepted but vehicle does not climb

Likely causes:

- Motors not being applied to QuadSim.
- Vehicle not armed despite command ack.
- PX4 outputs are zero.
- Motor map wrong.
- Thrust scale too low.
- EKF/local position not healthy.

Watch bridge console for actuator output. Watch QuadSim telemetry for motors.

## Offboard figure-eight issues

### OFFBOARD rejected

PX4 requires a stream of setpoints before switching to OFFBOARD.

The script pre-streams current position before arming and switching mode. If OFFBOARD is still rejected:

- Ensure PX4 is receiving setpoints on UDP 14540.
- Ensure EKF/local position is valid.
- Ensure the setpoint rate is high enough.
- Check PX4 commander messages.

### Figure-eight aborts due to position error

Likely causes:

- Trajectory is too aggressive.
- PX4 gains/airframe not tuned for the QuadSim model.
- Speed cap/wall-rate causing setpoint stream issues.
- Wind/disturbance active.
- Motor saturation.
- Wrong mass/thrust/inertia.

Start with smaller values:

```bash
python test_offboard_figure8_3d.py --speed 1.0 --x-amp 5 --y-amp 4 --altitude 5 --period 30
```

Then scale up.

### Yaw tracking is unstable

Try:

```bash
python test_offboard_figure8_3d.py --yaw-mode hold
```

or reduce yaw-rate command:

```bash
python test_offboard_figure8_3d.py --max-yaw-rate-deg 30
```

## Live viewer issues

### Viewer opens but no path appears

Check that the sender is actually sending UDP:

```bash
python headless-tools/live_viewer.py --port 14660 --debug
```

For PX4 figure-eight:

```bash
python test_offboard_figure8_3d.py --viz-udp 127.0.0.1:14660
```

For SDK experiments, use `UdpViz` or send compatible JSON packets.

### Viewer path looks mirrored or flipped

The generic viewer assumes FLU, z-up coordinates.

PX4 local position is NED. PX4-specific scripts should convert for human visualization when needed, or clearly label axes. Do not silently mix FLU and NED in the same packet stream.

### Viewer slows down experiment

The UDP sender should be fire-and-forget. If the experiment slows down:

- Reduce send rate.
- Reduce viewer draw rate.
- Disable plotting during high-speed sweeps.
- Log to CSV and plot after the run.

## Faster-than-real-time issues

### Speed is much lower than expected

Likely causes:

- Per-tick console printing.
- Running Unity Editor instead of headless build.
- QGroundControl attached.
- Too many Python plots/log writes.
- MAVLink round-trip bottleneck.
- RPC round-trip bottleneck.
- PX4 EKF load.

PX4 lockstep speed is limited by one PX4 exchange plus one QuadSim RPC/physics step per tick. It cannot batch like pure QuadSim stepping.

### PX4 datalink failsafe triggers when running fast

At faster-than-real-time speeds, PX4 may interpret wall-clock GCS heartbeats as too slow in simulation time.

Options:

- Run at `--speed 1.0` when using QGroundControl.
- Disable or adjust datalink-loss behavior for headless tests.
- Use the helper scripts' headless-friendly parameter settings.

## What to do when stuck

Use this isolation order:

1. Run QuadSim.
2. Confirm Python SDK can call `sim.get_status()`.
3. Start `px4_bridge.py`.
4. Start PX4.
5. Confirm bridge/PX4 heartbeat.
6. Run `px4_confirm.py`.
7. Verify attitude.
8. Verify arming.
9. Verify actuator outputs.
10. Verify takeoff.
11. Only then run offboard trajectories.

Do not debug offboard tracking until the basic loop is proven.
