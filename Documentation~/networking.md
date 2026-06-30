# Networking and Ports

QuadSim's external-control workflows use several local ports. This document explains what each port does, how the pieces connect, and how to debug connection problems.

## Default ports

| System | Port | Protocol | Direction | Purpose |
|---|---:|---|---|---|
| QuadSim RPC | `5555` | ZMQ REQ/REP | Python client ↔ Unity | Commands, stepping, resets, sensor queries |
| QuadSim telemetry | `5556` | ZMQ PUB/SUB | Unity → clients | Streaming sensors/telemetry |
| PX4 simulator/HIL | `4560` | MAVLink TCP | bridge ↔ PX4 | HIL sensor input and actuator output |
| PX4 onboard/offboard | `14540` | MAVLink UDP | scripts/QGC ↔ PX4 | Command, confirm, offboard control |
| Headless live viewer | `14660` | UDP JSON | experiment script → viewer | Live 3D visualization |

## Normal PX4 topology

```text
QuadSim Unity
  RPC 5555  <------>  px4_bridge.py  <------>  PX4 SITL TCP 4560
  TLM 5556              |
                        +----------->  PX4 onboard UDP 14540
                                      px4_confirm.py
                                      offboard scripts
                                      QGroundControl
```

The bridge is the only process that should talk to PX4's simulator/HIL TCP link.

Command scripts such as `px4_confirm.py` and `test_offboard_figure8_3d.py` talk to the PX4 onboard/offboard UDP link.

## QuadSim port overrides

QuadSim supports command-line port overrides:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5555 -telemetryPort 5556
```

Use different ports when running multiple QuadSim instances:

```bash
./QuadSim_Unity.x86_64 -batchmode -nographics -rpcPort 5565 -telemetryPort 5566
```

Then point clients at the matching host/ports.

## PX4 bridge options

Default bridge:

```bash
python px4_bridge.py --quadsim-host localhost --px4-port 4560
```

If PX4 is waiting for the simulator to connect:

```bash
python px4_bridge.py --px4-client
```

If QuadSim is running on a different host:

```bash
python px4_bridge.py --quadsim-host 192.168.1.20
```

If PX4 uses a different simulator port:

```bash
python px4_bridge.py --px4-port 4561
```

## UDP live viewer

The generic headless viewer listens for UDP JSON:

```bash
python headless-tools/live_viewer.py --host 0.0.0.0 --port 14660
```

Experiment scripts send to:

```text
127.0.0.1:14660
```

or to another machine:

```text
192.168.1.50:14660
```

The viewer is fire-and-forget. If nothing is listening, the sender should not block the experiment.

## WSL2 notes

When using Windows + WSL2:

- Prefer WSL2 mirrored networking if available.
- Keep QuadSim, PX4, and Python tools in the same environment when possible.
- If Unity runs on Windows and scripts run in WSL2, verify host IPs explicitly.
- Firewalls can block UDP/TCP even when localhost appears correct.
- QGroundControl running on Windows may not see WSL2 UDP traffic unless networking is configured correctly.

Useful checks:

```bash
ss -ltnp | grep 4560
ss -lunp | grep 14540
ss -lunp | grep 14660
```

On Windows, use PowerShell:

```powershell
netstat -ano | findstr 4560
netstat -ano | findstr 14540
netstat -ano | findstr 14660
```

## Connection debugging checklist

### QuadSim RPC

Check that QuadSim is running and listening on the expected RPC port.

Symptoms:

```text
Connection refused
RPC timeout
No response from QuadSim
```

Likely causes:

- QuadSim is not running.
- Wrong `-rpcPort`.
- RPC adapter disabled.
- Firewall or WSL networking issue.
- Another process already bound to the port.

### PX4 simulator TCP 4560

Symptoms:

```text
PX4: Waiting for simulator to connect on TCP port 4560
```

Fix:

```bash
python px4_bridge.py --px4-client
```

Symptoms:

```text
bridge waits forever for PX4 heartbeat
```

Likely causes:

- PX4 not started.
- Wrong TCP server/client mode.
- Wrong PX4 port.
- Firewall/WSL network boundary.

### PX4 onboard UDP 14540

Symptoms:

```text
px4_confirm.py waits forever for heartbeat
```

Likely causes:

- PX4 is not publishing onboard UDP.
- Wrong `--url`.
- PX4 not fully started.
- Network namespace mismatch between Windows/WSL/Linux.

Default URL:

```bash
python px4_confirm.py --url udpin:0.0.0.0:14540
```

### Live viewer UDP 14660

Symptoms:

```text
viewer opens but path does not move
```

Likely causes:

- Experiment script not sending `--viz-udp`.
- Wrong host/port.
- Sender and viewer in different network namespaces.
- Viewer is listening on a different port.
- Script crashed before sending samples.

Run viewer with debug:

```bash
python headless-tools/live_viewer.py --port 14660 --debug
```

## Running multiple instances

For parallel tuning or multi-instance experiments, give every process its own ports.

Example instance 0:

```text
QuadSim RPC 5555
QuadSim TLM 5556
PX4 TCP 4560
PX4 UDP 14540
Viewer UDP 14660
```

Example instance 1:

```text
QuadSim RPC 5565
QuadSim TLM 5566
PX4 TCP 4570
PX4 UDP 14550
Viewer UDP 14670
```

Avoid port collisions. A single collision can look like a controller or simulator bug.
