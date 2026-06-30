#!/usr/bin/env python3
"""
px4_bridge.py - QuadSim <-> PX4 SITL lockstep MAVLink bridge.

When PX4 flies, QuadSim is a *pure plant*: PX4 owns control, the sim only
feeds simulated sensors in and applies motor commands out. This script is
the conductor of that loop.

Per tick:
  1. Pull the sensor snapshot from QuadSim (the sim is already in PX4 mode:
     body-frame specific-force accel, body gyro/mag, and geodetic GPS).
  2. Pack HIL_SENSOR (+ HIL_GPS) and send to PX4.
  3. Block for HIL_ACTUATOR_CONTROLS (the four motor outputs, 0..1).
  4. Map -> QuadSim passthrough motors, step physics one tick, read back.
     Those sensors become tick N+1.

PX4's clock is the time_usec we stamp, so this is the same lockstep
handshake QuadSim already uses for step_with_command.

The bridge does NO frame or geo conversion - the sim owns all of that.
It forwards whatever the snapshot carries straight into the MAVLink messages.

SPEED (faster than realtime)
  Lockstep and wall-clock speed are orthogonal. Lockstep just means PX4 and
  the sim wait on each other's messages; it says nothing about how fast that
  exchange runs in wall time. So we go faster than realtime by NOT throttling
  the handshake, and the speed achieved is simply dt / (wall time per tick).
    --speed 0   (default) : no cap, as fast as the handshake allows (max).
    --speed 1.0           : pace to realtime (use when QGC / a human watches).
    --speed N             : cap at N x realtime.
  --speed is a CAP, not a target: above what the machine can do, it never
  sleeps. PX4 lockstep cannot batch (EKF2 needs every IMU sample), so the
  ceiling is one MAVLink + one RPC round trip per tick -- typically 6-10x.

  Watch the periodic "[bridge] sim=.. wall=.. speed=..x" line to see the
  real factor. If speed < 1.0, your per-tick wall cost exceeds dt; profile
  the RPC / MAVLink round trip, not the Python here.

  Running >1x with QGroundControl attached: PX4 sees GCS heartbeats arrive
  "slowly" in sim time and may trip datalink-loss. Scale COM_DL_LOSS_T with
  the factor, or set NAV_DLL_ACT=0 for headless. (PX4 params, not this file.)

TOPOLOGY
  QuadSim (C#) <--RPC--> this bridge <--MAVLink TCP 4560--> PX4 SITL
  Commands/confirmation ride a SEPARATE link: PX4 onboard UDP 14540.

RUN ORDER
  1. QuadSim scene already in PX4 mode (frame + GPS geodetic set in-sim).
  2. python px4_bridge.py
  3. In PX4-Autopilot:  make px4_sitl none_iris
     (if PX4 logs "Waiting for simulator to connect on TCP port 4560",
      rerun us with --px4-client)
  4. python px4_confirm.py   (or attach QGroundControl)

DEPENDS:  pip install pymavlink   (plus the quadsim SDK on PATH)
"""

from __future__ import annotations

import argparse
import math
import time

from pymavlink import mavutil

from quadsim import QuadSim
from quadsim.types import SensorData


# PX4 iris motor outputs (HIL_ACTUATOR_CONTROLS.controls[0..3]) -> QuadSim
# send_motors(fl, fr, bl, br). #1 thing to verify: if the drone flips instantly
# on arm, this mapping is wrong (PX4 mixer order != fl/fr/bl/br).
MOTOR_MAP = [2, 0, 1, 3]
# fl<-c2, fr<-c0, bl<-c1, br<-c3
# HIL_SENSOR.fields_updated bits, one group at a time.
F_ACCEL = 0x0007
F_GYRO  = 0x0038
F_MAG   = 0x01C0
F_BARO  = 0x1A00


def step_motors(drone, motors4, count: int) -> SensorData:
    """Apply 4 passthrough motors, step `count` physics ticks, return sensors."""
    fl, fr, bl, br = motors4
    resp = drone._transport.request("step_with_command", {
        "x": fl, "y": fr, "z": bl, "w": br,
        "mode": "passthrough", "count": count,
    })
    return SensorData.from_dict(resp)


def send_hil_sensor(mav, t_us: int, s: SensorData, fields: int) -> None:
    ax, ay, az = s.imu_accel        # specific force (hover ~ 0,0,-9.81)
    gx, gy, gz = s.imu_ang_vel      # rad/s
    mx, my, mz = s.mag_field        # Gauss
    abs_p = s.baro_pressure_hpa
    p_alt = s.baro_pressure_altitude
    temp  = s.baro_temperature_c
    try:
        mav.mav.hil_sensor_send(
            t_us, ax, ay, az, gx, gy, gz, mx, my, mz,
            abs_p, 0.0, p_alt, temp, fields, 0)   # last arg = id (newer pymavlink)
    except TypeError:
        mav.mav.hil_sensor_send(
            t_us, ax, ay, az, gx, gy, gz, mx, my, mz,
            abs_p, 0.0, p_alt, temp, fields)


def send_hil_gps(mav, t_us: int, s: SensorData) -> None:
    """Pure forward of sim-computed geodetic + NED velocity. No math here."""
    vn, ve, vd = s.gps_vel_ned
    gspeed = math.hypot(vn, ve)
    cog = math.degrees(math.atan2(ve, vn)) % 360.0
    args = dict(
        time_usec=t_us, fix_type=3,
        lat=int(s.gps_lat * 1e7), lon=int(s.gps_lon * 1e7), alt=int(s.gps_alt * 1e3),
        eph=20, epv=20,
        vel=int(gspeed * 100), vn=int(vn * 100), ve=int(ve * 100), vd=int(vd * 100),
        cog=int(cog * 100), satellites_visible=14,
    )
    try:
        mav.mav.hil_gps_send(**args, id=0, yaw=0)
    except TypeError:
        mav.mav.hil_gps_send(**args)


def run(args) -> None:
    # --- PX4 link -----------------------------------------------------------
    if args.px4_client:
        conn = f"tcp:{args.px4_host}:{args.px4_port}"
        print(f"[bridge] connecting to PX4 at {conn}")
    else:
        conn = f"tcpin:0.0.0.0:{args.px4_port}"
        print(f"[bridge] listening for PX4 on {conn}")
    mav = mavutil.mavlink_connection(conn, source_system=1, source_component=1)
    if not args.px4_client:
        print("[bridge] waiting for PX4 to connect...")
    mav.wait_heartbeat(timeout=None)
    print(f"[bridge] PX4 link up (sys {mav.target_system})")

    # --- QuadSim link -------------------------------------------------------
    with QuadSim(host=args.quadsim_host) as sim:
        drone = sim.drone()
        status = sim.get_status()
        fixed_dt = status.fixed_dt
        steps = max(1, round((1.0 / args.hz) / fixed_dt))
        dt = steps * fixed_dt
        dt_us = int(round(dt * 1e6))
        realized_hz = 1.0 / dt
        print(f"[bridge] fixed_dt={fixed_dt} steps/tick={steps} -> {realized_hz:.0f} Hz HIL")
        # steps rounds to whole physics ticks; warn if that misses the request.
        if abs(realized_hz - args.hz) > 1.0:
            print(f"[bridge] NOTE requested {args.hz:.0f} Hz but fixed_dt={fixed_dt} only "
                  f"divides to {realized_hz:.0f} Hz. Pick an hz that divides "
                  f"{1.0/fixed_dt:.0f}.")

        # Speed cap: 0 (or <=0) => unlimited (run as fast as the handshake allows).
        speed_cap = args.speed if args.speed and args.speed > 0 else None
        print(f"[bridge] speed cap = {'unlimited' if speed_cap is None else f'{speed_cap:.2f}x'}")

        sim.pause()
        drone.send_motors(0, 0, 0, 0)
        drone.reset_pose(x=0.0, y=0.0, z=1.0, qw=1.0)   # on the ground at origin
        sim.step(count=10)

        sensors = drone.get_sensors()
        # Sanity: at rest the accel should read ~ (0, 0, -9.81). +9.81 => sign
        # inverted upstream and EKF2 will init upside-down. (Delete if you like.)
        print(f"[bridge] rest accel = {tuple(round(v, 2) for v in sensors.imu_accel)}")

        gps_ok = (not args.no_gps)
        if gps_ok and sensors.gps_lat == 0.0 and sensors.gps_lon == 0.0:
            print("[bridge] WARN snapshot has no geodetic GPS yet "
                  "(gps_lat/lon = 0). Add the C# GPS geodetic fields, or run "
                  "with --no-gps for attitude-only bring-up. Skipping HIL_GPS.")
            gps_ok = False

        last_mag_ts = last_baro_ts = -1.0
        t_us = dt_us
        armed_logged = False
        gps_div = max(1, round(args.hz / 10.0))   # HIL_GPS ~10 Hz
        tick = 0
        last_motors = (0.0, 0.0, 0.0, 0.0)

        # --- pacing + speed instrumentation state ---------------------------
        log_every = max(1, int(round(args.hz)))   # ~ one sim-second between logs
        wall_start = time.perf_counter()
        last_wall = wall_start
        sim_t0_us = t_us

        # --- per-tick timing breakdown --------------------------------------
        timing = {
            "send_hil": 0.0,
            "wait_px4": 0.0,
            "step_rpc": 0.0,
            "sleep": 0.0,
        }

        print("[bridge] entering lockstep (Ctrl+C to stop)")
        try:
            while True:
                fields = F_ACCEL | F_GYRO
                if sensors.mag_timestamp != last_mag_ts:
                    fields |= F_MAG
                    last_mag_ts = sensors.mag_timestamp
                if sensors.baro_timestamp != last_baro_ts:
                    fields |= F_BARO
                    last_baro_ts = sensors.baro_timestamp
                t0 = time.perf_counter()

                send_hil_sensor(mav, t_us, sensors, fields)
                if gps_ok and (tick % gps_div == 0):
                    send_hil_gps(mav, t_us, sensors)

                t1 = time.perf_counter()

                # Block for actuators. Timeout covers PX4's "freewheeling"
                # startup (sensors-only until EKF inits): hold motors at zero so
                # the drone rests on the ground while PX4 boots.
                msg = mav.recv_match(type="HIL_ACTUATOR_CONTROLS",
                                     blocking=True, timeout=0.5)

                t2 = time.perf_counter()

                timing["send_hil"] += t1 - t0
                timing["wait_px4"] += t2 - t1
                if msg is None:
                    if armed_logged:

                        motors = last_motors
                    else:
                        motors = (0.0, 0.0, 0.0, 0.0)   # boot freewheel: rest on ground
                else:
                    c = msg.controls
                    motors = tuple(max(0.0, min(1.0, c[MOTOR_MAP[i]])) for i in range(4))
                    if tick % 50 == 0:
                        fl, fr, bl, br = motors

                        # These are only diagnostic mix channels in QuadSim motor order.
                        collective = 0.25 * (fl + fr + bl + br)
                        roll_mix = (fl + bl) - (fr + br)
                        pitch_mix = (fl + fr) - (bl + br)
                        yaw_mix_a = (fl + br) - (fr + bl)
                        yaw_mix_b = (fl + fr) - (bl + br)

                    if not armed_logged and max(motors) > 0.05:
                        print(f"[bridge] actuators live: {tuple(round(m, 2) for m in motors)}")
                        armed_logged = True
                last_motors = motors

                sensors = step_motors(drone, motors, steps)

                t3 = time.perf_counter()
                timing["step_rpc"] += t3 - t2

                t_us += dt_us
                tick += 1

                # --- optional pacing: cap wall speed, lockstep untouched -----
                if speed_cap is not None:
                    sim_el = (t_us - sim_t0_us) / 1e6
                    slack = sim_el / speed_cap - (time.perf_counter() - wall_start)
                    if slack > 0:
                        ts0 = time.perf_counter()
                        time.sleep(slack)
                        timing["sleep"] += time.perf_counter() - ts0

                # --- speed readout -------------------------------------------
                if tick % log_every == 0:
                    now = time.perf_counter()
                    sim_el = (t_us - sim_t0_us) / 1e6
                    wall_el = now - wall_start
                    interval_wall = now - last_wall
                    eff_hz = log_every / interval_wall if interval_wall > 0 else 0.0
                    avg = sim_el / wall_el if wall_el > 0 else 0.0
                    inst = eff_hz * dt   # sim-seconds advanced per wall-second

                    for k in timing:
                        timing[k] = 0.0

                    last_wall = now

        except KeyboardInterrupt:
            print("\n[bridge] stopping, cutting motors")
            try:
                drone.send_motors(0, 0, 0, 0)
            except Exception:
                pass


def main():
    p = argparse.ArgumentParser(description="QuadSim <-> PX4 SITL HIL bridge")
    p.add_argument("--quadsim-host", default="localhost")
    p.add_argument("--px4-host", default="127.0.0.1")
    p.add_argument("--px4-port", type=int, default=4560)
    p.add_argument("--px4-client", action="store_true",
                   help="Connect TO PX4 (use if PX4 logs 'Waiting for simulator to connect')")
    p.add_argument("--hz", type=float, default=250.0, help="HIL_SENSOR rate")
    p.add_argument("--speed", type=float, default=0.0,
                   help="Wall-clock speed cap (x realtime). 0 = unlimited (as fast "
                        "as the handshake allows). 1.0 = realtime. Lockstep is "
                        "preserved at any value.")
    p.add_argument("--no-gps", action="store_true", help="Skip HIL_GPS (attitude-only bring-up)")
    run(p.parse_args())


if __name__ == "__main__":
    main()