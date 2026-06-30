#!/usr/bin/env python3
"""
test_offboard_figure8_3d_yaw.py

PX4 OFFBOARD 3D figure-eight validation with yaw tracking.

This extends test_offboard_figure8_3d.py:
  - reads LOCAL_POSITION_NED and ATTITUDE
  - commands position plus yaw
  - default yaw mode is tangent-rate-limited
  - logs yaw command, actual yaw, yaw error, yaw rate
  - plots 3D path, position error, speed, yaw tracking
  - broadcasts actual/target/yaw over UDP for live_udp_3d_viewer_yaw.py

Coordinates:
  PX4 LOCAL_NED:
    x = North [m]
    y = East  [m]
    z = Down  [m]

Yaw convention:
  PX4 LOCAL_NED yaw:
    yaw = atan2(East velocity, North velocity)
  so yaw=0 points North, yaw=+90deg points East.

Run:
  python test_offboard_figure8_3d_yaw.py --speed 10.0

With live viewer:
  Terminal A:
    python live_udp_3d_viewer_yaw.py --port 14660 --range 24 --zmin 0 --zmax 25

  Terminal B:
    python test_offboard_figure8_3d_yaw.py --speed 10.0 --viz-udp 127.0.0.1:14660
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

mav = mavutil.mavlink

TYPEMASK_POS_ONLY = (
    mav.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

TYPEMASK_POS_YAW = (
    mav.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


@dataclass
class LocalPosition:
    t_boot_s: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


@dataclass
class Attitude:
    t_boot_s: float
    roll: float
    pitch: float
    yaw: float
    rollspeed: float
    pitchspeed: float
    yawspeed: float


@dataclass
class State:
    lp: Optional[LocalPosition] = None
    att: Optional[Attitude] = None


@dataclass
class Target:
    x: float
    y: float
    z: float
    name: str
    yaw_raw: float = 0.0
    yaw_cmd: float = 0.0


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def advance_angle_limited(prev: float, desired: float, max_step: float) -> float:
    err = wrap_pi(desired - prev)
    if err > max_step:
        err = max_step
    elif err < -max_step:
        err = -max_step
    return wrap_pi(prev + err)


def yaw_error(actual: float, command: float) -> float:
    return wrap_pi(command - actual)


def param_set(m, name: str, value: float, ptype=mav.MAV_PARAM_TYPE_REAL32) -> None:
    m.mav.param_set_send(
        m.target_system,
        m.target_component,
        name.encode(),
        float(value),
        ptype,
    )


def send_position_setpoint(m, t_boot_s: float, target: Target, use_yaw: bool) -> None:
    mask = TYPEMASK_POS_YAW if use_yaw else TYPEMASK_POS_ONLY
    yaw = target.yaw_cmd if use_yaw else 0.0

    t_ms = int(max(0.0, t_boot_s) * 1000.0) & 0xFFFFFFFF

    m.mav.set_position_target_local_ned_send(
        t_ms,
        m.target_system,
        m.target_component,
        mav.MAV_FRAME_LOCAL_NED,
        mask,
        float(target.x), float(target.y), float(target.z),
        0.0, 0.0, 0.0,       # ignored velocity
        0.0, 0.0, 0.0,       # ignored accel/force
        float(yaw), 0.0,     # yaw command, yaw-rate ignored
    )


def set_offboard_and_arm(m) -> None:
    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mav.MAV_CMD_DO_SET_MODE,
        0,
        mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6,  # PX4 custom main mode OFFBOARD
        0, 0, 0, 0, 0,
    )

    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mav.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )


def disarm(m) -> None:
    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mav.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0,
    )


def drain_latest_state(m, state: State) -> State:
    while True:
        msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
        if msg is None:
            break

        typ = msg.get_type()
        if typ == "LOCAL_POSITION_NED":
            state.lp = LocalPosition(
                t_boot_s=float(msg.time_boot_ms) / 1000.0,
                x=float(msg.x),
                y=float(msg.y),
                z=float(msg.z),
                vx=float(msg.vx),
                vy=float(msg.vy),
                vz=float(msg.vz),
            )
        elif typ == "ATTITUDE":
            state.att = Attitude(
                t_boot_s=float(msg.time_boot_ms) / 1000.0,
                roll=float(msg.roll),
                pitch=float(msg.pitch),
                yaw=float(msg.yaw),
                rollspeed=float(msg.rollspeed),
                pitchspeed=float(msg.pitchspeed),
                yawspeed=float(msg.yawspeed),
            )

    return state


def wait_for_state(m, timeout_wall_s: float = 30.0) -> State:
    print("[fig8yaw] waiting for LOCAL_POSITION_NED + ATTITUDE...")
    deadline = time.perf_counter() + timeout_wall_s
    state = State()

    while time.perf_counter() < deadline:
        msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=True, timeout=1.0)
        if msg is None:
            continue

        typ = msg.get_type()
        if typ == "LOCAL_POSITION_NED":
            state.lp = LocalPosition(
                t_boot_s=float(msg.time_boot_ms) / 1000.0,
                x=float(msg.x),
                y=float(msg.y),
                z=float(msg.z),
                vx=float(msg.vx),
                vy=float(msg.vy),
                vz=float(msg.vz),
            )
        elif typ == "ATTITUDE":
            state.att = Attitude(
                t_boot_s=float(msg.time_boot_ms) / 1000.0,
                roll=float(msg.roll),
                pitch=float(msg.pitch),
                yaw=float(msg.yaw),
                rollspeed=float(msg.rollspeed),
                pitchspeed=float(msg.pitchspeed),
                yawspeed=float(msg.yawspeed),
            )

        if state.lp is not None and state.att is not None:
            return state

    raise RuntimeError("No complete PX4 state received. EKF/attitude telemetry not ready.")


def smoothstep01(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return 6*u**5 - 15*u**4 + 10*u**3


def lerp(a: float, b: float, s: float) -> float:
    return a + s * (b - a)


def figure8_target_raw(
    t_rel: float,
    start: Target,
    altitude: float,
    climb_s: float,
    hold_s: float,
    period: float,
    laps: float,
    x_amp: float,
    y_amp: float,
    z_amp: float,
    return_s: float,
    final_hold_s: float,
) -> tuple[Target, float, bool]:
    center = Target(0.0, 0.0, -altitude, "center", 0.0, 0.0)

    fig_start = climb_s + hold_s
    fig_duration = period * laps
    return_start = fig_start + fig_duration
    final_start = return_start + return_s
    mission_duration = final_start + final_hold_s

    if t_rel < climb_s:
        s = smoothstep01(t_rel / climb_s)
        # During climb, keep heading North.
        return Target(
            lerp(start.x, center.x, s),
            lerp(start.y, center.y, s),
            lerp(start.z, center.z, s),
            "climb",
            0.0,
            0.0,
        ), mission_duration, False

    if t_rel < fig_start:
        return Target(center.x, center.y, center.z, "center_hold", 0.0, 0.0), mission_duration, False

    if t_rel < return_start:
        tau = t_rel - fig_start
        theta = 2.0 * math.pi * tau / period

        x = x_amp * math.sin(theta)
        y = y_amp * math.sin(2.0 * theta)
        z = -altitude - z_amp * math.sin(2.0 * theta + math.pi / 3.0)

        w = 2.0 * math.pi / period
        dx = x_amp * w * math.cos(theta)
        dy = 2.0 * y_amp * w * math.cos(2.0 * theta)

        # PX4 NED yaw: yaw = atan2(East velocity, North velocity).
        yaw_raw = math.atan2(dy, dx)

        lap_idx = int(tau // period) + 1
        return Target(x, y, z, f"figure8_lap_{lap_idx}", yaw_raw, yaw_raw), mission_duration, False

    theta_end = 2.0 * math.pi * laps
    end = Target(
        x_amp * math.sin(theta_end),
        y_amp * math.sin(2.0 * theta_end),
        -altitude - z_amp * math.sin(2.0 * theta_end + math.pi / 3.0),
        "fig8_end",
        0.0,
        0.0,
    )

    if t_rel < final_start:
        s = smoothstep01((t_rel - return_start) / return_s)
        return Target(
            lerp(end.x, center.x, s),
            lerp(end.y, center.y, s),
            lerp(end.z, center.z, s),
            "return_center",
            0.0,
            0.0,
        ), mission_duration, False

    if t_rel < mission_duration:
        return Target(center.x, center.y, center.z, "final_hold", 0.0, 0.0), mission_duration, False

    return Target(center.x, center.y, center.z, "done", 0.0, 0.0), mission_duration, True


def dist3(lp: LocalPosition, t: Target) -> float:
    return math.sqrt((lp.x - t.x)**2 + (lp.y - t.y)**2 + (lp.z - t.z)**2)


def speed_norm(lp: LocalPosition) -> float:
    return math.sqrt(lp.vx**2 + lp.vy**2 + lp.vz**2)


def make_udp_sender(viz_udp: str | None):
    if not viz_udp:
        return None, None

    host, port_s = viz_udp.rsplit(":", 1)
    addr = (host, int(port_s))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    print(f"[fig8yaw] live viewer UDP enabled -> {host}:{port_s}")
    return sock, addr


def send_viz(sock, addr, lp: LocalPosition, att: Attitude, target: Target,
             t_rel: float, err: float, spd: float, yaw_err: float) -> None:
    if sock is None or addr is None:
        return

    msg = {
        "type": "sample",
        "source": "px4_fig8_yaw",
        "t_rel": t_rel,
        "actual": {"x": lp.x, "y": lp.y, "z": lp.z},
        "target": {"x": target.x, "y": target.y, "z": target.z},
        "attitude": {
            "roll": att.roll,
            "pitch": att.pitch,
            "yaw": att.yaw,
            "yawspeed": att.yawspeed,
        },
        "yaw_cmd": target.yaw_cmd,
        "yaw_raw": target.yaw_raw,
        "yaw_err": yaw_err,
        "err": err,
        "speed": spd,
        "seg": target.name,
    }

    try:
        sock.sendto(json.dumps(msg, separators=(",", ":")).encode("utf-8"), addr)
    except OSError:
        pass


def set_axes_equal_3d(ax) -> None:
    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()

    xmid = 0.5 * (xlim[0] + xlim[1])
    ymid = 0.5 * (ylim[0] + ylim[1])
    zmid = 0.5 * (zlim[0] + zlim[1])

    radius = 0.5 * max(
        abs(xlim[1] - xlim[0]),
        abs(ylim[1] - ylim[0]),
        abs(zlim[1] - zlim[0]),
        1.0,
    )

    ax.set_xlim3d(xmid - radius, xmid + radius)
    ax.set_ylim3d(ymid - radius, ymid + radius)
    ax.set_zlim3d(zmid - radius, zmid + radius)


def write_csv(rows: list[dict], path: Path) -> None:
    if not rows:
        print("[fig8yaw] no rows to save")
        return

    fields = [
        "t_wall", "t_sim", "t_rel", "seg_idx", "seg_name",
        "x", "y", "z", "vx", "vy", "vz",
        "tx", "ty", "tz",
        "err", "speed",
        "roll", "pitch", "yaw",
        "rollspeed", "pitchspeed", "yawspeed",
        "yaw_cmd", "yaw_raw", "yaw_err",
    ]

    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    print(f"[fig8yaw] saved {path}")


def plot_results(rows: list[dict], out_prefix: Path, show: bool) -> None:
    if not rows:
        return

    actual_n = np.array([r["x"] for r in rows])
    actual_e = np.array([r["y"] for r in rows])
    actual_u = -np.array([r["z"] for r in rows])

    target_n = np.array([r["tx"] for r in rows])
    target_e = np.array([r["ty"] for r in rows])
    target_u = -np.array([r["tz"] for r in rows])

    t = np.array([r["t_rel"] for r in rows])
    err = np.array([r["err"] for r in rows])
    spd = np.array([r["speed"] for r in rows])
    yaw = np.unwrap(np.array([r["yaw"] for r in rows]))
    yaw_cmd = np.unwrap(np.array([r["yaw_cmd"] for r in rows]))
    yaw_err = np.array([r["yaw_err"] for r in rows])
    yawspeed = np.array([r["yawspeed"] for r in rows])

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(target_n, target_e, target_u, linestyle="--", linewidth=1.5, label="moving target")
    ax.plot(actual_n, actual_e, actual_u, linewidth=2.0, label="PX4 actual")
    ax.scatter([target_n[0]], [target_e[0]], [target_u[0]], s=45, marker="o", label="start")
    ax.scatter([target_n[-1]], [target_e[-1]], [target_u[-1]], s=45, marker="^", label="end")
    ax.set_title("PX4 OFFBOARD 3D Figure-Eight Tracking with Yaw")
    ax.set_xlabel("North x [m]")
    ax.set_ylabel("East y [m]")
    ax.set_zlabel("Up -z [m]")
    ax.legend()
    set_axes_equal_3d(ax)
    fig.tight_layout()

    path3d = out_prefix.with_suffix(".png")
    fig.savefig(path3d, dpi=180)
    print(f"[fig8yaw] saved {path3d}")

    fig2 = plt.figure(figsize=(10, 4))
    ax2 = fig2.add_subplot(111)
    ax2.plot(t, err)
    ax2.set_title("Figure-eight position tracking error")
    ax2.set_xlabel("mission sim time [s]")
    ax2.set_ylabel("||pos - target|| [m]")
    ax2.grid(True)
    fig2.tight_layout()
    err_path = out_prefix.with_name(out_prefix.stem + "_error.png")
    fig2.savefig(err_path, dpi=180)
    print(f"[fig8yaw] saved {err_path}")

    fig3 = plt.figure(figsize=(10, 4))
    ax3 = fig3.add_subplot(111)
    ax3.plot(t, spd)
    ax3.set_title("PX4 local velocity norm")
    ax3.set_xlabel("mission sim time [s]")
    ax3.set_ylabel("speed [m/s]")
    ax3.grid(True)
    fig3.tight_layout()
    spd_path = out_prefix.with_name(out_prefix.stem + "_speed.png")
    fig3.savefig(spd_path, dpi=180)
    print(f"[fig8yaw] saved {spd_path}")

    fig4 = plt.figure(figsize=(10, 5))
    ax4 = fig4.add_subplot(111)
    ax4.plot(t, np.degrees(yaw_cmd), label="yaw command")
    ax4.plot(t, np.degrees(yaw), label="actual yaw")
    ax4.set_title("Yaw tracking")
    ax4.set_xlabel("mission sim time [s]")
    ax4.set_ylabel("yaw [deg, unwrapped]")
    ax4.grid(True)
    ax4.legend()
    fig4.tight_layout()
    yaw_path = out_prefix.with_name(out_prefix.stem + "_yaw.png")
    fig4.savefig(yaw_path, dpi=180)
    print(f"[fig8yaw] saved {yaw_path}")

    fig5 = plt.figure(figsize=(10, 4))
    ax5 = fig5.add_subplot(111)
    ax5.plot(t, np.degrees(yaw_err), label="yaw error")
    ax5.plot(t, np.degrees(yawspeed), label="yaw rate")
    ax5.set_title("Yaw error and yaw rate")
    ax5.set_xlabel("mission sim time [s]")
    ax5.set_ylabel("deg / deg/s")
    ax5.grid(True)
    ax5.legend()
    fig5.tight_layout()
    yerr_path = out_prefix.with_name(out_prefix.stem + "_yaw_error.png")
    fig5.savefig(yerr_path, dpi=180)
    print(f"[fig8yaw] saved {yerr_path}")

    if show:
        plt.show()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--speed", type=float, default=10.0)
    ap.add_argument("--sim-stream-hz", type=float, default=50.0)
    ap.add_argument("--altitude", type=float, default=15.0)
    ap.add_argument("--x-amp", type=float, default=14.0)
    ap.add_argument("--y-amp", type=float, default=10.0)
    ap.add_argument("--z-amp", type=float, default=2.5)
    ap.add_argument("--period", type=float, default=36.0)
    ap.add_argument("--laps", type=float, default=3.0)
    ap.add_argument("--climb-s", type=float, default=12.0)
    ap.add_argument("--hold-s", type=float, default=2.0)
    ap.add_argument("--return-s", type=float, default=6.0)
    ap.add_argument("--final-hold-s", type=float, default=3.0)
    ap.add_argument("--yaw-mode", choices=["ignore", "hold", "tangent"], default="tangent",
                    help="tangent tracks path heading; hold commands yaw=0; ignore does not command yaw.")
    ap.add_argument("--max-yaw-rate-deg", type=float, default=60.0,
                    help="Rate limit applied to yaw command in tangent mode.")
    ap.add_argument("--abort-err", type=float, default=100.0)
    ap.add_argument("--abort-down-z", type=float, default=20.0)
    ap.add_argument("--abort-speed", type=float, default=45.0)
    ap.add_argument("--abort-yawrate-deg", type=float, default=720.0)
    ap.add_argument("--viz-udp", default="127.0.0.1:14660",
                    help="UDP JSON stream for live_udp_3d_viewer_yaw.py. Use '' to disable.")
    ap.add_argument("--out-prefix", default="px4_figure8_3d_yaw")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--no-disarm", action="store_true")
    args = ap.parse_args()

    wall_stream_hz = max(5.0, args.sim_stream_hz * max(1.0, args.speed))
    period_wall = 1.0 / wall_stream_hz
    max_yaw_step_per_sim_s = math.radians(args.max_yaw_rate_deg)

    viz_udp = args.viz_udp if args.viz_udp else None
    viz_sock, viz_addr = make_udp_sender(viz_udp)

    print(f"[fig8yaw] connecting {args.url}")
    m = mavutil.mavlink_connection(args.url)
    m.wait_heartbeat()
    print(f"[fig8yaw] heartbeat sys={m.target_system} comp={m.target_component}")
    print(f"[fig8yaw] wall stream rate = {wall_stream_hz:.1f} Hz "
          f"({args.sim_stream_hz:.1f} Hz sim target @ speed={args.speed:.2f}x)")
    print(f"[fig8yaw] params: A={args.x_amp:.1f}m B={args.y_amp:.1f}m "
          f"z_amp={args.z_amp:.1f}m period={args.period:.1f}s laps={args.laps:.1f} "
          f"yaw_mode={args.yaw_mode} max_yaw_rate={args.max_yaw_rate_deg:.1f}deg/s")

    print("[fig8yaw] setting headless-friendly PX4 params")
    param_set(m, "NAV_RCL_ACT", 0, mav.MAV_PARAM_TYPE_INT32)
    param_set(m, "NAV_DLL_ACT", 0, mav.MAV_PARAM_TYPE_INT32)
    param_set(m, "COM_RC_IN_MODE", 4, mav.MAV_PARAM_TYPE_INT32)

    state = wait_for_state(m)

    print("[fig8yaw] draining telemetry for 1 wall second")
    t_end = time.perf_counter() + 1.0
    while time.perf_counter() < t_end:
        state = drain_latest_state(m, state)
        time.sleep(0.02)

    if state.lp is None or state.att is None:
        raise RuntimeError("Incomplete PX4 state after drain.")

    # Prestream current position so PX4 accepts OFFBOARD.
    lp = state.lp
    att = state.att
    yaw_cmd_state = 0.0 if args.yaw_mode == "hold" else att.yaw
    pre_target = Target(lp.x, lp.y, lp.z, "pre_offboard_hold", yaw_cmd_state, yaw_cmd_state)

    print(f"[fig8yaw] pre-streaming current NED=({lp.x:+.2f},{lp.y:+.2f},{lp.z:+.2f}) "
          f"yaw={math.degrees(att.yaw):+.1f}deg")

    t_pre = time.perf_counter()
    while time.perf_counter() - t_pre < 2.0:
        state = drain_latest_state(m, state)
        if state.lp is not None:
            lp = state.lp
            pre_target = Target(lp.x, lp.y, lp.z, "pre_offboard_hold", yaw_cmd_state, yaw_cmd_state)
        send_position_setpoint(m, lp.t_boot_s, pre_target, use_yaw=(args.yaw_mode != "ignore"))
        time.sleep(period_wall)

    print("[fig8yaw] -> OFFBOARD + ARM")
    set_offboard_and_arm(m)

    state = drain_latest_state(m, state)
    lp = state.lp
    att = state.att
    if lp is None or att is None:
        raise RuntimeError("Incomplete state after arming.")

    start_target = Target(lp.x, lp.y, lp.z, "start", yaw_cmd_state, yaw_cmd_state)
    _, mission_duration, _ = figure8_target_raw(
        0.0, start_target, args.altitude, args.climb_s, args.hold_s,
        args.period, args.laps, args.x_amp, args.y_amp, args.z_amp,
        args.return_s, args.final_hold_s,
    )
    print(f"[fig8yaw] mission duration = {mission_duration:.1f}s sim")

    rows: list[dict] = []
    wall0 = time.perf_counter()
    sim0: Optional[float] = None
    last_sim_t: Optional[float] = None
    last_print_bucket = -1
    aborted = False

    try:
        while True:
            loop_wall_start = time.perf_counter()

            state = drain_latest_state(m, state)
            lp = state.lp
            att = state.att
            if lp is None or att is None:
                time.sleep(period_wall)
                continue

            if sim0 is None:
                sim0 = lp.t_boot_s

            t_rel = lp.t_boot_s - sim0
            dt_sim = 0.0 if last_sim_t is None else max(0.0, t_rel - last_sim_t)
            last_sim_t = t_rel

            target, mission_duration, done = figure8_target_raw(
                t_rel, start_target, args.altitude, args.climb_s, args.hold_s,
                args.period, args.laps, args.x_amp, args.y_amp, args.z_amp,
                args.return_s, args.final_hold_s,
            )

            if args.yaw_mode == "ignore":
                target.yaw_cmd = 0.0
            elif args.yaw_mode == "hold":
                target.yaw_cmd = 0.0
            else:
                max_step = max_yaw_step_per_sim_s * max(dt_sim, 1.0 / args.sim_stream_hz)
                yaw_cmd_state = advance_angle_limited(yaw_cmd_state, target.yaw_raw, max_step)
                target.yaw_cmd = yaw_cmd_state

            send_position_setpoint(m, lp.t_boot_s, target, use_yaw=(args.yaw_mode != "ignore"))

            err = dist3(lp, target)
            spd = speed_norm(lp)
            ye = yaw_error(att.yaw, target.yaw_cmd) if args.yaw_mode != "ignore" else 0.0
            send_viz(viz_sock, viz_addr, lp, att, target, t_rel, err, spd, ye)

            rows.append({
                "t_wall": time.perf_counter() - wall0,
                "t_sim": lp.t_boot_s,
                "t_rel": t_rel,
                "seg_idx": 0,
                "seg_name": target.name,
                "x": lp.x,
                "y": lp.y,
                "z": lp.z,
                "vx": lp.vx,
                "vy": lp.vy,
                "vz": lp.vz,
                "tx": target.x,
                "ty": target.y,
                "tz": target.z,
                "err": err,
                "speed": spd,
                "roll": att.roll,
                "pitch": att.pitch,
                "yaw": att.yaw,
                "rollspeed": att.rollspeed,
                "pitchspeed": att.pitchspeed,
                "yawspeed": att.yawspeed,
                "yaw_cmd": target.yaw_cmd,
                "yaw_raw": target.yaw_raw,
                "yaw_err": ye,
            })

            bucket = int(t_rel)
            if bucket != last_print_bucket:
                last_print_bucket = bucket
                print(
                    f"  t={t_rel:6.1f}/{mission_duration:5.1f}s "
                    f"{target.name:14s} "
                    f"err={err:5.2f}m speed={spd:5.2f}m/s "
                    f"yaw={math.degrees(att.yaw):+7.1f}deg "
                    f"yaw_cmd={math.degrees(target.yaw_cmd):+7.1f}deg "
                    f"yaw_err={math.degrees(ye):+6.1f}deg "
                    f"yawrate={math.degrees(att.yawspeed):+6.1f}deg/s"
                )

            if err > args.abort_err:
                print(f"[fig8yaw] ABORT: err={err:.1f}m > {args.abort_err:.1f}m")
                aborted = True
                break

            if lp.z > args.abort_down_z:
                print(f"[fig8yaw] ABORT: NED z={lp.z:.1f}m means vehicle is below origin / falling")
                aborted = True
                break

            if spd > args.abort_speed:
                print(f"[fig8yaw] ABORT: speed={spd:.1f}m/s > {args.abort_speed:.1f}m/s")
                aborted = True
                break

            if abs(math.degrees(att.yawspeed)) > args.abort_yawrate_deg:
                print(f"[fig8yaw] ABORT: yawspeed={math.degrees(att.yawspeed):.1f}deg/s")
                aborted = True
                break

            if done:
                print("[fig8yaw] mission complete")
                break

            sleep_s = period_wall - (time.perf_counter() - loop_wall_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\n[fig8yaw] interrupted")
        aborted = True

    finally:
        final_target = target if "target" in locals() else pre_target
        for _ in range(int(wall_stream_hz * 0.25)):
            state = drain_latest_state(m, state)
            if state.lp is not None:
                send_position_setpoint(m, state.lp.t_boot_s, final_target, use_yaw=(args.yaw_mode != "ignore"))
            time.sleep(period_wall)

        if not args.no_disarm:
            print("[fig8yaw] disarming")
            disarm(m)

        if viz_sock is not None:
            try:
                viz_sock.close()
            except OSError:
                pass

    out_prefix = Path(args.out_prefix)
    write_csv(rows, out_prefix.with_suffix(".csv"))
    plot_results(rows, out_prefix.with_suffix(".png"), show=args.show)

    if aborted:
        print("[fig8yaw] ended after abort/interruption; plots still saved for diagnosis")


if __name__ == "__main__":
    main()
