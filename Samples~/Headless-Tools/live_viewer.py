#!/usr/bin/env python3
"""
live_viewer.py — general live 3D viewer for QuadSim headless runs.

Listens for UDP JSON from quadsim.viz.UdpViz. Two packet types:

  {"type": "path",   "points": [[x,y,z], ...]}                 # one-shot ghost
  {"type": "sample", "t_rel": t,
                     "actual": [x,y,z], "target": [x,y,z],     # target optional
                     "err": e, "speed": s, "trial": n}

FLU, z up: positions are plotted as-is — no frame flips.

Draws the flown path, the commanded-setpoint trail, current drone/setpoint
markers, and (if sent) a static planned-trajectory ghost. The path resets on a
trial change or a backwards time jump; the ghost persists.

Run:  python live_viewer.py --port 14660
Controls: left-drag rotate, scroll zoom, right-drag pan.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np


def _xyz(v):
    """Accept either {'x','y','z'} or [x,y,z]."""
    if isinstance(v, dict):
        return float(v["x"]), float(v["y"]), float(v["z"])
    if isinstance(v, (list, tuple)) and len(v) >= 3:
        return float(v[0]), float(v[1]), float(v[2])
    raise ValueError(f"bad xyz payload: {v!r}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=14660)
    ap.add_argument("--max-points", type=int, default=10000)
    ap.add_argument("--draw-hz", type=float, default=20.0)
    ap.add_argument("--range", type=float, default=24.0)
    ap.add_argument("--zmin", type=float, default=0.0)
    ap.add_argument("--zmax", type=float, default=25.0)
    ap.add_argument("--autoscale", action="store_true")
    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.setblocking(False)
    print(f"[viewer] listening on udp://{args.host}:{args.port}  (FLU, z up)")
    print("[viewer] controls: left-drag rotate, scroll zoom, right-drag pan")

    ax_d = deque(maxlen=args.max_points)
    ay_d = deque(maxlen=args.max_points)
    az_d = deque(maxlen=args.max_points)
    tx_d = deque(maxlen=args.max_points)
    ty_d = deque(maxlen=args.max_points)
    tz_d = deque(maxlen=args.max_points)

    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    actual_line, = ax.plot([], [], [], lw=2.0, label="actual")
    target_line, = ax.plot([], [], [], ls="--", lw=1.2, label="setpoint trail")
    actual_marker, = ax.plot([], [], [], marker="o", ms=7, ls="", label="drone")
    target_marker, = ax.plot([], [], [], marker="x", ms=8, ls="", label="setpoint")
    path_line, = ax.plot([], [], [], color="0.6", lw=1.0, label="planned path")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z up [m]")
    ax.legend(loc="upper left")
    ax.set_xlim(-args.range, args.range)
    ax.set_ylim(-args.range, args.range)
    ax.set_zlim(args.zmin, args.zmax)
    fig.tight_layout()
    fig.show()

    def set3d(line, xs, ys, zs):
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    latest = None
    current_trial = None
    last_t = None
    last_draw = 0.0

    try:
        while plt.fignum_exists(fig.number):
            # ---- drain every pending packet ----
            while True:
                try:
                    data, _ = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                try:
                    msg = json.loads(data.decode("utf-8"))
                except Exception as e:
                    if args.debug:
                        print(f"[viewer] bad json: {e}")
                    continue

                mtype = msg.get("type")

                if mtype == "path":
                    pts = msg.get("points", [])
                    if pts:
                        P = np.array([_xyz(p) for p in pts])
                        set3d(path_line, P[:, 0], P[:, 1], P[:, 2])
                    continue

                if mtype != "sample":
                    continue

                trial = int(msg.get("trial", -1))
                t_rel = float(msg.get("t_rel", 0.0))

                trial_changed = (current_trial is not None
                                 and trial != -1 and trial != current_trial)
                time_reset = (last_t is not None and t_rel < last_t - 0.5)
                if trial_changed or time_reset:
                    for d in (ax_d, ay_d, az_d, tx_d, ty_d, tz_d):
                        d.clear()
                    print(f"[viewer] reset path (trial {trial})")

                current_trial = trial
                last_t = t_rel

                axx, ayy, azz = _xyz(msg["actual"])
                ax_d.append(axx); ay_d.append(ayy); az_d.append(azz)
                if "target" in msg:
                    txx, tyy, tzz = _xyz(msg["target"])
                    tx_d.append(txx); ty_d.append(tyy); tz_d.append(tzz)
                latest = msg

            # ---- throttled redraw ----
            now = time.perf_counter()
            if latest is not None and now - last_draw >= 1.0 / max(1.0, args.draw_hz):
                last_draw = now

                set3d(actual_line, list(ax_d), list(ay_d), list(az_d))
                set3d(target_line, list(tx_d), list(ty_d), list(tz_d))
                if ax_d:
                    set3d(actual_marker, [ax_d[-1]], [ay_d[-1]], [az_d[-1]])
                if tx_d:
                    set3d(target_marker, [tx_d[-1]], [ty_d[-1]], [tz_d[-1]])

                if args.autoscale and len(ax_d) > 5:
                    X, Y, Z = np.array(ax_d), np.array(ay_d), np.array(az_d)
                    pad = 3.0
                    ax.set_xlim(float(X.min()) - pad, float(X.max()) + pad)
                    ax.set_ylim(float(Y.min()) - pad, float(Y.max()) + pad)
                    ax.set_zlim(float(Z.min()) - pad, float(Z.max()) + pad)

                ax.set_title(
                    f"QuadSim live | trial={latest.get('trial', -1)} "
                    f"t={latest.get('t_rel', 0.0):.1f}s "
                    f"err={latest.get('err', 0.0):.2f}m "
                    f"speed={latest.get('speed', 0.0):.2f}m/s"
                )

                fig.canvas.draw_idle()
                plt.pause(0.001)
            else:
                plt.pause(0.005)

    except KeyboardInterrupt:
        print("\n[viewer] interrupted")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
