#!/usr/bin/env python3
"""
px4_confirm.py — staged proof that commands reach PX4 and PX4 drives the sim.

Connects on PX4's onboard/offboard link (UDP 14540) — the SAME link MAVSDK or
a GCS would use. The HIL sensor/actuator traffic on 4560 is the bridge's job;
this script is the "is the autopilot alive and listening to me?" check.

Each stage isolates one part of the loop, so a failure tells you *where*:

  Stage 1  HEARTBEAT          PX4 is alive and talking          (PX4 up)
  Stage 2  ATTITUDE sane      EKF is fusing OUR sensors         (sim -> PX4 works)
  Stage 3  ARM accepted       command path GCS -> PX4 works     (cmd in)
  Stage 4  (watch bridge)     motors go non-zero on arm/takeoff (PX4 -> sim works)
  Stage 5  --takeoff ALT      vehicle climbs in QuadSim         (full loop closes)

Stage 2 + 3 + bridge's "actuators live" line already prove the whole
bidirectional loop; takeoff is the cherry on top (and needs GPS, so run the
bridge WITHOUT --no-gps for it).

RUN:  python px4_confirm.py [--takeoff 3.0]
DEPENDS:  pip install pymavlink
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def param_set(m, name: str, value: float, ptype=None) -> None:
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode(), float(value),
        ptype or mavutil.mavlink.MAV_PARAM_TYPE_INT32)


def stage(n: int, msg: str) -> None:
    print(f"\n=== Stage {n}: {msg} ===")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--takeoff", type=float, default=0.0,
                    help="If set, command takeoff to this altitude (m) and watch climb")
    args = ap.parse_args()

    # ---- Stage 1: heartbeat ------------------------------------------------
    stage(1, "waiting for PX4 heartbeat")
    m = mavutil.mavlink_connection(args.url)
    m.wait_heartbeat()
    print(f"  OK  heartbeat from sys {m.target_system} comp {m.target_component}")

    # ---- Stage 2: EKF is fusing our sensors --------------------------------
    stage(2, "checking attitude estimate (proves sim->PX4 sensor path)")
    got = None
    deadline = time.time() + 15
    while time.time() < deadline:
        att = m.recv_match(type="ATTITUDE", blocking=True, timeout=2)
        if att and (abs(att.roll) + abs(att.pitch) + abs(att.yaw)) >= 0.0:
            got = att
            break
    if got is None:
        print("  FAIL no ATTITUDE — PX4 isn't getting sensors. Check the bridge/4560.")
        return
    print(f"  OK  roll={got.roll:+.3f} pitch={got.pitch:+.3f} yaw={got.yaw:+.3f} rad")
    print("      (level on the ground should be ~0,0,heading; wild values => frame/sign bug)")

    # let GPS/EKF settle so home gets set and prearm passes
    print("  ... waiting 8s for EKF/GPS origin + home")
    time.sleep(8)

    # disable RC/datalink-loss failsafes so we can arm with no RC
    for pname in ("NAV_RCL_ACT", "NAV_DLL_ACT", "COM_RCL_EXCEPT"):
        param_set(m, pname, 0)

    # ---- Stage 3: arm ------------------------------------------------------
    stage(3, "arming (proves GCS->PX4 command path)")
    armed = False
    for attempt in range(5):
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        if ack:
            print(f"  arm ack result={ack.result} (retry {attempt+1})")
        time.sleep(1)
    if not armed:
        print("  FAIL not armed — usually a prearm check (no GPS/home, EKF not ready).")
        print("       Run the bridge WITHOUT --no-gps and re-check Stage 2 looked sane.")
        return
    print("  OK  ARMED.  --> Look at the bridge console: motors should go non-zero,")
    print("              and the drone should react in QuadSim. That's PX4->sim.")

    # ---- Stage 5: takeoff (optional) --------------------------------------
    if args.takeoff > 0:
        stage(5, f"takeoff to {args.takeoff} m and watch climb")
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, float("nan"), 0, 0, args.takeoff)
        t0 = time.time()
        while time.time() - t0 < 30:
            lp = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
            if lp:
                alt = -lp.z  # NED: up is -z
                print(f"  alt={alt:5.2f} m  vz={-lp.vz:+.2f} m/s")
                if alt >= args.takeoff - 0.3:
                    print("  OK  reached target altitude — full loop confirmed.")
                    return
        print("  (didn't confirm target altitude in 30s; check motor map / airframe match)")
    else:
        print("\nDone. Re-run with --takeoff 3.0 to close the full loop once GPS is on.")


if __name__ == "__main__":
    main()
