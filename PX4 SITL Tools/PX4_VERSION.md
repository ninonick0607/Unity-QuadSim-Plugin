# Tested PX4 Version

PX4 behavior can change across commits and releases. For reproducible QuadSim experiments, pin the PX4-Autopilot checkout used for validation.

## Current tested configuration

Fill this in for each release of the QuadSim plugin.

```text
QuadSim plugin version: v0.1.0
QuadSim SDK version:   <quadsim-sdk version or commit>
PX4-Autopilot commit:  <git rev-parse HEAD>
PX4 tag/branch:        <tag or branch>
PX4 target:            px4_sitl none_iris
QGroundControl:        optional
OS:                    <Ubuntu/WSL2/etc.>
Unity version:         Unity 6
```

Get the PX4 commit:

```bash
cd PX4-Autopilot
git rev-parse HEAD
```

Get the current branch/tag context:

```bash
git status
git describe --tags --always
```

## Recommended rule

For any published result, paper figure, or reproducible demo, record:

- QuadSim plugin git tag.
- QuadSim SDK version/commit.
- PX4-Autopilot commit.
- PX4 airframe target.
- Any changed PX4 params.
- Whether QGroundControl was attached.
- Whether the run was real time or faster than real time.

## Why this matters

PX4 SITL is an active project. EKF defaults, failsafe defaults, mixer behavior, MAVLink message handling, and SITL startup behavior can change.

A setup that flies well on one PX4 commit may need parameter or bridge adjustments on another.
