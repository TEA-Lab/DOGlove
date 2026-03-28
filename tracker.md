# tracker.py Usage Guide

This project currently provides SteamVR tracking through [`tracker.py`](./tracker.py) and `ViveTrackerModule`.

The examples below are aligned with how we uses the tracker pipeline.

## Provenance

`tracker.py` is adapted from the SteamVR_Tracking GitHub repository and then updated locally for this project.

- Upstream reference: https://github.com/tianshengs/SteamVR_Tracking
- Upstream base implementation file: `triad_openvr.py`

## Quick Start

```python
from tracker import ViveTrackerModule

v_tracker = ViveTrackerModule()
v_tracker.print_discovered_objects()

# Typical key names: tracker_1, controller_1, hmd_1, tracking_reference_1
tracker = v_tracker.devices["tracker_1"]

pose_euler = tracker.get_pose_euler()        # [x, y, z, yaw, pitch, roll]
pose_quat = tracker.get_pose_quaternion()    # [x, y, z, qx, qy, qz, qw]
pose_T = tracker.get_T()                     # 4x4 homogeneous transform
```

## Use with the robot control script

We uses the tracker in three phases:

1. Initialize tracker module and select a device:

```python
v_tracker = ViveTrackerModule()
v_tracker.print_discovered_objects()
tracker = v_tracker.devices["tracker_1"]
```

2. Capture baseline state before teleop loop:

```python
tracker_init_state = tracker.get_pose_euler()
init_rotation = tracker.get_pose_quaternion()[3:7]
```

3. Read tracker pose continuously in control loop:

```python
position = tracker.get_pose_euler()
rotation = tracker.get_pose_quaternion()[3:7]
```

Then robot-specific code maps tracker deltas to robot commands (for Franka this is done in [`control_franka.py`](https://github.com/Fanqi-Lin/Data-Scaling-Laws/blob/master/scripts_real/control_franka.py)).

## API Notes

- `ViveTrackerModule()`:
  - Initializes OpenVR (`openvr.init(...)`), discovers connected devices, and populates `devices`.
- `print_discovered_objects()`:
  - Prints all discovered devices by class.
- `devices`:
  - Dictionary of tracked devices keyed by generated names, for example `tracker_1`.
- `get_pose_euler()`:
  - Returns `[x, y, z, yaw, pitch, roll]` if pose is valid; otherwise `None`.
- `get_pose_quaternion()`:
  - Returns `[x, y, z, qx, qy, qz, qw]` if pose is valid; otherwise `None`.
- `get_T()`:
  - Returns a 4x4 homogeneous transform matrix as `numpy.ndarray`.

## Important Quaternion Convention

In this repository's `track.py`, quaternion output order is:

- `qx, qy, qz, qw` (with position prepended to make `[x, y, z, qx, qy, qz, qw]`)

If you integrate with code expecting `qw, qx, qy, qz`, reorder explicitly.

## Known Caveats

- `get_pose_euler()` currently prints every valid reading to stdout before returning the value.
- `controller_state_to_dict()` is a placeholder and does not expose full controller button parsing.
- Device names like `tracker_1` depend on discovery order; use `print_discovered_objects()` to confirm names.

## Minimal Environment For Track-Only Use

Use a clean Python environment and install only the runtime packages needed by `tracker.py`:

```bash
pip install numpy openvr
```

Quick check (with SteamVR running and tracker powered on):

```bash
python -c "from tracker import ViveTrackerModule; v=ViveTrackerModule(); v.print_discovered_objects()"
```
