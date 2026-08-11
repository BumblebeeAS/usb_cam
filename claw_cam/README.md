# claw_cam

Bringup for the AUV4 claw camera (DWE exploreHD), driven by `usb_cam`.

## Why it lives in the usb_cam fork

This package is inseparable from the driver patches on `bumblebee/humble`:

- `usb_cam_test.yaml` sets exposure and white-balance parameters that only take
  effect with the kernel-5.15 UVC control renames applied.
- The nodes and the launch file assume `SensorDataQoS` publishers — a reliable
  subscriber will not match the patched driver.
- `pixel_format: mjpeg2rgb` is only usable with the MJPEG decode fix.

Keeping both in one repo means one `.repos` entry pins a driver and its
configuration to the same commit, instead of two repos that can drift apart.

The repo therefore holds two packages: `usb_cam` (the upstream fork, at the repo
root) and `claw_cam` (this directory).

## Layout

```
launch/usb_cam_test.launch.py   driver + watchdog + autotune, v4l2 control seeding
config/usb_cam_test.yaml        usb_cam parameters for the claw cam
config/camera_autotune.yaml     autotune loop gains, targets, trim dead-bands
config/fastdds_claw_bigshm.xml  16MB SHM segment so a raw frame fits (see file)
config/calib/claw_cam.yaml      Kalibr intrinsics, calibrated at 800x600
scripts/watch_exposure.sh       live v4l2 control readout while tuning
```

## Nodes

**`camera_autotune_node`** — closed-loop exposure control. Forces the camera to
manual AE at startup and drives `exposure_time_absolute` from measured image
brightness toward `target_mean`, capped at one frame period so it cannot starve
framerate. `gamma` / `contrast` / `backlight_compensation` are slow secondary
trims with dead-bands and cooldowns, off by default. Controls are discovered at
runtime from `v4l2-ctl --list-ctrls`, so missing or renamed controls are skipped
rather than fatal.

**`camera_freeze_watchdog_node`** — detects a frame gap and records evidence
(per-thread `/proc/<tid>/syscall` and `wchan`, `strace` where ptrace is allowed,
`v4l2-ctl --all`, kernel log tail). Mode 1 (default) detects and records only;
Mode 2 (`restart_enabled:=true`) additionally kills the driver so the launch
file's `respawn` revives it, then sweeps orphaned SHM segments.

This exists because of a fastrtps 2.6.x AB-BA deadlock between the flow
controller delivering a reliable sample and the EDP thread unpairing a departing
subscriber, which froze the driver silently on subscriber churn. That is fixed at
the source by the `SensorDataQoS` commit; the watchdog stays as a safety net for
the still-reliable rosout/parameter writers and for new failure modes.

## Run

```bash
ros2 launch claw_cam usb_cam_test.launch.py
```

Both nodes are live-tunable:

```bash
ros2 param set /auv4/claw_cam/camera_autotune target_mean 130
ros2 param set /auv4/claw_cam/camera_freeze_watchdog restart_enabled true
```

Requires `v4l-utils` in the container — both nodes and the launch file shell out
to `v4l2-ctl`.
