#!/usr/bin/env bash
# Live-watch the claw cam's exposure-related v4l2 controls while camera_autotune
# drives them. Greps ONLY the exposure controls out of `v4l2-ctl --list-ctrls`.
#
# Usage:  ./scripts/watch_exposure.sh [video_device] [interval_sec]
#   ./scripts/watch_exposure.sh                 # /dev/video0, 0.5s
#   ./scripts/watch_exposure.sh /dev/video2 1   # other device, 1s
#
# Run inside the container where v4l2-ctl + the camera live.

DEV="${1:-/dev/video0}"
INTERVAL="${2:-0.5}"

watch -n "$INTERVAL" \
  "v4l2-ctl -d $DEV --list-ctrls | grep -E 'auto_exposure|exposure_time_absolute|exposure_dynamic_framerate'"
