import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node, PushRosNamespace

VIDEO_DEVICE = "/dev/video0"


def _yaml_framerate(config_path):
    """Read usb_cam's configured framerate from the yaml so the autotune's
    exposure cap tracks the ONE source of truth. No hardcoded fallback: if the
    yaml has no framerate, fail loudly rather than silently guess a value."""
    with open(config_path) as f:
        data = yaml.safe_load(f) or {}
    for node in data.values():
        params = (node or {}).get("ros__parameters", {})
        if "framerate" in params:
            return float(params["framerate"])
    raise RuntimeError(
        f"'framerate' not found in {config_path}; the camera_autotune exposure "
        f"cap is derived from it"
    )


# v4l2 controls applied with v4l2-ctl AFTER the node opens the device (see the
# OnProcessStart handler below). This is the reliable path on this cam: the
# image controls (brightness/contrast/saturation/sharpness/gain) set through
# usb_cam's yaml params are inconsistent -- some fall back to driver defaults
# (observed: yaml brightness=-64/contrast=16 but the device read 0/32), while
# others DO land and can land LATE, after a one-shot seed (observed 2026-07-09:
# yaml saturation=32 overwrote the t=2s seed's 64 -> near-B&W image). So the
# seed re-runs on EVERY usb_cam (re)start, delayed past device open, and the
# yaml mirrors these values so whichever write lands last gives the same image.
# Also covers the controls usb_cam has no params for at all
# (hue/gamma/backlight/power_line_frequency/exposure_dynamic_framerate).
# Tune live, then bake the value in here:
#   v4l2-ctl -d /dev/video0 -c gamma=120
# NOTE: exposure is NOT set here -- camera_autotune owns it. contrast is only
# honored while the autotune contrast trim is OFF (contrast_enabled: false);
# with the trim on, the node will drive contrast and override this seed.
V4L2_CONTROLS = {
    # image controls (usb_cam yaml params for these don't stick on this cam)
    "brightness": 0,  # -64..64, default 0
    "contrast": 32,  # 0..64, default 32; only sticks with the autotune trim OFF
    "saturation": 64,  # 0..128, default 64
    "sharpness": 6,  # 0..6, default 3
    "gain": 0,  # 0..100, default 0; camera_autotune does not touch gain
    # controls usb_cam has no params for at all
    "hue": 0,  # -40..40, default 0; small negative pulls green toward magenta
    "gamma": 100,  # 72..500, default 100
    "backlight_compensation": 0,  # 0..2, default 1; try 0 if highlights clip
    "power_line_frequency": 1,  # anti-flicker: 0 = off, 1 = 50 Hz, 2 = 60 Hz
    "exposure_dynamic_framerate": 0,  # auto-exposure only: 1 = AE may drop fps in the dark
}


def generate_launch_description():
    share = get_package_share_directory("claw_cam")
    config = os.path.join(share, "config", "usb_cam_test.yaml")
    autotune_config = os.path.join(share, "config", "camera_autotune.yaml")
    # Exposure cap = one frame period, derived from the yaml's framerate so it
    # never drifts from what usb_cam actually requests.
    framerate = _yaml_framerate(config)
    # Large-SHM FastDDS profile, loaded on the usb_cam WRITER process only, so a
    # full 1.44 MB raw frame fits one contiguous SHM buffer instead of dropping
    # to fragmented UDP (the ~8-9 Hz raw stall). See fastdds_claw_bigshm.xml.
    fastdds_profile = os.path.join(share, "config", "fastdds_claw_bigshm.xml")

    # Match the front_cam namespace/topics so the rest of the pipeline and viz
    # tooling can consume the test stream directly.
    launch_objects = [
        PushRosNamespace("/auv4/claw_cam"),
    ]

    # Claw cam (exploreHD) via the patched workspace usb_cam. All yaml params
    # tune live with `ros2 param set /auv4/claw_cam/usb_cam_node <param> <value>`.
    #
    # The yaml uses yuyv2rgb, the only mode that produces a correct image here:
    # color/image is a normal rgb8 raw image (~9fps at 1280x720 — USB 2.0 caps
    # uncompressed YUYV). MJPEG modes are broken on this cam (corrupt JPEG buffer),
    # so the compressed/hardware-decode route is unavailable until that usb_cam bug
    # is fixed; for higher fps with correct color, lower the resolution in the yaml
    # or put the camera on a USB 3.0 port.
    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        output="screen",
        parameters=[config],
        remappings=[
            ("image_raw", "color/image"),
            ("image_raw/compressed", "color/image/compressed"),
            ("camera_info", "color/camera_info"),
        ],
        # Big-SHM DDS profile on the WRITER only; subscribers stay on default
        # DDS and just mmap this segment (see fastdds_claw_bigshm.xml).
        additional_env={"FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile},
        # Revive usb_cam if it dies OR if the freeze watchdog (Mode 2)
        # kills it after a deadlock -- see the watchdog comment below.
        respawn=True,
        respawn_delay=2.0,
    )

    camera_nodes = [
        usb_cam_node,
        # Freeze watchdog. Root cause of the silent claw_cam freezes
        # (gdb-confirmed 2026-07-06): fastrtps 2.6.x AB-BA deadlock between the
        # flow-controller delivering a reliable sample and the EDP discovery
        # thread unpairing a departing subscriber; usb_cam's next publish()
        # then blocks forever (process alive, zero frames, no error). Triggered
        # by subscriber churn (foxglove drops, CLI tools exiting). Mitigated at
        # the source: the workspace usb_cam now publishes SensorDataQoS
        # (best-effort = stateless writer, out of the deadlock path). This
        # watchdog remains as the safety net: it DETECTS a frame gap, RECORDS
        # evidence (/proc syscall+wchan, strace, v4l2 state, kernel log) to
        # evidence_dir, and -- once restart_enabled is set -- SIGKILLs the
        # deadlocked node so the respawn above revives it (Mode 2). Leave
        # restart_enabled false to keep it detect-only (Mode 1). Started
        # immediately (not on a delay) so it watches from first frame,
        # independent of whether camera_autotune is running.
        Node(
            package="claw_cam",
            executable="camera_freeze_watchdog_node",
            name="camera_freeze_watchdog",
            output="screen",
            parameters=[
                {
                    "video_device": VIDEO_DEVICE,
                    "freeze_timeout_sec": 2.0,
                    "evidence_dir": "/tmp/claw_cam_freeze",
                    # Flip to True (or `ros2 param set`) for Mode-2 auto-restart
                    # once the QoS fix has been bench-verified.
                    "restart_enabled": False,
                }
            ],
            # Watches the compressed stream only -- proving JPEG frames still
            # flow is enough to detect the freeze, and keeps the watchdog off the
            # heavy raw topic (nothing here decodes pixels).
            remappings=[("image_raw/compressed", "color/image/compressed")],
        ),
        # Re-seed V4L2_CONTROLS 5s after EVERY usb_cam start (first launch and
        # each respawn): usb_cam writes its own yaml controls during device
        # open, so a seed fired too early gets overwritten (the near-B&W
        # saturation regression). 5s is comfortably past open/format
        # negotiation; exposure is untouched here so there is no fight with
        # camera_autotune.
        RegisterEventHandler(
            OnProcessStart(
                target_action=usb_cam_node,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[
                            ExecuteProcess(
                                cmd=[
                                    "v4l2-ctl",
                                    "-d",
                                    VIDEO_DEVICE,
                                    "-c",
                                    ",".join(
                                        f"{k}={v}" for k, v in V4L2_CONTROLS.items()
                                    ),
                                ],
                                name="v4l2_extra_controls",
                                output="screen",
                            ),
                        ],
                    ),
                ],
            ),
        ),
        # Closed-loop camera auto-tuner: measures the live image and drives the
        # camera's own v4l2 controls. Manual-only exposure on a fixed
        # mean-brightness target, with exposure capped to one frame period so it
        # can't starve framerate (min_fps read from the yaml above); gamma/
        # contrast/backlight are settled trims (default off). Once this runs it
        # OWNS those controls, so the static V4L2_CONTROLS above and the yaml's
        # manual exposure are just boot seeds it will override.
        # Started on a delay so the device is open before it reads/writes v4l2.
        # Live-tunable, e.g.:
        #   ros2 param set /auv4/claw_cam/camera_autotune target_mean 130
        #   ros2 param set /auv4/claw_cam/camera_autotune gamma_enabled false
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="claw_cam",
                    executable="camera_autotune_node",
                    name="camera_autotune",
                    output="screen",
                    parameters=[
                        autotune_config,
                        # Passed inline so they override the yaml: min_fps stays
                        # derived from usb_cam's framerate (one source of truth).
                        {"video_device": VIDEO_DEVICE, "min_fps": framerate},
                    ],
                    remappings=[
                        ("image_raw", "color/image"),
                        ("image_raw/compressed", "color/image/compressed"),
                    ],
                ),
            ],
        ),
    ]

    return LaunchDescription(launch_objects + camera_nodes)
