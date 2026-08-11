# Freeze watchdog for the claw_cam (usb_cam_test bench).
#
# ROOT CAUSE (gdb-confirmed live on the AUV, 2026-07-06): NOT the camera/USB.
# fastrtps 2.6.11 AB-BA deadlock inside the usb_cam process between two FastDDS
# threads: the flow-controller delivering a reliable sample (StatefulWriter::
# deliver_sample_nts, writer mutex held, waiting on the discovery lock) and the
# EDP discovery thread unpairing a DEPARTING SUBSCRIBER (EDP::unpairReaderProxy
# -> StatefulWriter::matched_reader_remove, discovery lock held, waiting on the
# writer mutex). The executor then blocks forever on the writer mutex at its
# next publish(): process alive, zero frames, nothing logged. Triggered by
# subscriber churn -- every foxglove drop / CLI tool exit is one race ticket.
# Mitigated in the workspace usb_cam by publishing SensorDataQoS (best-effort
# = stateless writer, no reliable-writer paths); this watchdog stays as the
# safety net for any residual freeze (rosout/parameter writers are still
# reliable) and for genuinely new failure modes.
#
# Mode 1 (default): DETECT the freeze and RECORD evidence; restart nothing.
# Mode 2 (restart_enabled=true): same, then SIGTERM->SIGKILL usb_cam and let
# the launch file's respawn revive it (the deadlocked executor cannot run its
# own shutdown path), followed by `fastdds shm clean` for the orphaned SHM.
#
# Evidence captured either way:
#   - a timestamped ERROR log line the instant frames stop, and
#   - an evidence file snapshotting WHY the usb_cam thread is stuck:
#       * strace of the usb_cam thread (select/poll/ioctl) if ptrace is allowed;
#       * a no-ptrace fallback -- per-thread /proc/<tid>/syscall + wchan, which
#         names the exact syscall the thread is blocked in even when strace is
#         denied (yama ptrace_scope) -- this is the decisive signal;
#       * v4l2-ctl --all (device state at freeze time);
#       * kernel log tail (best effort; usually empty for a soft stall).
#
# Lightweight by design: subscribes best-effort to the COMPRESSED (JPEG) stream
# and only timestamps arrivals (no decode, no cv_bridge) so it can't perturb the
# stream it is watching. Compressed is more than enough to prove liveness -- if
# JPEG frames keep arriving the camera isn't frozen -- and it keeps the watchdog
# off the heavy raw topic entirely (nothing here needs pixels).
#
# Live-tunable, e.g.:
#   ros2 param set /auv4/claw_cam/camera_freeze_watchdog freeze_timeout_sec 3.0

import os
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CameraFreezeWatchdog(Node):
    def __init__(self):
        super().__init__("camera_freeze_watchdog")

        p = self.declare_parameter
        # Gap (s) with no frames before we call it a freeze. At 15 fps a healthy
        # inter-frame gap is ~0.07s, so 2s is ~30 missed frames -- unambiguous.
        self.freeze_timeout_sec = p("freeze_timeout_sec", 2.0).value
        self.check_period_sec = p("check_period_sec", 0.5).value
        # Don't arm until we've actually seen a stream, so slow bringup isn't
        # mistaken for a freeze.
        self.min_frames_before_arm = p("min_frames_before_arm", 10).value

        # Evidence capture.
        self.video_device = p("video_device", "/dev/video0").value
        # Which process to strace: the usb_cam grab thread. Matched via pgrep.
        self.usb_cam_match = p("usb_cam_match", "usb_cam_node_exe").value
        self.strace_secs = p("strace_secs", 2).value
        # strace is OPT-IN. usb_cam (FastDDS) has hundreds of threads and
        # `strace -p` attaches to every one, so each capture emits hundreds of
        # yama "ptrace attach" lines into kern.log -- and with freezes flapping
        # on subscriber churn that floods the disk (600G incident, 2026-07-06).
        # The no-ptrace /proc syscall+wchan fallback is the decisive artifact
        # anyway, so strace stays off unless explicitly enabled for a debug run:
        #   ros2 param set .../camera_freeze_watchdog strace_enabled true
        self.strace_enabled = p("strace_enabled", False).value
        # Written inside THIS container. Bind-mounted workspace paths are visible
        # from the host too; /tmp is container-local but survives until restart.
        self.evidence_dir = p("evidence_dir", "/tmp/claw_cam_freeze").value

        # --- Mode 2: auto-restart (default OFF = Mode 1 detect-only) ---
        # ROOT CAUSE (gdb-confirmed 2026-07-06): fastrtps 2.6.x AB-BA deadlock
        # between the flow-controller delivering a reliable sample and the EDP
        # discovery thread unpairing a departing subscriber. usb_cam's next
        # publish() blocks forever on the writer mutex -- process alive, zero
        # frames, nothing logged. Triggered by subscriber churn (foxglove
        # drops, CLI tools exiting). The deadlocked executor cannot run its
        # SIGINT/SIGTERM shutdown path, so recovery is SIGTERM -> grace ->
        # SIGKILL; the launch file's respawn=True brings the node back, and a
        # `fastdds shm clean` sweeps the SHM segments the kill orphans.
        self.restart_enabled = p("restart_enabled", False).value
        self.kill_grace_sec = p("kill_grace_sec", 5.0).value

        os.makedirs(self.evidence_dir, exist_ok=True)

        # --- state ---
        self._last_rx = None      # seconds (ROS clock) of most recent frame
        self._frames = 0          # total frames seen since start
        self._frozen = False      # currently inside a detected freeze?
        self._freeze_since = None # ROS-clock seconds of the last good frame
        self._freeze_count = 0    # how many freeze events this run
        self._capturing = False   # an evidence capture thread is in flight

        # Watch the compressed stream only -- liveness proof, no raw subscriber.
        self.create_subscription(
            CompressedImage, "image_raw/compressed", self._on_image,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(self.check_period_sec, self._check)

        self.get_logger().info(
            f"freeze watchdog armed: timeout={self.freeze_timeout_sec}s, "
            f"device={self.video_device}, evidence_dir={self.evidence_dir} "
            f"(Mode 1: detect + record, NO restart)"
        )
        if self.strace_enabled and shutil.which("strace") is None:
            self.get_logger().warn(
                "strace_enabled=true but strace not on PATH -- syscall trace will "
                "be skipped (apt install strace). The no-ptrace /proc syscall+wchan "
                "fallback still runs."
            )

    def _now(self):
        # ROS clock (matches camera_autotune_node's convention here).
        return self.get_clock().now().nanoseconds / 1e9

    def _on_image(self, _msg):
        now = self._now()
        self._last_rx = now
        self._frames += 1
        if self._frozen:
            # Device self-healed on its own (rare in Mode 1 -- nothing restarts it).
            dur = now - self._freeze_since if self._freeze_since else 0.0
            self.get_logger().warn(
                f"claw_cam RECOVERED after {dur:.1f}s without intervention "
                f"(Mode 1 did not restart it)"
            )
            self._frozen = False
            self._freeze_since = None

    def _check(self):
        if self._last_rx is None or self._frames < self.min_frames_before_arm:
            return  # not armed yet
        now = self._now()
        gap = now - self._last_rx
        if not self._frozen and gap > self.freeze_timeout_sec:
            self._frozen = True
            self._freeze_since = self._last_rx
            self._freeze_count += 1
            mode = "restarting usb_cam (Mode 2)" if self.restart_enabled \
                else "camera left frozen (Mode 1)"
            self.get_logger().error(
                f"claw_cam FROZEN (freeze #{self._freeze_count}): no frames for "
                f"{gap:.1f}s after {self._frames} frames. Capturing evidence; "
                f"{mode}."
            )
            self._spawn_capture(gap)
        elif self._frozen:
            held = now - self._freeze_since if self._freeze_since else gap
            self.get_logger().error(
                f"claw_cam STILL frozen for {held:.0f}s",
                throttle_duration_sec=10.0,
            )

    # ---- evidence capture (runs off the executor thread) --------------------
    def _spawn_capture(self, gap):
        if self._capturing:
            return
        self._capturing = True
        threading.Thread(
            target=self._capture, args=(gap,), daemon=True
        ).start()

    def _run(self, cmd, timeout):
        try:
            r = subprocess.run(
                cmd, capture_output=True, text=True, timeout=timeout
            )
            return (r.stdout or "") + (r.stderr or "")
        except Exception as e:  # noqa: BLE001 - evidence tool, never let it raise
            return f"<{' '.join(cmd)} failed: {e}>"

    def _find_usb_cam_pid(self):
        out = self._run(["pgrep", "-f", self.usb_cam_match], timeout=5).split()
        return out[0] if out else None

    def _thread_diag(self, pid):
        """Per-thread syscall + wchan straight from /proc -- needs NO ptrace, so
        it works even when strace is denied. This is the decisive artifact: for
        the stuck thread it names the current syscall (select/ppoll/ioctl) and
        the kernel function it's parked in."""
        lines = []
        task_dir = f"/proc/{pid}/task"
        try:
            tids = sorted(os.listdir(task_dir))
        except Exception as e:  # noqa: BLE001
            return f"<cannot list {task_dir}: {e}>"
        for tid in tids:
            base = f"{task_dir}/{tid}"
            comm = self._read_proc(f"{base}/comm").strip()
            wchan = self._read_proc(f"{base}/wchan").strip()
            syscall = self._read_proc(f"{base}/syscall").strip()
            state = self._read_proc(f"{base}/status")
            st = next(
                (ln.split("\t", 1)[1].strip()
                 for ln in state.splitlines() if ln.startswith("State:")),
                "?",
            )
            lines.append(
                f"tid {tid} [{comm}] state={st} wchan={wchan or '-'} "
                f"syscall={syscall or '-'}"
            )
        return "\n".join(lines)

    def _read_proc(self, path):
        try:
            with open(path) as f:
                return f.read()
        except Exception as e:  # noqa: BLE001
            return f"<{path}: {e}>"

    def _kernel_tail(self):
        # Best effort: inside a container these are frequently unavailable.
        for cmd in (["journalctl", "-k", "-n", "40", "--no-pager"],
                    ["dmesg", "--ctime"]):
            if shutil.which(cmd[0]):
                out = self._run(cmd, timeout=5)
                if out and "failed:" not in out[:20] and "not permitted" not in out.lower():
                    return f"$ {' '.join(cmd)}\n{out[-4000:]}"
        return "<no kernel log accessible from this container>"

    def _capture(self, gap):
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(
            self.evidence_dir, f"freeze_{self._freeze_count:03d}_{ts}.txt"
        )
        try:
            with open(path, "w") as f:
                def w(s):
                    f.write(s + "\n")
                    f.flush()

                w("=== claw_cam FREEZE evidence (Mode 1) ===")
                w(f"wall_time: {ts}")
                w(f"gap_since_last_frame: {gap:.1f}s")
                w(f"frames_before_freeze: {self._frames}")

                pid = self._find_usb_cam_pid()
                w(f"usb_cam pid: {pid}")

                # No-ptrace fallback FIRST -- it is the reliable artifact.
                if pid:
                    w("\n--- per-thread syscall + wchan (/proc, no ptrace) ---")
                    w(self._thread_diag(pid))

                # strace -- richer, but OPT-IN: it ptrace-attaches to every
                # usb_cam thread (hundreds), flooding kern.log via yama. Off by
                # default; the /proc diag above is the decisive artifact regardless.
                if not self.strace_enabled:
                    w("\n--- strace skipped: strace_enabled=false (per-thread "
                      "/proc syscall+wchan above is the decisive signal; set "
                      "strace_enabled:=true for a one-off debug run) ---")
                elif pid and shutil.which("strace"):
                    w(f"\n--- strace select/poll/ioctl ({self.strace_secs}s) ---")
                    w(self._run(
                        ["strace", "-tt", "-e", "trace=select,pselect6,poll,ppoll,ioctl",
                         "-p", str(pid)],
                        timeout=self.strace_secs + 3,
                    ))
                elif not shutil.which("strace"):
                    w("\n--- strace skipped: not installed (apt install strace) ---")

                if shutil.which("v4l2-ctl"):
                    w("\n--- v4l2-ctl --all ---")
                    w(self._run(
                        ["v4l2-ctl", "-d", self.video_device, "--all"], timeout=5
                    ))

                w("\n--- kernel log tail ---")
                w(self._kernel_tail())

            self.get_logger().error(
                f"FREEZE evidence written: {path} (inside the "
                f"isaac_ros_dev container)"
            )
        except Exception as e:  # noqa: BLE001 - never let capture crash the node
            self.get_logger().error(f"evidence capture failed: {e}")
        finally:
            if self.restart_enabled:
                self._restart_usb_cam()
            self._capturing = False

    def _restart_usb_cam(self):
        """Mode 2: kill the deadlocked usb_cam; launch respawn revives it.
        Evidence is always captured BEFORE this runs (called from _capture)."""
        pid = self._find_usb_cam_pid()
        if not pid:
            self.get_logger().error("Mode 2: no usb_cam process found to restart")
            return
        # SIGTERM first for form's sake, but a deadlocked executor can't run
        # its shutdown path, so escalate to SIGKILL after the grace period.
        self.get_logger().error(f"Mode 2: SIGTERM usb_cam pid {pid}")
        self._run(["kill", "-TERM", pid], timeout=5)
        deadline = time.monotonic() + self.kill_grace_sec
        while time.monotonic() < deadline:
            if not os.path.exists(f"/proc/{pid}"):
                break
            time.sleep(0.2)
        if os.path.exists(f"/proc/{pid}"):
            self.get_logger().error(f"Mode 2: SIGKILL usb_cam pid {pid}")
            self._run(["kill", "-KILL", pid], timeout=5)
        # SIGKILL skips FastDDS SHM cleanup; sweep the orphaned segments so
        # they don't accumulate in /dev/shm (`fastdds shm clean` only removes
        # orphans, so it is safe while other stacks are up).
        if shutil.which("fastdds"):
            self._run(["fastdds", "shm", "clean"], timeout=15)
        self.get_logger().error(
            "Mode 2: usb_cam killed; launch respawn should bring it back"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CameraFreezeWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
