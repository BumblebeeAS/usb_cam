# Closed-loop camera auto-tuner for the claw_cam (usb_cam_test) bench.
#
# Reads the live image, measures its exposure/contrast, and drives the CAMERA's
# own v4l2 controls so downstream detectors get a consistently-exposed image as
# lighting changes -- no per-dive gamma/exposure tuning by hand.
#
# Design: MANUAL-ONLY. The camera's own auto-exposure is never trusted -- at
# startup we force auto_exposure to Manual and the node owns exposure from then
# on, so the image behaves predictably for the detectors (no camera AE hunting,
# no auto<->manual switching). A proportional loop drives mean brightness to a
# fixed target_mean, with exposure capped to protect framerate (see min_fps).
# gamma / contrast / backlight are SLOW secondary trims, applied only once
# exposure has settled, each with a dead-band + cooldown so nothing chatters.
#
# All controls are discovered at runtime from `v4l2-ctl --list-ctrls`, so
# missing / renamed controls (this cam has kernel-5.15 renamed UVC controls) are
# skipped rather than erroring, and every value is clamped to the device range.
#
# Bench tool: lives with usb_cam_test, NOT in the trash deployment. Everything
# is live-tunable, e.g.:
#   ros2 param set /auv4/claw_cam/camera_autotune target_mean 130
#   ros2 param set /auv4/claw_cam/camera_autotune gamma_enabled false

import re
import shutil
import subprocess

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

# UVC auto_exposure menu value (V4L2): 1 = Manual Mode. Manual-only, so this is
# the only mode used (3 = Aperture Priority / auto is never selected).
AE_MANUAL = 1


class CameraAutotune(Node):
    def __init__(self):
        super().__init__("camera_autotune")

        p = self.declare_parameter
        # --- device / cadence ---
        self.video_device = p("video_device", "/dev/video0").value
        self.control_period = p("control_period_sec", 0.5).value  # 2 Hz
        # Subscribe to the compressed (JPEG) stream instead of raw. The loop only
        # needs coarse brightness stats and downsamples anyway, so JPEG artifacts
        # are irrelevant; this avoids a second heavy raw subscriber (matters most
        # cross-host). Requires compressed_image_transport publishing the topic.
        # Defaults to compressed so nothing here falls back onto the raw stream.
        self.use_compressed = p("use_compressed", True).value

        # --- brightness setpoint (fixed-mean strategy) ---
        self.target_mean = p("target_mean", 120.0).value  # 0..255
        # Asymmetric brightness band around target_mean. Blown highlights are
        # unrecoverable while shadows can be lifted, so we bias toward the dark
        # side: tolerate the image being DARKER than target (wide low band ->
        # "allow darker") but react quickly once it goes BRIGHTER (narrow high
        # band -> "clip brighter"). Equal values reproduce the old symmetric band.
        self.mean_deadband_low = p("mean_deadband_low", 30.0).value  # below target: allowed
        self.mean_deadband_high = p("mean_deadband_high", 6.0).value  # above target: clip

        # --- exposure supervision ---
        self.exposure_enabled = p("exposure_enabled", True).value
        self.ae_ctrl = p("ae_ctrl_name", "auto_exposure").value
        self.exposure_ctrl = p("exposure_ctrl_name", "exposure_time_absolute").value
        self.exposure_step_frac = p("exposure_step_frac", 0.4).value  # max jump/tick
        # Highlight-priority guard. Global mean is blind to spatial layout: a
        # split scene (dark half + blown-out bright half) can average right onto
        # target while one side is whitewashed. So if more than this fraction of
        # pixels are saturating (clip_hi), pull exposure DOWN regardless of mean
        # -- protects highlights at the cost of the dark side (shadows recover,
        # clipped highlights don't). Set high (e.g. 1.0) to disable the guard.
        self.highlight_clip_max = p("highlight_clip_max", 0.02).value  # frac >= clip_hi_level allowed
        self.highlight_pull = p("highlight_pull", 0.15).value  # downward nudge/tick when over
        # Luma levels that count as "clipped". Defaults assume full-range 0..255
        # luma. The exploreHD's JPEG stream carries limited-range BT.601 Y
        # (16..235 -- measured 2026-07-09: p99.9 pinned at 235-236 with ~10% of
        # pixels AT the ceiling, nothing above JPEG ringing ~241), so with the
        # full-range defaults clip_hi/clip_lo read a permanent 0.0% and every
        # clip guard is silently dead. camera_autotune.yaml overrides these to
        # the real rail values for this cam.
        self.clip_hi_level = p("clip_hi_level", 250).value
        self.clip_lo_level = p("clip_lo_level", 5).value
        # fps enforcement: exposure time (100us units) can't exceed one frame
        # period, so the ceiling for F fps is 10000/F (e.g. 15fps -> ~667,
        # 30fps -> ~333, matching the yaml's "<=330 keeps 30fps" note). The
        # launch passes the camera's framerate here (read from the yaml, the
        # single source of truth); default 0 = no fps cap (nothing hardcoded).
        self.min_fps = p("min_fps", 0.0).value
        # Optional hard override in 100us units (0 = derive purely from min_fps).
        self.exposure_fps_cap = p("exposure_fps_cap", 0).value
        self.hunt_window = p("hunt_window", 10).value  # ticks of mean history
        self.hunt_std_thresh = p("hunt_std_thresh", 20.0).value  # AE-oscillation

        # --- brightness trim (anti-windup backup for the exposure loop) ---
        # Exposure owns the mean; brightness (an ADDITIVE offset) only acts when
        # exposure is railed (floored at min / capped) and still off-target, then
        # relaxes back toward 0 once exposure has room again -- so the two never
        # fight over the mean. This is the lever for "exposure floored at 1 but
        # the image is still over-bright" that exposure alone cannot fix.
        self.brightness_enabled = p("brightness_enabled", False).value
        self.brightness_ctrl = p("brightness_ctrl_name", "brightness").value
        self.brightness_step = p("brightness_step", 2).value  # offset/tick when assisting
        self.brightness_relax_step = p("brightness_relax_step", 1).value  # /tick back to 0

        # --- secondary trims (only when exposure settled) ---
        self.settle_hold_sec = p("settle_hold_sec", 3.0).value
        # KIV: exposure could instead maximize histogram spread; fixed-mean for now.
        self.spread_mode = p("exposure_spread_mode", False).value

        # Secondary trims default OFF: bench-verify the exposure loop alone
        # first, then enable each live (`ros2 param set ... gamma_enabled true`).
        self.contrast_enabled = p("contrast_enabled", False).value
        self.contrast_ctrl = p("contrast_ctrl_name", "contrast").value
        self.target_std = p("target_std", 50.0).value
        self.std_deadband = p("std_deadband", 8.0).value
        self.contrast_step = p("contrast_step", 4).value
        # Clip guard: raising contrast pushes highlights toward 255, which is how
        # the trim previously self-blew-out to contrast=max on a flat/hazy scene.
        # Refuse to RAISE contrast once clip_hi reaches this fraction of
        # highlight_clip_max (lowering is always allowed). 1.0 disables the guard.
        self.contrast_clip_guard = p("contrast_clip_guard", 0.5).value

        self.gamma_enabled = p("gamma_enabled", False).value
        self.gamma_ctrl = p("gamma_ctrl_name", "gamma").value
        self.shadow_percentile = p("shadow_percentile", 5.0).value
        self.shadow_target = p("shadow_target", 25.0).value  # lift crushed shadows
        self.shadow_deadband = p("shadow_deadband", 8.0).value
        self.gamma_step = p("gamma_step", 10).value

        self.backlight_enabled = p("backlight_enabled", False).value
        self.backlight_ctrl = p("backlight_ctrl_name", "backlight_compensation").value
        self.clip_hi_frac_thresh = p("clip_hi_frac_thresh", 0.05).value
        self.backlight_cooldown = p("backlight_cooldown_sec", 10.0).value

        self.bridge = CvBridge()
        self.controls = {}  # name -> {min,max,default,value,inactive}
        self.latest = None  # newest cv2 gray image
        self.mean_hist = []
        self._manual_ready = False  # camera forced to manual exposure yet?
        self._settled_since = None
        self._last_backlight = 0.0
        self._tick_count = 0
        # Re-verify the cam is still in manual every ~10s (at control_period).
        self._manual_recheck_ticks = max(1, int(10.0 / max(self.control_period, 0.05)))

        self.add_on_set_parameters_callback(self._on_set_parameters)
        if self.use_compressed:
            self.create_subscription(
                CompressedImage, "image_raw/compressed", self._on_compressed,
                qos_profile_sensor_data,
            )
        else:
            self.create_subscription(
                Image, "image_raw", self._on_image, qos_profile_sensor_data
            )
        self.timer = self.create_timer(self.control_period, self._tick)

        # The node drives the camera via v4l2-ctl; if it's not installed, it can
        # do nothing -- say so loudly rather than looping silently.
        if shutil.which("v4l2-ctl") is None:
            self.get_logger().error(
                "v4l2-ctl not on PATH -- install v4l2-utils in this environment; "
                "camera_autotune cannot control the camera without it"
            )

        self._discover_controls()
        self.get_logger().info(
            f"camera_autotune on {self.video_device}: target_mean={self.target_mean}, "
            f"controls found: {sorted(self.controls)}"
        )

    # ---- time helper (ROS clock; wall-clock funcs are unavailable in some ctx) ----
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---- v4l2-ctl plumbing --------------------------------------------------
    def _discover_controls(self):
        """Parse `v4l2-ctl --list-ctrls` into a name->range/value map. Controls
        not reported (missing / renamed) simply won't be tuned."""
        try:
            out = subprocess.run(
                ["v4l2-ctl", "-d", self.video_device, "--list-ctrls"],
                capture_output=True, text=True, timeout=5,
            ).stdout
        except Exception as e:
            self.get_logger().warn(f"v4l2-ctl list failed (retry): {e}")
            return
        line_re = re.compile(r"^\s*(\w+)\s+0x[0-9a-f]+\s+\((\w+)\)\s*:\s*(.*)$")
        found = {}
        for line in out.splitlines():
            m = line_re.match(line)
            if not m:
                continue
            name, _type, rest = m.groups()
            fields = dict(re.findall(r"(\w+)=(-?\d+)", rest))
            found[name] = {
                "min": int(fields.get("min", 0)),
                "max": int(fields.get("max", 0)),
                "default": int(fields.get("default", 0)),
                "value": int(fields.get("value", 0)),
                "inactive": "flags=inactive" in rest,
            }
        # Keep the previous cache if this read produced nothing (device briefly
        # unavailable / parse miss) rather than wiping a good map.
        if found:
            self.controls = found

    def _read(self, name):
        try:
            out = subprocess.run(
                ["v4l2-ctl", "-d", self.video_device, "-C", name],
                capture_output=True, text=True, timeout=5,
            ).stdout
            m = re.search(r":\s*(-?\d+)", out)
            return int(m.group(1)) if m else None
        except Exception:
            return None

    def _write(self, name, value, force=False):
        """Clamp to device range and set the control; returns applied value or None.
        force=True re-issues the write even if the cached value already matches --
        needed when the register READS the target but the driver hasn't committed
        it (e.g. the auto->manual exposure transition on this UVC cam)."""
        c = self.controls.get(name)
        if c is None:
            return None
        value = int(max(c["min"], min(c["max"], value)))
        if value == c["value"] and not force:
            return value  # no-op, avoid churn
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", self.video_device, "-c", f"{name}={value}"],
                capture_output=True, text=True, timeout=5, check=True,
            )
        except Exception as e:
            self.get_logger().warn(f"set {name}={value} failed: {e}")
            return None
        c["value"] = value
        self.get_logger().info(f"{name} -> {value}")
        return value

    # ---- image intake -------------------------------------------------------
    def _ingest(self, gray):
        # Downsample for cheap stats; exact pixels don't matter for control.
        if gray.shape[1] > 320:
            scale = 320.0 / gray.shape[1]
            gray = cv2.resize(gray, None, fx=scale, fy=scale)
        self.latest = gray

    def _on_image(self, msg: Image):
        try:
            if "mono" in msg.encoding or msg.encoding in ("8UC1", "mono8"):
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            self._ingest(gray)
        except Exception as e:
            self.get_logger().warn(f"decode failed: {e}", throttle_duration_sec=5.0)

    def _on_compressed(self, msg: CompressedImage):
        try:
            # JPEG straight to grayscale -- no color needed for brightness stats.
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise ValueError("imdecode returned None (bad/empty JPEG)")
            self._ingest(gray)
        except Exception as e:
            self.get_logger().warn(
                f"compressed decode failed: {e}", throttle_duration_sec=5.0
            )

    def _metrics(self, gray):
        lo, hi = np.percentile(gray, [self.shadow_percentile, 98.0])
        return {
            "mean": float(gray.mean()),
            "std": float(gray.std()),
            "p_lo": float(lo),
            "p_hi": float(hi),
            "clip_hi": float((gray >= self.clip_hi_level).mean()),
            "clip_lo": float((gray <= self.clip_lo_level).mean()),
        }

    # ---- control loop -------------------------------------------------------
    def _tick(self):
        if not self.controls:
            self._discover_controls()
            if not self.controls:
                self.get_logger().warn(
                    f"no v4l2 controls on {self.video_device} yet (device open?)",
                    throttle_duration_sec=5.0,
                )
            return
        # Manual-only: force the camera out of auto-exposure once, up front, so
        # it never runs its own AE loop (even before the first frame arrives).
        self._tick_count += 1
        # Periodically re-read the device so the cache (and the heartbeat) reflect
        # external `v4l2-ctl` changes, and the trims step from the real value
        # instead of a stale one. Same ~10s cadence as the manual-exposure recheck.
        if self.controls and self._tick_count % self._manual_recheck_ticks == 0:
            self._discover_controls()
        if self.exposure_enabled and self.ae_ctrl in self.controls:
            if not self._manual_ready:
                self._enter_manual()
                self._manual_ready = True
            # Re-assert periodically: something (usb_cam re-applying its yaml,
            # another tool) can flip the cam back to auto behind our back, which
            # would silently return control to the camera's own bad AE.
            elif self._tick_count % self._manual_recheck_ticks == 0:
                if self._read(self.ae_ctrl) != AE_MANUAL:
                    self.get_logger().warn("camera fell back to auto-exposure; re-asserting manual")
                    self._enter_manual()
        elif self.exposure_enabled:
            self.get_logger().warn(
                f"exposure control '{self.ae_ctrl}' not found -- discovered "
                f"{sorted(self.controls)}; set ae_ctrl_name/exposure_ctrl_name",
                throttle_duration_sec=10.0,
            )
        if self.latest is None:
            self.get_logger().warn(
                "no frames on image_raw yet (check topic remap / camera up)",
                throttle_duration_sec=5.0,
            )
            return
        m = self._metrics(self.latest)

        self.mean_hist.append(m["mean"])
        if len(self.mean_hist) > self.hunt_window:
            self.mean_hist.pop(0)
        hunting = (
            len(self.mean_hist) >= self.hunt_window
            and float(np.std(self.mean_hist)) > self.hunt_std_thresh
        )
        # Asymmetric: darker-than-target is tolerated further than brighter-than.
        err = m["mean"] - self.target_mean
        in_band = -self.mean_deadband_low <= err <= self.mean_deadband_high

        # Heartbeat so it's visible the loop is alive even when not writing.
        exp = self.controls.get(self.exposure_ctrl, {}).get("value")
        bright = self.controls.get(self.brightness_ctrl, {}).get("value")
        contrast = self.controls.get(self.contrast_ctrl, {}).get("value")
        self.get_logger().info(
            f"mean={m['mean']:.0f}/{self.target_mean:.0f} in_band={in_band} "
            f"exposure={exp} brightness={bright} contrast={contrast} "
            f"std={m['std']:.0f} clip_hi={m['clip_hi']*100:.1f}% hunting={hunting}",
            throttle_duration_sec=2.0,
        )

        if self.exposure_enabled and self.ae_ctrl in self.controls:
            self._run_exposure(m, in_band, hunting)
            # Brightness backs up exposure at the rails (runs AFTER exposure so it
            # sees the exposure value this tick already moved -- clean handoff).
            if self.brightness_enabled:
                self._run_brightness(m)

        # Secondary trims only once exposure is calm, so they don't chase AE.
        # Not "settled" while highlights are still clipping -- the exposure loop
        # is actively pulling down in that case.
        settled = in_band and not hunting and m["clip_hi"] <= self.highlight_clip_max
        now = self._now()
        if settled:
            self._settled_since = self._settled_since or now
        else:
            self._settled_since = None
        stable = (
            self._settled_since is not None
            and now - self._settled_since >= self.settle_hold_sec
        )
        if stable:
            if self.contrast_enabled:
                self._trim_contrast(m)
            if self.gamma_enabled:
                self._trim_gamma(m)
            if self.backlight_enabled:
                self._trim_backlight(m, now)

    def _exposure_cap(self):
        """Max exposure (100us units): device max, bounded by the fps ceiling
        (10000/min_fps) and any explicit exposure_fps_cap override."""
        cap = self.controls[self.exposure_ctrl]["max"]
        if self.min_fps > 0:
            cap = min(cap, int(10000 / self.min_fps))
        if self.exposure_fps_cap > 0:
            cap = min(cap, self.exposure_fps_cap)
        return cap

    def _run_exposure(self, m, in_band, hunting):
        # Manual-only: the node owns exposure; the camera's auto-exposure is
        # never used. Proportional loop drives mean brightness -> target_mean.
        # Highlight guard is a pull-DOWN only, and is gated to the bright side:
        # only engage it when the mean is already at/above target. If the mean is
        # BELOW target the scene is globally dark and the clipping is a localized
        # bright source (a light, a specular glint) that lowering exposure can't
        # fix -- cutting exposure would just crush the rest of the frame and, at
        # the floor, pin exposure at min forever. There, trust the mean and climb.
        # When mean >= target, clipping means genuine over-exposure worth pulling
        # down (also catches a split scene that averages on-target while its
        # bright half saturates).
        over_clip = (
            m["clip_hi"] > self.highlight_clip_max
            and m["mean"] >= self.target_mean
        )
        if in_band and not over_clip:
            return  # on target and highlights safe; skip the write to avoid churn
        cur = self.controls[self.exposure_ctrl]["value"] or self._read(
            self.exposure_ctrl
        )
        if not cur:
            return
        # Sensor brightness is ~linear in exposure time, so target/current is a
        # direct correction estimate; clamp the per-tick jump to avoid overshoot.
        ratio = self.target_mean / max(m["mean"], 1.0)
        if over_clip:
            # Highlights clipping: pull down regardless of what the mean wants.
            ratio = min(ratio, 1.0 - self.highlight_pull)
        ratio = max(1 - self.exposure_step_frac, min(1 + self.exposure_step_frac, ratio))
        new = int(round(cur * ratio))
        # Integer rounding stalls the multiplicative step at tiny exposures
        # (e.g. 1 * 1.4 -> 1), so force at least a unit step toward the target.
        if new == cur:
            new = cur + (1 if ratio > 1 else -1)
        self._write(self.exposure_ctrl, min(new, self._exposure_cap()))

    def _run_brightness(self, m):
        """Anti-windup backup for the exposure loop. Exposure owns the mean; the
        additive `brightness` offset only acts when exposure is RAILED and still
        off-target, then relaxes back toward 0 as soon as exposure has room --
        so the two never fight over the mean. Fixes "exposure floored at min but
        the image is still over-bright" (exposure can't go lower; pull brightness
        down instead), and the mirror case at the exposure cap."""
        c = self.controls.get(self.brightness_ctrl)
        ec = self.controls.get(self.exposure_ctrl)
        if c is None or ec is None:
            return
        exp = ec.get("value")
        if exp is None:
            return
        at_floor = exp <= ec["min"]
        at_cap = exp >= self._exposure_cap()
        err = m["mean"] - self.target_mean
        too_bright = err > self.mean_deadband_high
        too_dark = err < -self.mean_deadband_low
        cur = c["value"]
        if too_bright and at_floor:
            new = cur - self.brightness_step  # exposure can't go lower; darken via offset
        elif too_dark and at_cap:
            new = cur + self.brightness_step  # exposure maxed; brighten via offset
        elif at_floor or at_cap:
            # Railed but not fighting it (in-band, or railed the harmless way):
            # hold the offset -- relaxing here would just re-create the condition
            # exposure can't fix and oscillate.
            return
        else:
            # Exposure is inside its adjustable range -> it owns the mean; relax
            # brightness toward neutral so exposure reclaims full authority.
            if cur == 0:
                return
            step = min(self.brightness_relax_step, abs(cur))
            new = cur + (step if cur < 0 else -step)
        self._write(self.brightness_ctrl, new)

    def _enter_manual(self):
        """Force the camera into manual exposure and seed the loop from whatever
        value it currently holds. Called at startup (and re-asserted if the cam
        is ever found back in auto).

        Two UVC quirks are handled here, both of which caused a fresh launch to
        stay stuck in the camera's own (over-bright) auto-exposure while the node
        *thought* it was manual:
          1. The register can READ manual (value=1) before the driver has
             committed the transition, so the cache-guarded write no-ops and the
             switch never actually happens -> force=True.
          2. auto_exposure=1 alone doesn't always leave the AE loop; the driver
             keeps auto-exposing until exposure_time_absolute is written at least
             once. The proportional loop won't do that when it's floored (bright
             scene wants exposure below min), so commit one explicit exposure
             write here to guarantee manual actually engages."""
        self._write(self.ae_ctrl, AE_MANUAL, force=True)
        self.controls[self.exposure_ctrl]["inactive"] = False
        # Verify the switch really took; retry once before trusting it.
        if self._read(self.ae_ctrl) != AE_MANUAL:
            self._write(self.ae_ctrl, AE_MANUAL, force=True)
            if self._read(self.ae_ctrl) != AE_MANUAL:
                self.get_logger().warn(
                    f"{self.ae_ctrl} would not switch to manual ({AE_MANUAL}); "
                    f"camera may keep running its own auto-exposure"
                )
        start = self._read(self.exposure_ctrl)
        if not start:
            start = (
                self.controls[self.exposure_ctrl]["value"]
                or self.controls[self.exposure_ctrl]["default"]
                or self.controls[self.exposure_ctrl]["min"]
            )
        # Commit an explicit exposure write (force) so the driver leaves AE.
        self.controls[self.exposure_ctrl]["value"] = start
        self._write(self.exposure_ctrl, min(start, self._exposure_cap()), force=True)
        self.get_logger().info(
            "manual-only exposure: camera auto-exposure disabled"
        )

    def _trim_contrast(self, m):
        err = self.target_std - m["std"]
        if abs(err) <= self.std_deadband:
            return
        cur = self.controls.get(self.contrast_ctrl, {}).get("value")
        if cur is None:
            return
        raising = err > 0  # std below target -> want MORE contrast
        # Clip guard: raising contrast stretches highlights toward 255. If we're
        # already near the clip limit, refuse to raise -- this is exactly what let
        # the trim self-clip up to contrast=max before. Lowering is always safe.
        if raising and m["clip_hi"] >= self.contrast_clip_guard * self.highlight_clip_max:
            return
        self._write(self.contrast_ctrl, cur + (self.contrast_step if raising else -self.contrast_step))

    def _trim_gamma(self, m):
        # Higher gamma lifts shadows; only act on the shadow percentile so this
        # stays decoupled from exposure (which owns the mean).
        err = self.shadow_target - m["p_lo"]
        if abs(err) <= self.shadow_deadband:
            return
        cur = self.controls.get(self.gamma_ctrl, {}).get("value")
        if cur is None:
            return
        self._write(self.gamma_ctrl, cur + (self.gamma_step if err > 0 else -self.gamma_step))

    def _trim_backlight(self, m, now):
        if now - self._last_backlight < self.backlight_cooldown:
            return
        cur = self.controls.get(self.backlight_ctrl, {}).get("value")
        if cur is None:
            return
        # Blown highlights while the scene is otherwise dark => backlit; compensate.
        if m["clip_hi"] > self.clip_hi_frac_thresh and m["mean"] < self.target_mean:
            new = cur + 1
        elif m["clip_hi"] < self.clip_hi_frac_thresh:
            new = cur - 1  # relax toward default when not needed
        else:
            return
        if self._write(self.backlight_ctrl, new) is not None:
            self._last_backlight = now

    # ---- live param updates -------------------------------------------------
    def _on_set_parameters(self, params):
        for pr in params:
            if hasattr(self, pr.name):
                setattr(self, pr.name, pr.value)
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = CameraAutotune()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
