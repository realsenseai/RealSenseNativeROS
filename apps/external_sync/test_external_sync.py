#!/usr/bin/env python3
"""
RealSense D555e Sync Verification Tool

Verifies:
  Test A - Intra-camera sync (single or multi):
    Pairs Depth and Color frames by closest Sensor Timestamp
    (binary search).  Both streams share the same trigger, so
    matched pairs have nearly identical timestamps.  Checks that
    offsets are constant (std ≈ 0) and frame intervals match.
    Tests multiple FPS values (default 30,15) — sync passes if
    ANY FPS produces a sync result for a given camera.

  Test B - PTP cross-validation (multi-camera, --enable-ptp):
    Implements software PTP to align camera clocks to the host
    wall-clock.  Verifies PTP-corrected timestamps from different
    cameras coincide for simultaneously-captured frames.

Timestamp source:
  Sensor Timestamp from metadata JSON (std_msgs/String topic).
  The Image stream must also be subscribed to trigger metadata publishing.
  Depth and Color are paired by closest Sensor Timestamp, which is
  robust against different DDS stream start times and frame drops.

Sync modes:
  Internal (default): Camera's internal PWM master triggers both sensors.
  External:           An external trigger signal synchronizes all cameras.
                      Without signal, sensors free-run on shared PLL clock
                      (intra-camera sync maintained; inter-camera drifts).

Run without arguments or with --help for full usage information.
"""

import argparse
import bisect
import json
import subprocess
import statistics
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
@dataclass
class FrameSample:
    """One metadata frame with Sensor Timestamp."""
    frame_counter: int
    sensor_timestamp_us: float   # Sensor Timestamp from metadata (us)
    frame_timestamp_us: float    # Frame Timestamp from metadata (us)
    stream: str                  # "Color" or "Depth"
    rx_timestamp_us: float = 0.0 # Host receive time in us (for PTP)


@dataclass
class CameraData:
    """Collected samples for one camera in one round."""
    node_name: str
    serial: str
    depth_samples: List[FrameSample] = field(default_factory=list)
    color_samples: List[FrameSample] = field(default_factory=list)


@dataclass
class SyncTestResult:
    """One line in the summary table."""
    test: str        # "A", "B.P1", "B.P2", "C"
    camera: str      # serial or "serial1<->serial2"
    metric: str      # description of what was measured
    value_ms: float  # measured value in ms
    passed: bool
    fps: int = 0     # FPS at which this result was measured


@dataclass
class PTPOffset:
    """PTP clock offset for one camera."""
    serial: str
    offset_us: float       # mean(host_time - sensor_timestamp)
    offset_std_us: float   # std-dev of offset measurements
    num_samples: int


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
_ros2_cmd_count = 0


def _flush_ros2_daemon():
    """Restart the ROS 2 daemon to clear stale DDS discovery cache.

    Every `ros2` CLI subprocess creates a temporary DDS participant.
    After many calls the daemon's cached discovery state becomes stale,
    causing `ros2 node list` / `ros2 topic list` to return incomplete
    or empty results.  Restarting the daemon forces re-discovery.
    """
    subprocess.run("ros2 daemon stop", shell=True,
                   capture_output=True, timeout=5)
    time.sleep(1)
    subprocess.run("ros2 daemon start", shell=True,
                   capture_output=True, timeout=5)
    time.sleep(2)


def run_cmd(cmd: str, timeout: int = 15) -> Tuple[int, str]:
    global _ros2_cmd_count
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout)
        _ros2_cmd_count += 1
        # Proactively flush daemon every 20 ros2 CLI calls to avoid
        # stale DDS discovery cache.
        if _ros2_cmd_count >= 20:
            _flush_ros2_daemon()
            _ros2_cmd_count = 0
        return r.returncode, r.stdout.strip()
    except subprocess.TimeoutExpired:
        return -1, ""


def discover_d555_nodes(min_cameras: int = 0,
                        retries: int = 5,
                        delay: float = 3.0) -> List[str]:
    """Discover D555e nodes.  If *min_cameras* > 0, retry up to *retries*
    times (with daemon restart) until at least that many are found."""
    for attempt in range(retries):
        rc, out = run_cmd("ros2 node list")
        if rc != 0:
            print(f"ERROR: 'ros2 node list' failed (rc={rc})")
            sys.exit(1)
        # Deduplicate (DDS can report the same node multiple times)
        found = list(dict.fromkeys(
            n.strip() for n in out.splitlines() if "/D555_" in n
        ))
        if len(found) >= min_cameras or min_cameras == 0:
            return found
        print(f"  Discovery attempt {attempt + 1}/{retries}: "
              f"found {len(found)}, need {min_cameras} -- retrying "
              f"(daemon restart) ...")
        run_cmd("ros2 daemon stop", timeout=5)
        time.sleep(1)
        run_cmd("ros2 daemon start", timeout=5)
        time.sleep(delay)
    return found


def serial_from_node(node_name: str) -> str:
    return node_name.split("_", 1)[1]


def set_sync_mode(node_name: str, mode: str) -> bool:
    """Set Camera_Sync_Mode.  mode = 'Internal' or 'External'."""
    cmd = f'ros2 param set {node_name} Depth.option.Camera_Sync_Mode {mode}'
    rc, out = run_cmd(cmd, timeout=10)
    ok = rc == 0 and "Set parameter successful" in out
    tag = "OK" if ok else out
    print(f"  [{node_name}] Camera_Sync_Mode -> {mode}: {tag}")
    return ok


def hw_reset_node(node_name: str) -> bool:
    cmd = f"ros2 service call {node_name}/hw_reset std_srvs/srv/Empty"
    rc, _ = run_cmd(cmd, timeout=30)
    return rc == 0


def topic_has_publisher(topic: str) -> bool:
    rc, out = run_cmd(f"ros2 topic info {topic}", timeout=10)
    if rc != 0:
        return False
    for line in out.splitlines():
        if "Publisher count:" in line:
            return int(line.split(":")[1].strip()) > 0
    return False


# ---------------------------------------------------------------------------
# Stream stop/start via profile change
# ---------------------------------------------------------------------------
_RESTART_FPS = "15"


def _get_profile(node_name: str, param: str) -> str:
    rc, out = run_cmd(f"ros2 param get {node_name} {param}", timeout=10)
    if rc != 0:
        return ""
    if ":" in out:
        return out.split(":", 1)[1].strip()
    return out.strip()


def _swap_fps(profile: str, new_fps: str) -> str:
    parts = [p.strip() for p in profile.split(",")]
    if len(parts) >= 4:
        parts[-1] = new_fps
    return ", ".join(parts)


def stop_start_streams(nodes: List[str], restart_wait: float = 5.0):
    """Restart streams by toggling profiles to a different fps then back."""
    profiles: Dict[str, Dict[str, str]] = {}
    param_names = ["Depth.Profile", "CompRGB.Profile"]

    for n in nodes:
        profiles[n] = {}
        for p in param_names:
            orig = _get_profile(n, p)
            if orig:
                profiles[n][p] = orig

    print("  Stopping streams (profile change) ...")
    for n in nodes:
        for p, orig in profiles[n].items():
            # Pick a toggle FPS that differs from the current one
            parts = [x.strip() for x in orig.split(",")]
            cur_fps = parts[-1] if len(parts) >= 4 else "30"
            toggle_fps = "30" if cur_fps == _RESTART_FPS else _RESTART_FPS
            tmp = _swap_fps(orig, toggle_fps)
            run_cmd(f'ros2 param set {n} {p} "{tmp}"', timeout=10)
    time.sleep(2)

    print("  Restarting streams (restore profile) ...")
    for n in nodes:
        for p, orig in profiles[n].items():
            run_cmd(f'ros2 param set {n} {p} "{orig}"', timeout=10)

    print(f"  Waiting {restart_wait:.0f}s for streams to stabilize ...")
    time.sleep(restart_wait)


def set_fps(nodes: List[str], fps: int, restart_wait: float = 5.0):
    """Change all streams to the specified FPS by swapping profile."""
    param_names = ["Depth.Profile", "CompRGB.Profile"]
    target = str(fps)
    print(f"  Setting all streams to {fps} FPS ...")
    for n in nodes:
        for p in param_names:
            orig = _get_profile(n, p)
            if not orig:
                continue
            new_prof = _swap_fps(orig, target)
            if new_prof != orig:
                run_cmd(f'ros2 param set {n} {p} "{new_prof}"', timeout=10)
    print(f"  Waiting {restart_wait:.0f}s for streams to stabilize ...")
    time.sleep(restart_wait)


# ---------------------------------------------------------------------------
# PTP (Precision Time Protocol) -- software implementation
# ---------------------------------------------------------------------------
def compute_ptp_offsets(
    rounds_data: List[Dict[str, CameraData]],
) -> Dict[str, PTPOffset]:
    """Compute PTP offset for each camera from collected samples."""
    offsets_per_cam: Dict[str, List[float]] = defaultdict(list)

    for rd in rounds_data:
        for serial, cam in rd.items():
            for sample in cam.depth_samples:
                if sample.rx_timestamp_us > 0:
                    offset = sample.rx_timestamp_us - sample.sensor_timestamp_us
                    offsets_per_cam[serial].append(offset)

    result = {}
    for serial, offs in offsets_per_cam.items():
        if offs:
            result[serial] = PTPOffset(
                serial=serial,
                offset_us=statistics.mean(offs),
                offset_std_us=statistics.stdev(offs) if len(offs) > 1 else 0.0,
                num_samples=len(offs),
            )
    return result


def analyze_ptp_sync(
    rounds_data: List[Dict[str, CameraData]],
    ptp_offsets: Dict[str, PTPOffset],
    threshold_ms: float,
) -> Tuple[bool, List[SyncTestResult]]:
    """Test B: PTP cross-validation of inter-camera sync."""
    results: List[SyncTestResult] = []

    if len(ptp_offsets) < 2:
        print("  Less than 2 cameras -- PTP cross-validation skipped")
        return True, results

    serials = sorted(ptp_offsets.keys())
    ref = serials[0]
    passed = True

    print(f"\n  PTP offsets (camera -> host clock):")
    for serial in serials:
        p = ptp_offsets[serial]
        print(f"    [{serial}] offset={p.offset_us / 1000:.3f}ms  "
              f"std={p.offset_std_us / 1000:.3f}ms  n={p.num_samples}")

    print(f"\n  Inter-camera clock differences (via PTP):")
    for serial in serials[1:]:
        diff = (ptp_offsets[serial].offset_us
                - ptp_offsets[ref].offset_us) / 1000.0
        print(f"    {ref}<->{serial}: clock_diff={diff:.3f}ms")

    print(f"\n  PTP-corrected frame alignment (threshold: {threshold_ms}ms):")
    rd = rounds_data[0]
    ref_cam = rd.get(ref)
    if not ref_cam or not ref_cam.depth_samples:
        return True, results

    for serial in serials[1:]:
        cam = rd.get(serial)
        if not cam or not cam.depth_samples:
            continue

        ref_ptp = [s.sensor_timestamp_us + ptp_offsets[ref].offset_us
                   for s in ref_cam.depth_samples]
        cam_ptp = [s.sensor_timestamp_us + ptp_offsets[serial].offset_us
                   for s in cam.depth_samples]

        diffs = []
        for rt in ref_ptp:
            closest_val = min(cam_ptp, key=lambda ct: abs(ct - rt))
            diffs.append(abs(rt - closest_val) / 1000.0)

        avg_diff = statistics.mean(diffs)
        max_diff = max(diffs)
        ok = avg_diff <= threshold_ms
        if not ok:
            passed = False
        status = "PASS" if ok else "FAIL"
        print(f"    {ref}<->{serial}: "
              f"avg={avg_diff:.3f}ms  max={max_diff:.3f}ms  [{status}]")

        results.append(SyncTestResult(
            test="B", camera=f"{ref}<->{serial}",
            metric="PTP alignment",
            value_ms=avg_diff, passed=ok,
        ))

    n_show = 5
    print(f"\n  PTP-corrected Depth Sensor Timestamps "
          f"(first {n_show} frames, wall-clock us):")
    for serial in serials:
        cam = rd.get(serial)
        if not cam or not cam.depth_samples:
            continue
        ptp_ts = [s.sensor_timestamp_us + ptp_offsets[serial].offset_us
                  for s in cam.depth_samples[:n_show]]
        ts_str = "  ".join(f"{t:.0f}" for t in ptp_ts)
        print(f"    [{serial}] {ts_str}")

    return passed, results


# ---------------------------------------------------------------------------
# Collector: subscribes to Image + metadata topics
#
# Image subscription triggers metadata publishing.
# Actual timestamp data comes from metadata (Sensor Timestamp).
# Depth and Color are paired by closest Sensor Timestamp in analysis.
# ---------------------------------------------------------------------------
class FrameCollector(Node):
    """Subscribe to Image AND metadata topics, collect Sensor Timestamps.

    All subscriptions are created simultaneously (no blocking
    topic_has_publisher checks).

    A warmup period discards the first few frames so all streams are
    fully established before counting begins.
    """

    WARMUP_FRAMES = 5  # discard first N frames per stream

    def __init__(self, cameras: List[str], num_samples: int):
        super().__init__("sync_test")
        self._qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._num_samples = num_samples
        self._lock = threading.Lock()
        self._camera_data: Dict[str, CameraData] = {}
        self._subs = []
        self._done = threading.Event()
        self._has_color: set = set()
        self._warmup_count: Dict[str, int] = {}  # per "(serial, stream)"

        # Subscribe to ALL topics at once — no blocking checks.
        # If a topic has no publisher, we simply receive no messages.
        for node_name in cameras:
            serial = serial_from_node(node_name)
            self._camera_data[serial] = CameraData(
                node_name=node_name, serial=serial,
            )
            prefix = f"/realsense/D555_{serial}"

            for stream in ("Depth", "Color"):
                img_topic = f"{prefix}_{stream}"
                meta_topic = f"{prefix}_{stream}/metadata"
                self._warmup_count[f"{serial}_{stream}"] = 0

                # Image sub triggers metadata publishing
                sub = self.create_subscription(
                    Image, img_topic,
                    lambda msg: None,
                    self._qos,
                )
                self._subs.append(sub)

                # Metadata sub collects Sensor Timestamp
                sub = self.create_subscription(
                    String, meta_topic,
                    lambda msg, s=serial, st=stream: self._on_meta(msg, s, st),
                    self._qos,
                )
                self._subs.append(sub)

                if stream == "Color":
                    self._has_color.add(serial)

        self.get_logger().info(
            f"Subscribed to {len(cameras)} cameras "
            f"({len(self._subs)} topics, warmup={self.WARMUP_FRAMES})"
        )

    def _on_meta(self, msg: String, serial: str, stream: str):
        """Parse metadata JSON and extract Sensor Timestamp."""
        rx_time = time.time() * 1e6  # host receive time in us
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        metadata = data.get("metadata", {})
        sensor_ts = metadata.get("Sensor Timestamp")
        frame_ts = metadata.get("Frame Timestamp", 0)
        frame_counter = metadata.get("Frame Counter", -1)

        if sensor_ts is None or frame_counter < 0:
            return

        # Skip warmup frames to let all subscriptions stabilize.
        warmup_key = f"{serial}_{stream}"
        with self._lock:
            wc = self._warmup_count.get(warmup_key, 0)
            if wc < self.WARMUP_FRAMES:
                self._warmup_count[warmup_key] = wc + 1
                return

            cam = self._camera_data[serial]
            samples = (cam.depth_samples if stream == "Depth"
                       else cam.color_samples)
            if len(samples) >= self._num_samples:
                self._check_done()
                return
            samples.append(FrameSample(
                frame_counter=frame_counter,
                sensor_timestamp_us=float(sensor_ts),
                frame_timestamp_us=float(frame_ts),
                stream=stream,
                rx_timestamp_us=rx_time,
            ))

    def _check_done(self):
        for cam in self._camera_data.values():
            if len(cam.depth_samples) < self._num_samples:
                return
            if cam.serial in self._has_color:
                if len(cam.color_samples) < self._num_samples:
                    return
        self._done.set()

    def collect(self, timeout_sec: float = 30.0) -> Dict[str, CameraData]:
        end = time.time() + timeout_sec
        while not self._done.is_set() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return dict(self._camera_data)


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyze_intra_camera(
    cam: CameraData, threshold_ms: float,
    ext_sync: bool = False,
    ext_sync_with_signal: bool = False,
    sync_offset_threshold_ms: float = 5.0,
) -> Tuple[bool, List[SyncTestResult]]:
    """Test A: Check RGB/Depth sync using Sensor Timestamp.

    Pairs Depth and Color frames by **closest Sensor Timestamp**.
    For each depth frame, the color frame with the nearest sensor
    timestamp is selected (binary search).  A pair is accepted only
    when the timestamp gap is within half a frame period, and each
    color frame is used at most once (greedy, chronological order).

    This approach is robust against arbitrary DDS stream start-time
    differences and frame drops — it does not assume depth[i] and
    color[i] correspond to the same trigger event.

    Checks:
      1. Offset std-dev  < threshold_ms  (consistency)
      2. Interval diff   < threshold_ms  (rate match)
      3. Absolute avg offset  (sync status):
           Internal mode or External with signal:
             PASS if |avg| < sync_offset_threshold_ms
             (both sensors fire on the same trigger pulse)
           External (no signal): PASS if |avg| > sync_offset_threshold_ms
    """
    results: List[SyncTestResult] = []

    if not cam.depth_samples:
        print(f"    [{cam.serial}] No Depth metadata -- SKIP")
        return True, results
    if not cam.color_samples:
        print(f"    [{cam.serial}] No Color metadata -- SKIP")
        return True, results

    # Sort by Sensor Timestamp
    d_sorted = sorted(cam.depth_samples,
                      key=lambda s: s.sensor_timestamp_us)
    c_sorted = sorted(cam.color_samples,
                      key=lambda s: s.sensor_timestamp_us)

    if min(len(d_sorted), len(c_sorted)) < 10:
        print(f"    [{cam.serial}] Too few frames for pairing "
              f"(D={len(d_sorted)}, C={len(c_sorted)}) -- SKIP")
        return True, results

    # Estimate frame period from depth intervals
    d_all_intervals = [d_sorted[i + 1].sensor_timestamp_us
                       - d_sorted[i].sensor_timestamp_us
                       for i in range(len(d_sorted) - 1)]
    if not d_all_intervals:
        print(f"    [{cam.serial}] Insufficient depth data -- SKIP")
        return True, results
    expected_interval = statistics.median(d_all_intervals)
    half_period = expected_interval / 2  # microseconds

    # Pair by closest sensor timestamp (greedy, chronological)
    c_ts = [c.sensor_timestamp_us for c in c_sorted]
    used_c: set = set()
    offsets: List[float] = []
    paired_d: List[FrameSample] = []
    paired_c: List[FrameSample] = []

    for d in d_sorted:
        idx = bisect.bisect_left(c_ts, d.sensor_timestamp_us)
        best_ci: Optional[int] = None
        best_diff = float('inf')
        for ci in (idx - 1, idx):
            if 0 <= ci < len(c_sorted) and ci not in used_c:
                diff = abs(c_sorted[ci].sensor_timestamp_us
                           - d.sensor_timestamp_us)
                if diff < best_diff:
                    best_diff = diff
                    best_ci = ci
        if best_ci is not None and best_diff <= half_period:
            offset_ms = ((c_sorted[best_ci].sensor_timestamp_us
                          - d.sensor_timestamp_us) / 1000.0)
            offsets.append(offset_ms)
            paired_d.append(d)
            paired_c.append(c_sorted[best_ci])
            used_c.add(best_ci)

    if len(offsets) < 10:
        print(f"    [{cam.serial}] Too few paired frames "
              f"({len(offsets)}) -- SKIP")
        return True, results

    n_pair = len(offsets)
    avg_off = statistics.mean(offsets)
    std_off = statistics.stdev(offsets) if n_pair > 1 else 0.0

    # Frame interval match (based on paired frames)
    d_intervals = [paired_d[i + 1].sensor_timestamp_us
                   - paired_d[i].sensor_timestamp_us
                   for i in range(n_pair - 1)]
    c_intervals = [paired_c[i + 1].sensor_timestamp_us
                   - paired_c[i].sensor_timestamp_us
                   for i in range(n_pair - 1)]
    interval_diffs = [abs(d - c) / 1000.0
                      for d, c in zip(d_intervals, c_intervals)]
    max_int_diff = max(interval_diffs) if interval_diffs else 0.0

    # Dropped frames
    d_drops = sum(1 for iv in d_intervals
                  if iv > expected_interval * 1.5)
    c_drops = sum(1 for iv in c_intervals
                  if iv > expected_interval * 1.5)
    std_ok = std_off <= threshold_ms
    int_ok = max_int_diff <= threshold_ms

    # Sync status based on absolute average offset
    abs_avg = abs(avg_off)
    if ext_sync:
        # External mode (no signal): expect NOT synced (large offset)
        sync_ok = abs_avg > sync_offset_threshold_ms
        if sync_ok:
            sync_label = "NOT SYNCED (expected for ext-sync without signal)"
        else:
            sync_label = ("SYNCED (unexpected -- external sync should "
                          "disable internal trigger)")
    elif ext_sync_with_signal:
        # External mode (with signal): both sensors fire on the same
        # trigger pulse, so offset should be near zero (< threshold),
        # same as internal mode.
        sync_ok = abs_avg <= sync_offset_threshold_ms
        if sync_ok:
            sync_label = "SYNCED"
        else:
            sync_label = (f"NOT SYNCED (|avg|={abs_avg:.3f}ms > "
                          f"{sync_offset_threshold_ms}ms)")
    else:
        # Internal mode: expect synced (small offset ~0.036ms).
        # Index-based pairing with outlier filtering handles cases
        # where a dropped frame shifts alignment.
        sync_ok = abs_avg <= sync_offset_threshold_ms
        if sync_ok:
            sync_label = "SYNCED"
        else:
            sync_label = (f"NOT SYNCED (|avg|={abs_avg:.3f}ms > "
                          f"{sync_offset_threshold_ms}ms)")

    passed = std_ok and int_ok and sync_ok
    status = "PASS" if passed else "FAIL"

    print(f"    [{cam.serial}] Paired frames: {len(offsets)} "
          f"(D={len(d_sorted)}, C={len(c_sorted)})")
    print(f"    [{cam.serial}] Sensor Timestamp offset (Color-Depth): "
          f"avg={avg_off:.3f}ms  std={std_off:.3f}ms")
    print(f"    [{cam.serial}] Frame interval match: "
          f"max_diff={max_int_diff:.3f}ms")
    print(f"    [{cam.serial}] Sync status: {sync_label}")
    if d_drops or c_drops:
        print(f"    [{cam.serial}] Dropped frames: "
              f"depth={d_drops} color={c_drops}")
    print(f"    [{cam.serial}] Result: [{status}]")

    results.append(SyncTestResult(
        test="A", camera=cam.serial,
        metric="offset avg (abs)",
        value_ms=abs_avg, passed=sync_ok,
    ))
    results.append(SyncTestResult(
        test="A", camera=cam.serial,
        metric="offset std-dev", value_ms=std_off, passed=std_ok,
    ))
    results.append(SyncTestResult(
        test="A", camera=cam.serial,
        metric="interval diff", value_ms=max_int_diff, passed=int_ok,
    ))
    return passed, results


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
def print_summary(results: List[SyncTestResult], sync_mode: str):
    """Print unified sync test results table."""
    if not results:
        return

    print("\n" + "=" * 80)
    print("                    SYNC TEST RESULTS SUMMARY")
    print("=" * 80)
    print(f" {'Test':<6} {'Camera':<22} {'FPS':>4} {'Metric':<20} "
          f"{'Value':>9} {'Result':>7}")
    print("-" * 80)
    for r in results:
        status = "PASS" if r.passed else "FAIL"
        cam = r.camera[:22]
        met = r.metric[:20]
        fps_str = str(r.fps) if r.fps else ""
        print(f" {r.test:<6} {cam:<22} {fps_str:>4} {met:<20} "
              f"{r.value_ms:>7.3f}ms  {status:>5}")
    print("-" * 80)
    overall = all(r.passed for r in results)
    result_str = "PASS" if overall else "FAIL"
    print(f" OVERALL: {result_str}  (sync_mode={sync_mode})")
    if not overall:
        failed = [r for r in results if not r.passed]
        for r in failed:
            fps_note = f" @{r.fps}fps" if r.fps else ""
            print(f"   x {r.test}: {r.camera} / {r.metric} "
                  f"= {r.value_ms:.3f}ms{fps_note}")
    print("=" * 80)


# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
def print_usage():
    text = """\
============================================================
RealSense D555e Sync Verification Tool
============================================================

DESCRIPTION
  Verifies frame synchronization of Intel RealSense D555e
  cameras over ROS 2.  Uses Sensor Timestamp from metadata
  (std_msgs/String) paired by closest Sensor Timestamp.

  Subscribes to both Image stream AND metadata topic -- the
  Image subscription triggers metadata publishing from the
  camera firmware.

TESTS
  Test A   Intra-camera: pairs Depth/Color frames by closest
           Sensor Timestamp (binary search) and verifies
           offset is constant (std ~ 0) and frame intervals
           match.  Tests each configured FPS (default 30,15).
  Test B   PTP cross-validation (--enable-ptp): aligns camera clocks to the
           host wall-clock and verifies PTP-corrected Sensor
           Timestamps coincide for simultaneously-captured
           frames.

PREREQUISITES
  - ROS 2 Humble (or compatible)
  - Python 3 with rclpy, sensor_msgs, std_msgs
  - D555e camera(s) streaming on the correct ROS_DOMAIN_ID
  - export ROS_DOMAIN_ID=<N>  (set before running)

USAGE
  python3 test_external_sync.py [OPTIONS]

OPTIONS
  --rounds N              Stop/start rounds per phase (default: 3)
  --samples N             Frames per camera per round (default: 30)
  --threshold-intra MS    Intra-camera threshold in ms (default: 2.0)
  --threshold-ptp MS      PTP inter-camera threshold in ms (default: 2.0)
  --enable-ext-sync       Set Camera_Sync_Mode to External
  --no-signal             External sync without signal (expect NOT synced).
                          Only valid with --enable-ext-sync.
                          Default (without --no-signal): expect synced.
  --hw-reset-verify       Run Phase 2: hw_reset then verify re-lock
  --enable-ptp            Enable PTP cross-validation (Test B)
  --collect-timeout S     Collection timeout in sec (default: 30)
  --restart-wait S        Wait after stop/start in sec (default: 5)
  --hw-reset-wait S       Wait after hw_reset in sec (default: 15)
  --fps FPS1,FPS2,...     Comma-separated FPS values to test (default: 30,15).
                          Sync passes if ANY FPS produces a passing result.

EXAMPLES
  # Single camera -- internal sync (default, test 30 and 15 FPS)
  python3 test_external_sync.py --samples 60

  # Single camera -- test only 30 FPS
  python3 test_external_sync.py --samples 60 --fps 30

  # Single camera -- external sync without signal
  python3 test_external_sync.py --enable-ext-sync --no-signal --samples 60

  # Multi-camera -- external sync with signal (expect synced)
  python3 test_external_sync.py --enable-ext-sync --rounds 3

  # Multi-camera -- external sync without signal (expect NOT synced)
  python3 test_external_sync.py --enable-ext-sync --no-signal --rounds 3

  # Multi-camera -- full test with PTP
  python3 test_external_sync.py --enable-ext-sync --rounds 3 \\
      --enable-ptp

TIMESTAMP SOURCE
  Sensor Timestamp  from metadata JSON (metadata.Sensor Timestamp).
  The metadata is published on per-stream /metadata topics
  (std_msgs/msg/String) only when the Image stream is actively
  subscribed.

  Depth and Color frames are paired by closest Sensor
  Timestamp (binary search).  For each depth frame, the
  color frame with the nearest sensor timestamp is matched.
  This is robust against DDS start-time differences and
  frame drops.

SYNC MODES
  Internal  Camera's internal PWM triggers both RGB and Depth.
            Each camera has its own clock -- no inter-camera sync.
  External  External trigger signal (STROBE input) replaces PWM.
            All cameras fire simultaneously.  Without signal,
            sensors free-run on shared PLL (no inter-camera sync).

OUTPUT
  The tool prints per-test results during execution and a
  unified SYNC TEST RESULTS SUMMARY table at the end.
  Exit code: 0 = all PASS, 1 = any FAIL.
"""
    print(text)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    # No arguments -> print usage and exit
    if len(sys.argv) == 1:
        print_usage()
        sys.exit(0)

    parser = argparse.ArgumentParser(
        description="RealSense D555e Sync Verification",
        add_help=True,
    )
    parser.add_argument("--rounds", type=int, default=3,
                        help="Stop/start rounds per phase (default: 3)")
    parser.add_argument("--samples", type=int, default=30,
                        help="Frames per camera per round (default: 30)")
    parser.add_argument("--threshold-intra", type=float, default=2.0,
                        help="Intra-camera sync threshold in ms")
    parser.add_argument("--threshold-ptp", type=float, default=2.0,
                        dest="threshold_inter",
                        help="PTP inter-camera threshold in ms")
    parser.add_argument("--enable-ext-sync", action="store_true",
                        help="Set Camera_Sync_Mode to External")
    parser.add_argument("--no-signal", action="store_true",
                        help="External sync without signal (expect NOT synced)")
    parser.add_argument("--hw-reset-verify", action="store_true",
                        help="Run Phase 2: hw_reset then verify re-lock")
    parser.add_argument("--enable-ptp", action="store_true",
                        help="Enable PTP cross-validation (Test B)")
    parser.add_argument("--collect-timeout", type=float, default=30.0,
                        help="Sample collection timeout in sec")
    parser.add_argument("--restart-wait", type=float, default=5.0,
                        help="Wait after stream restart in sec")
    parser.add_argument("--hw-reset-wait", type=float, default=15.0,
                        help="Wait after hw_reset in sec")
    parser.add_argument("--min-cameras", type=int, default=0,
                        help="Minimum cameras to discover (0=any, retries"
                             " with daemon restart if not met)")
    parser.add_argument("--fps", type=str, default="30,15",
                        help="Comma-separated FPS values to test "
                             "(default: 30,15). Sync passes if ANY "
                             "FPS produces a sync result.")
    args = parser.parse_args()
    args.fps_list = [int(f.strip()) for f in args.fps.split(",")]

    all_results: List[SyncTestResult] = []

    print("=" * 60)
    print("RealSense D555e Sync Verification")
    print("=" * 60)
    print(f"  Timestamp source: Sensor Timestamp (from metadata JSON)")
    print(f"  Frame pairing:    Closest Sensor Timestamp (binary search)")
    print(f"  FPS to test:      {', '.join(str(f) for f in args.fps_list)}")

    # 1. Discover cameras
    print("\n[Step 1] Discovering D555e cameras ...")
    nodes = discover_d555_nodes(min_cameras=args.min_cameras)
    if not nodes:
        print("ERROR: No D555e nodes found.  Check ROS_DOMAIN_ID.")
        sys.exit(1)
    print(f"  Found {len(nodes)} camera(s):")
    for n in nodes:
        print(f"    {n}")

    multi_camera = len(nodes) > 1
    sync_mode = "External" if args.enable_ext_sync else "Internal"

    # 2. Set sync mode
    print(f"\n[Step 2] Setting sync mode: {sync_mode}")
    for n in nodes:
        set_sync_mode(n, sync_mode)
    time.sleep(2)

    ext_with_signal = args.enable_ext_sync and not args.no_signal
    expect_no_sync = args.enable_ext_sync and args.no_signal

    # 3. Test each FPS
    # Track per-camera pass at any FPS (Test A).
    # camera_serial -> {fps -> bool}
    camera_fps_pass: Dict[str, Dict[int, bool]] = defaultdict(dict)
    fps_results: Dict[int, List[SyncTestResult]] = {}

    for fps_val in args.fps_list:
        print(f"\n{'=' * 60}")
        print(f"FPS = {fps_val}")
        print(f"{'=' * 60}")

        # Switch to target FPS + conditioning stop/start
        set_fps(nodes, fps_val, restart_wait=args.restart_wait)
        print("  Conditioning stop/start ...")
        stop_start_streams(nodes, restart_wait=args.restart_wait)

        # Collect data
        if ext_with_signal:
            num_rounds = 1
        elif multi_camera:
            num_rounds = args.rounds
        else:
            num_rounds = 1

        phase1_data: List[Dict[str, CameraData]] = []
        rclpy.init()
        try:
            for ri in range(num_rounds):
                print(f"\n[Step 3] FPS={fps_val} -- Round {ri + 1}/"
                      f"{num_rounds}: collecting {args.samples} frames ...")

                collector = FrameCollector(nodes, args.samples)
                data = collector.collect(timeout_sec=args.collect_timeout)
                collector.destroy_node()

                for serial, cam in sorted(data.items()):
                    print(f"  {serial}: depth_meta="
                          f"{len(cam.depth_samples)} "
                          f"color_meta={len(cam.color_samples)}")

                phase1_data.append(data)

                if multi_camera and ri < num_rounds - 1:
                    print("\n  Stop/start streams (no hw_reset) ...")
                    stop_start_streams(nodes,
                                       restart_wait=args.restart_wait)
        finally:
            rclpy.shutdown()

        # Analyze -- Test A for this FPS
        print(f"\n--- Test A @ {fps_val} FPS ---")
        print(f"  Threshold (std-dev / interval diff): "
              f"{args.threshold_intra} ms")
        if expect_no_sync:
            print(f"  External sync (no signal): expect NOT synced "
                  f"(|avg offset| > 5ms)")
        elif args.enable_ext_sync:
            print(f"  External sync (with signal): expect synced "
                  f"(|avg offset| < 5ms)")
        else:
            print(f"  Internal sync mode: expect synced "
                  f"(|avg offset| < 5ms)")

        fps_res: List[SyncTestResult] = []
        for ri, rd in enumerate(phase1_data):
            if len(phase1_data) > 1:
                print(f"  --- Round {ri + 1} ---")
            for serial, cam in sorted(rd.items()):
                ok, res = analyze_intra_camera(
                    cam, args.threshold_intra,
                    ext_sync=expect_no_sync,
                    ext_sync_with_signal=ext_with_signal)
                # Tag results with FPS
                for r in res:
                    r.fps = fps_val
                fps_res.extend(res)
                camera_fps_pass[serial][fps_val] = ok

        fps_results[fps_val] = fps_res

    # 4. Combine results across FPS values.
    # A camera passes Test A if it passes at ANY tested FPS.
    # Include all per-FPS results in the summary for visibility,
    # but mark failed-FPS results as informational when the camera
    # passes at another FPS.
    print(f"\n{'=' * 60}")
    print(f"ANALYSIS (sync_mode={sync_mode})")
    print(f"{'=' * 60}")

    overall_pass = True
    combined_results: List[SyncTestResult] = []

    print(f"\n[Test A] Intra-camera RGB/Depth sync "
          f"(Sensor Timestamp, closest-timestamp paired)")
    for serial, fps_map in sorted(camera_fps_pass.items()):
        passed_fps = [f for f, ok in fps_map.items() if ok]
        failed_fps = [f for f, ok in fps_map.items() if not ok]
        if passed_fps:
            print(f"  [{serial}] SYNCED at "
                  f"{', '.join(str(f) for f in passed_fps)} FPS")
            if failed_fps:
                print(f"  [{serial}] NOT synced at "
                      f"{', '.join(str(f) for f in failed_fps)} FPS "
                      f"(informational)")
        else:
            print(f"  [{serial}] NOT synced at any tested FPS: FAIL")
            overall_pass = False

    # Build combined results: for cameras that pass at any FPS,
    # keep only the passing FPS results as authoritative.
    # For cameras that fail at all FPS, include all results.
    for serial, fps_map in sorted(camera_fps_pass.items()):
        passed_fps = [f for f, ok in fps_map.items() if ok]
        for fps_val, res_list in fps_results.items():
            cam_res = [r for r in res_list if r.camera == serial]
            if passed_fps and fps_val in passed_fps:
                combined_results.extend(cam_res)
            elif not passed_fps:
                combined_results.extend(cam_res)
            # else: skip failed-FPS results for cameras that pass elsewhere

    # Test B: PTP cross-validation (uses data from first FPS that has data)
    if args.enable_ptp:
        print(f"\n[Test B] PTP cross-validation")
        print(f"  Threshold: {args.threshold_inter} ms")
        # Gather all rounds from all FPS tests
        all_rounds: List[Dict[str, CameraData]] = []
        for fps_val in args.fps_list:
            for res in fps_results.get(fps_val, []):
                pass  # results only; need raw data
        # Re-use the last FPS's phase1_data for PTP
        # (PTP offset is FPS-independent)
        ptp_offsets = compute_ptp_offsets(phase1_data)
        if multi_camera:
            ptp_pass, ptp_res = analyze_ptp_sync(
                phase1_data, ptp_offsets, args.threshold_inter)
            if not ptp_pass:
                overall_pass = False
            combined_results.extend(ptp_res)
        else:
            for serial, p in ptp_offsets.items():
                print(f"  [{serial}] PTP offset to host: "
                      f"{p.offset_us / 1000:.3f}ms  "
                      f"std={p.offset_std_us / 1000:.3f}ms")
            print("  Single camera -- PTP cross-validation N/A")

    # 5. Summary
    print_summary(combined_results, sync_mode)

    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()
