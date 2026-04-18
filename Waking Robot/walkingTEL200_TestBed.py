"""Part 1 testbed for TEL200 walking robot.

This module builds motion primitives, runs the required Part 1 tests,
and exports trajectory plots plus numeric pose errors.
"""

from pathlib import Path
import csv
import logging
import sys
import time
import random

import matplotlib.pyplot as plt
import numpy as np
from math import pi
from roboticstoolbox import ERobot, ET, mstraj, rtb_load_matfile, PRMPlanner
from roboticstoolbox.backends.PyPlot import PyPlot
from spatialgeometry import Cuboid
from spatialmath import SE3

#=============================================================
# Globals
#=============================================================

# Global simulation handles used only when render=True.
env = None
legs = None
body = None
leg_local_offsets = None


MM = 0.001
# Geometry and kinematics.
L = 200 * MM
L1 = 100 * MM
L2 = 100 * MM
W = 100 * MM
PHASE_FLIPS = (False, False, True, True)
PHASE_OFFSETS = (0, 100, 200, 300)

# Gait and primitive definitions.
FORWARD_DISTANCE_M = 0.10
TURN_ANGLE_DEG = 1.0

ROBOT_ROT_SPEED = 10.0   # deg/s
ROBOT_SPEED = 0.10       # m/s

# Viewer frame and camera behavior.
CAMERA_FOLLOW = True
CAMERA_HALF_WIDTH_X = 0.30
CAMERA_HALF_WIDTH_Y = 0.30
CAMERA_Z_MAX = 0.10
CAMERA_Z_MIN = -0.20

# Runtime defaults. Update these constants directly before running.
DT = 0.02
RENDER = False

CATCHUP_MIN_RENDER_DETAIL = 0.005
CATCHUP_MIN_STEP_DT = 0.001
CATCHUP_RENDER_DETAIL = 0.20

DEBUG_MODE = False
HOLD_WINDOW = False
LOG_PRIMITIVE_DETAILS = False
REALTIME_CATCHUP = True
REALTIME_LAG_TOL_S = 0.05

SHOW_PROGRESS = False
SIM_DETAIL = 1.0         # 0-1: 1 renders all frames, 0.5 renders ~half.
SIM_SPEED = 5.0          # viewer playback multiplier


#=============================================================
# Logger
#=============================================================

LOGGER = logging.getLogger("walking_testbed")


def setup_logging(debug=False):
    """Configure the module logger.

    Args:
        debug: If True, use DEBUG level; otherwise INFO.

    Returns:
        None.
    """
    level = logging.DEBUG if debug else logging.INFO

    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s | %(message)s",
        datefmt="%H:%M:%S",
    )

    if not LOGGER.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        LOGGER.addHandler(handler)
    else:
        for handler in LOGGER.handlers:
            handler.setFormatter(formatter)

    LOGGER.setLevel(level)
    LOGGER.propagate = False

    # Keep third-party logs quiet.
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    logging.getLogger("PIL").setLevel(logging.WARNING)


class SimpleProgressBar:
    """Minimal terminal progress bar without external dependencies."""

    def __init__(self, total, prefix="", enabled=True):
        """Initialize progress bar state.

        Args:
            total: Total number of expected steps.
            prefix: Label displayed before the bar.
            enabled: If False, disables all terminal output.
        """
        self.total = max(1, int(total))
        self.prefix = prefix
        self.enabled = enabled
        self.current = 0
        self.last_percent = -1
        if self.enabled:
            self._render()

    def update(self, step=1):
        """Advance the progress bar.

        Args:
            step: Number of steps to increment.

        Returns:
            None.
        """
        self.current = min(self.total, self.current + step)
        if not self.enabled:
            return
        percent = int((self.current / self.total) * 100)
        if percent != self.last_percent or self.current == self.total:
            self.last_percent = percent
            self._render()

    def _render(self):
        """Render the progress line in-place."""
        bar_width = 30
        filled = int(bar_width * self.current / self.total)
        bar = "#" * filled + "-" * (bar_width - filled)
        sys.stdout.write(
            f"\r{self.prefix} [{bar}] {self.current}/{self.total} ({100.0 * self.current / self.total:5.1f}%)"
        )
        sys.stdout.flush()

    def close(self):
        """Close the bar with a trailing newline."""
        if self.enabled:
            sys.stdout.write("\n")
            sys.stdout.flush()


#=============================================================
# Gait And Kinematic Model
#=============================================================

def gait(cycle, k, offset, flip):
    """Return one leg joint sample from a cyclic gait table.

    Args:
        cycle: Joint-space gait cycle with shape (N, 3).
        k: Base step index in the cycle.
        offset: Phase offset in samples.
        flip: If True, mirror the first joint for opposite-side legs.

    Returns:
        np.ndarray: Joint vector with shape (3,).
    """
    k = (k + offset) % cycle.shape[0]
    q = cycle[k, :].copy()
    if flip:
        q[0] = -q[0]
    return q


def build_leg_model():
    """Create the assignment baseline 3-DOF leg model.

    Returns:
        ERobot: Leg kinematic chain.
    """
    return ERobot(ET.Rz() * ET.Rx() * ET.ty(-L1) * ET.Rx() * ET.tz(-L2))


def build_gait_cycle(leg):
    """Generate a Cartesian gait cycle and solve it to joint space.

    Args:
        leg: ERobot leg model used for inverse kinematics.

    Returns:
        tuple:
            qcycle: Joint trajectory with shape (N, 3).
            xcycle: Cartesian trajectory with shape (N, 3).
            zu_m: Swing-foot z height in meters.
            zd_m: Support-foot z height in meters.

    Raises:
        RuntimeError: If IK fails for any gait sample.
    """
    xf = 50
    xb = -xf
    y = -50
    zu = -20
    zd = -50
    segments = np.array(
        [
            [xf, y, zd],
            [xb, y, zd],
            [xb, y, zu],
            [xf, y, zu],
            [xf, y, zd],
        ]
    ) * MM

    x = mstraj(segments, tsegment=[3, 0.25, 0.5, 0.25], dt=0.01, tacc=0.07)
    xcycle = x.q
    xcycle = np.vstack((xcycle, xcycle[-3:, :]))
    LOGGER.debug("Built Cartesian gait cycle with shape %s", xcycle.shape)

    # Solve IK sequentially so each step stays on the same kinematic branch.
    # This avoids abrupt joint flips between consecutive samples.
    qcycle = np.zeros((xcycle.shape[0], 3), dtype=float)
    q_prev = np.zeros(3, dtype=float)

    for i, p in enumerate(xcycle):
        sol = leg.ikine_LM(SE3(p[0], p[1], p[2]), q0=q_prev, mask=[1, 1, 1, 0, 0, 0])
        if hasattr(sol, "success") and not sol.success:
            raise RuntimeError(f"IK failed at gait step {i}")

        q = np.asarray(sol.q, dtype=float).reshape(3)
        q = q_prev + np.arctan2(np.sin(q - q_prev), np.cos(q - q_prev))
        qcycle[i, :] = q
        q_prev = q

    return qcycle, xcycle, zu * MM, zd * MM


#=============================================================
# Rendering And Scene Helpers
#=============================================================

def start_robot_environment(initial_pose):
    """Create and launch the interactive robot visualization scene.

    Args:
        initial_pose: Initial robot body pose [x, y, theta].

    Returns:
        None.
    """
    global env, legs, body, leg_local_offsets

    leg = build_leg_model()
    legs = [ERobot(leg, name=f"leg{i}") for i in range(4)]

    env = PyPlot()
    env.launch(limits=[-0.4, 0.4, -0.4, 0.4, CAMERA_Z_MIN, CAMERA_Z_MAX])

    leg_adjustment = SE3.Rz(pi)
    leg_local_offsets = [
        SE3(L / 2, -W / 2, 0),
        SE3(-L / 2, -W / 2, 0),
        SE3(L / 2, W / 2, 0) * leg_adjustment,
        SE3(-L / 2, W / 2, 0) * leg_adjustment,
    ]

    for i, robot_leg in enumerate(legs):
        robot_leg.q = np.r_[0, 0, 0]
        robot_leg.base = pose_to_se3(initial_pose) * leg_local_offsets[i]
        env.add(robot_leg, readonly=True, jointaxes=False, eeframe=False, shadow=False)

    body = Cuboid([L, W, 30 * MM], color="b")
    body.base = pose_to_se3(initial_pose)
    env.add(body)

    # Keep proportional 3D scaling so link lengths are visually correct.
    if hasattr(env, "ax") and hasattr(env.ax, "set_box_aspect"):
        env.ax.set_box_aspect((1.0, 1.0, 1.0))

    update_camera(initial_pose)
    env.step()


def stop_robot_environment():
    """Close interactive figures and clear rendering handles.

    Returns:
        None.
    """
    global env, legs, body, leg_local_offsets
    env = None
    legs = None
    body = None
    leg_local_offsets = None
    plt.close("all")


def pose_to_se3(pose):
    """Convert planar pose to SE3 transform.

    Args:
        pose: Pose [x, y, theta].

    Returns:
        SE3: transform in world frame.
    """
    return SE3(pose[0], pose[1], 0) * SE3.Rz(pose[2])


def set_render_state(pose, leg_joint_angles):
    """Update rendered body and leg states.

    Args:
        pose: Body pose [x, y, theta] in world frame.
        leg_joint_angles: Joint angles for all four legs, shape (4, 3).

    Returns:
        None.
    """
    if env is None:
        return

    robot_pose = pose_to_se3(pose)
    body.base = robot_pose

    for i, robot_leg in enumerate(legs):
        robot_leg.base = robot_pose * leg_local_offsets[i]
        robot_leg.q = leg_joint_angles[i]

    update_camera(pose)


def update_camera(pose):
    """Center the camera around the robot pose.

    Args:
        pose: Body pose [x, y, theta].

    Returns:
        None.
    """
    if env is None or not CAMERA_FOLLOW:
        return

    if not hasattr(env, "ax"):
        return

    x, y, _ = pose
    env.ax.set_xlim(x - CAMERA_HALF_WIDTH_X, x + CAMERA_HALF_WIDTH_X)
    env.ax.set_ylim(y - CAMERA_HALF_WIDTH_Y, y + CAMERA_HALF_WIDTH_Y)
    env.ax.set_zlim(CAMERA_Z_MIN, CAMERA_Z_MAX)


#=============================================================
# Primitive Construction And Validation
#=============================================================

def support_leg_count(xcycle, gait_step_index, support_threshold_z):
    """Estimate support-leg count at one gait sample.

    Args:
        xcycle: Cartesian gait cycle with shape (N, 3).
        gait_step_index: Current gait index.
        support_threshold_z: Z threshold between support and swing.

    Returns:
        int: Estimated number of support legs.
    """
    count = 0
    cycle_len = xcycle.shape[0]
    for phase in PHASE_OFFSETS:
        idx = (gait_step_index + phase) % cycle_len
        if xcycle[idx, 2] <= support_threshold_z:
            count += 1
    return count


def build_joint_sequence(qcycle, steps, start_index=0):
    """Build four-leg joint command sequence from a single-leg cycle.

    Args:
        qcycle: Single-leg cycle, shape (N, 3).
        steps: Number of output steps.
        start_index: Starting index into qcycle.

    Returns:
        np.ndarray: Joint command tensor with shape (steps, 4, 3).
    """
    seq = np.zeros((steps, 4, 3), dtype=float)
    cycle_len = qcycle.shape[0]

    for s in range(steps):
        gait_idx = (start_index + s) % cycle_len
        for leg_i, (phase, flip) in enumerate(zip(PHASE_OFFSETS, PHASE_FLIPS)):
            seq[s, leg_i, :] = gait(qcycle, gait_idx, phase, flip)

    return seq


def validate_support_constraint(xcycle, steps, start_index, support_threshold_z):
    """Compute minimum support-leg count across a window.

    Args:
        xcycle: Cartesian gait cycle with shape (N, 3).
        steps: Number of steps to evaluate.
        start_index: Initial gait index.
        support_threshold_z: Z support threshold.

    Returns:
        int: Minimum support-leg count over the window.
    """
    min_support = 4
    cycle_len = xcycle.shape[0]
    for s in range(steps):
        gait_idx = (start_index + s) % cycle_len
        support = support_leg_count(xcycle, gait_idx, support_threshold_z)
        min_support = min(min_support, support)
    return min_support


def create_motion_primitives(primitives_dir):
    """Build and save assignment primitives in local joint-space.

    Files generated:
    - forward_10cm.npz
    - backward_10cm.npz
    - turn_1deg_ccw.npz
    - turn_1deg_cw.npz

    Args:
        primitives_dir: Directory to store generated primitive .npz files.

    Returns:
        dict: Primitive metadata and command arrays keyed by name.

    Raises:
        RuntimeError: If support-legs minimum falls below assignment limit.
    """
    primitives_dir.mkdir(parents=True, exist_ok=True)

    leg = build_leg_model()
    qcycle, xcycle, zu_m, zd_m = build_gait_cycle(leg)

    # One full gait cycle gives smooth cyclic boundaries for repeated use.
    primitive_steps = qcycle.shape[0]
    support_threshold_z = (zu_m + zd_m) / 2.0

    min_support = validate_support_constraint(
        xcycle, primitive_steps, start_index=0, support_threshold_z=support_threshold_z
    )
    if min_support < 3:
        raise RuntimeError(
            f"Support-legs constraint violated. Minimum support legs: {min_support}"
        )

    joint_sequence = build_joint_sequence(qcycle, primitive_steps, start_index=0)

    forward_step = np.array([FORWARD_DISTANCE_M / primitive_steps, 0.0, 0.0], dtype=float)
    turn_step_ccw = np.array(
        [0.0, 0.0, np.deg2rad(TURN_ANGLE_DEG) / primitive_steps], dtype=float
    )
    turn_step_cw = np.array(
        [0.0, 0.0, -np.deg2rad(TURN_ANGLE_DEG) / primitive_steps], dtype=float
    )
    backward_step = np.array([-FORWARD_DISTANCE_M / primitive_steps, 0.0, 0.0], dtype=float)

    np.savez(
        primitives_dir / "forward_10cm.npz",
        joint_sequence=joint_sequence,
        body_local_step=forward_step,
        primitive_name="forward_10cm",
    )
    np.savez(
        primitives_dir / "backward_10cm.npz",
        joint_sequence=joint_sequence,
        body_local_step=backward_step,
        primitive_name="backward_10cm",
    )
    np.savez(
        primitives_dir / "turn_1deg_ccw.npz",
        joint_sequence=joint_sequence,
        body_local_step=turn_step_ccw,
        primitive_name="turn_1deg_ccw",
    )
    np.savez(
        primitives_dir / "turn_1deg_cw.npz",
        joint_sequence=joint_sequence,
        body_local_step=turn_step_cw,
        primitive_name="turn_1deg_cw",
    )

    LOGGER.info(
        "Saved motion primitives in %s (steps=%d, min_support_legs=%d)",
        primitives_dir,
        primitive_steps,
        min_support,
    )

    return {
        "forward_10cm": {
            "joint_sequence": joint_sequence,
            "body_local_step": forward_step,
            "steps": primitive_steps,
            "min_support_legs": min_support,
        },
        "turn_1deg_ccw": {
            "joint_sequence": joint_sequence,
            "body_local_step": turn_step_ccw,
            "steps": primitive_steps,
            "min_support_legs": min_support,
        },
        "turn_1deg_cw": {
            "joint_sequence": joint_sequence,
            "body_local_step": turn_step_cw,
            "steps": primitive_steps,
            "min_support_legs": min_support,
        },
        "backward_10cm": {
            "joint_sequence": joint_sequence,
            "body_local_step": backward_step,
            "steps": primitive_steps,
            "min_support_legs": min_support,
        },
    }


#=============================================================
# Primitive Execution And Timing
#=============================================================

def apply_local_body_step(pose, body_local_step):
    """Apply one local body increment to a world-frame pose.

    Args:
        pose: Current pose [x, y, theta].
        body_local_step: Local increment [dx_local, dy_local, dtheta].

    Returns:
        np.ndarray: Updated pose in world frame.
    """
    x, y, theta = pose
    dx_local, dy_local, dtheta = body_local_step

    dx_global = dx_local * np.cos(theta) - dy_local * np.sin(theta)
    dy_global = dx_local * np.sin(theta) + dy_local * np.cos(theta)
    return np.array([x + dx_global, y + dy_global, theta + dtheta], dtype=float)


def primitive_step_sim_dt(primitive):
    """Compute simulated time per primitive step.

    Translational components are constrained by ROBOT_SPEED (m/s).
    Rotational components are constrained by ROBOT_ROT_SPEED (deg/s).
    If a primitive has both translation and rotation, the slower
    (larger dt) constraint is used.

    Args:
        primitive: Primitive dictionary containing body_local_step.

    Returns:
        float: Simulated seconds per primitive step.
    """
    step_local = np.asarray(primitive["body_local_step"], dtype=float)
    step_distance = float(np.linalg.norm(step_local[:2]))
    step_angle_deg = float(abs(np.rad2deg(step_local[2])))

    dt_candidates = []
    if step_distance > 0:
        dt_candidates.append(step_distance / ROBOT_SPEED)

    if step_angle_deg > 0:
        dt_candidates.append(step_angle_deg / ROBOT_ROT_SPEED)

    if dt_candidates:
        return max(dt_candidates)

    return DT


def primitive_step_render_dt(primitive):
    """Compute viewer step duration from simulated duration and SIM_SPEED.

    Args:
        primitive: Primitive dictionary containing body_local_step.

    Returns:
        float: Viewer step duration in seconds.
    """
    return primitive_step_sim_dt(primitive) / SIM_SPEED


def execute_primitive(
    pose,
    primitive,
    repeats=1,
    render=False,
    primitive_name="primitive",
    show_progress=True,
):
    """Execute a primitive repeatedly and return final pose plus trajectory.

    Args:
        pose: Initial pose [x, y, theta].
        primitive: Primitive dictionary with joint and body-step data.
        repeats: Number of primitive repeats.
        render: If True, update active visualizer each rendered step.
        primitive_name: Name used in logs/progress.
        show_progress: If True, show terminal progress bar.

    Returns:
        tuple:
            pose: Final pose after execution.
            trajectory: Pose history array with shape (M, 3).
    """
    pose = np.array(pose, dtype=float)
    trajectory = [pose.copy()]
    body_step = primitive["body_local_step"]

    # Playback dt in viewer from physical step distance and configured speeds.
    sim_dt = primitive_step_sim_dt(primitive)
    render_dt = primitive_step_render_dt(primitive)
    detail = min(max(float(SIM_DETAIL), 0.0), 1.0)
    primitive_log_level = logging.INFO if LOG_PRIMITIVE_DETAILS else logging.DEBUG

    total_steps = repeats * primitive["steps"]
    progress = SimpleProgressBar(total_steps, prefix=primitive_name, enabled=show_progress)
    LOGGER.log(
        primitive_log_level,
        "Executing %s for %d repeats (sim_dt=%.6fs, real_dt=%.6fs, detail=%.2f)",
        primitive_name,
        repeats,
        sim_dt,
        render_dt,
        detail,
    )

    # Accumulate frame budget and dt so lower SIM_DETAIL still advances time correctly.
    frame_budget = 0.0
    pending_render_dt = 0.0
    run_wall_start = time.perf_counter()
    target_real_elapsed = 0.0
    catchup_logged = False

    for repeat_idx in range(repeats):
        for s in range(primitive["steps"]):
            pose = apply_local_body_step(pose, body_step)
            trajectory.append(pose.copy())

            if render and env is not None:
                pending_render_dt += render_dt
                target_real_elapsed += render_dt
                is_last_step = (repeat_idx == repeats - 1) and (s == primitive["steps"] - 1)

                wall_elapsed = time.perf_counter() - run_wall_start
                lag = wall_elapsed - target_real_elapsed
                catchup_active = REALTIME_CATCHUP and (lag > REALTIME_LAG_TOL_S)
                effective_detail = detail

                if catchup_active:
                    # Reduce render density aggressively when lagging behind wall-clock time.
                    lag_ratio = lag / max(render_dt, 1e-9)
                    adaptive_detail = detail / (1.0 + lag_ratio)
                    adaptive_detail = max(CATCHUP_MIN_RENDER_DETAIL, adaptive_detail)
                    effective_detail = min(detail, CATCHUP_RENDER_DETAIL, adaptive_detail)
                    if not catchup_logged:
                        LOGGER.log(
                            primitive_log_level,
                            "Realtime catch-up engaged for %s (lag=%.3fs, detail %.2f -> %.2f)",
                            primitive_name,
                            lag,
                            detail,
                            effective_detail,
                        )
                        catchup_logged = True

                if effective_detail >= 1.0:
                    should_render = True
                elif effective_detail <= 0.0:
                    should_render = is_last_step
                else:
                    frame_budget += effective_detail
                    should_render = (frame_budget >= 1.0) or is_last_step

                if should_render:
                    if effective_detail > 0.0:
                        frame_budget %= 1.0
                    else:
                        frame_budget = 0.0
                    set_render_state(pose, primitive["joint_sequence"][s])
                    step_dt = (
                        min(pending_render_dt, CATCHUP_MIN_STEP_DT)
                        if catchup_active
                        else pending_render_dt
                    )
                    env.step(dt=step_dt)
                    pending_render_dt = 0.0

            progress.update(1)
            if DEBUG_MODE and ((s + 1) % 50 == 0 or s == primitive["steps"] - 1):
                LOGGER.debug(
                    "%s repeat %d/%d step %d/%d pose=%s",
                    primitive_name,
                    repeat_idx + 1,
                    repeats,
                    s + 1,
                    primitive["steps"],
                    np.array2string(pose, precision=4),
                )

    progress.close()
    LOGGER.log(
        primitive_log_level,
        "Finished %s. Final pose=%s",
        primitive_name,
        np.array2string(pose, precision=4),
    )
    return pose, np.array(trajectory)


#=============================================================
# Output, Metrics, And Sequence Utilities
#=============================================================

def save_trajectory_plot(trajectory, target_pose, title, output_path):
    """Save trajectory plot with start, target, and final markers.

    Args:
        trajectory: Pose trajectory array with shape (N, 3).
        target_pose: Target pose [x, y, theta].
        title: Plot title.
        output_path: Path to output image file.

    Returns:
        None.
    """
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(trajectory[:, 0], trajectory[:, 1], "b-", linewidth=1.5, label="trajectory")
    ax.scatter(trajectory[0, 0], trajectory[0, 1], c="green", s=80, label="start")
    ax.scatter(target_pose[0], target_pose[1], c="red", s=80, marker="x", label="target")
    ax.scatter(trajectory[-1, 0], trajectory[-1, 1], c="black", s=60, label="final")

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_path_planning_map_plot(floorplan, path_segments, title, output_path):
    """Save a house-map plot with PRM path segments overlaid.

    Args:
        floorplan: Occupancy grid for the house map.
        path_segments: List of (start_name, goal_name, path_points) tuples.
        title: Plot title.
        output_path: Path to output image file.

    Returns:
        None.
    """
    fig, ax = plt.subplots(figsize=(10, 7))

    height, width = floorplan.shape
    ax.imshow(
        floorplan,
        origin="lower",
        cmap="gray_r",
        extent=[0, width, 0, height],
        interpolation="nearest",
    )

    color_map = plt.colormaps.get_cmap("tab20")
    for idx, (start_name, goal_name, path) in enumerate(path_segments):
        path = np.asarray(path, dtype=float)
        color = color_map(idx % color_map.N)
        label = f"{start_name} -> {goal_name}" if idx < 8 else None
        ax.plot(path[:, 0], path[:, 1], color=color, linewidth=1.6, alpha=0.9, label=label)
        ax.scatter(path[0, 0], path[0, 1], color=color, s=18, marker="o")
        ax.scatter(path[-1, 0], path[-1, 1], color=color, s=18, marker="x")

    ax.set_title(title)
    ax.set_xlabel("x [cm]")
    ax.set_ylabel("y [cm]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.grid(False)
    if path_segments:
        ax.legend(loc="upper right", fontsize=8, framealpha=0.85)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def format_pose_m_deg(pose):
    """Format pose [x, y, theta] as meters/degrees text.

    Args:
        pose: Pose vector [x, y, theta].

    Returns:
        str: Readable pose string.
    """
    return f"x={pose[0]:.4f} m, y={pose[1]:.4f} m, theta={np.rad2deg(pose[2]):.2f} deg"


def append_metrics_row(metrics, test_name, pose, target):
    """Append one metrics row and return pose error.

    Args:
        metrics: Mutable list of CSV row entries.
        test_name: Test case identifier.
        pose: Final pose [x, y, theta].
        target: Target pose [x, y, theta].

    Returns:
        np.ndarray: Error vector pose - target.
    """
    err = pose - target
    metrics.append([
        test_name,
        pose[0],
        pose[1],
        np.rad2deg(pose[2]),
        err[0],
        err[1],
        np.rad2deg(err[2]),
    ])
    return err


def log_test_summary(test_name, pose, target):
    """Log one-line summary for a completed test.

    Args:
        test_name: Test case identifier.
        pose: Final pose [x, y, theta].
        target: Target pose [x, y, theta].

    Returns:
        None.
    """
    err = pose - target
    LOGGER.info(
        "%s | final=(%s) | target=(%s) | err=(%.6f m, %.6f m, %.6f deg)",
        test_name,
        format_pose_m_deg(pose),
        format_pose_m_deg(target),
        err[0],
        err[1],
        np.rad2deg(err[2]),
    )


def reset_render_to_start(render, primitives, start_pose):
    """Reset render state to a consistent start pose.

    Args:
        render: Whether rendering is enabled.
        primitives: Primitive dictionary.
        start_pose: Reset pose [x, y, theta].

    Returns:
        None.
    """
    if render and env is not None:
        set_render_state(np.array(start_pose, dtype=float), primitives["forward_10cm"]["joint_sequence"][0])
        env.step(dt=primitive_step_render_dt(primitives["forward_10cm"]))


def predict_sequence_target(start_pose, primitives, sequence_steps):
    """Predict final pose by integrating sequence body-steps offline.

    Args:
        start_pose: Initial pose [x, y, theta].
        primitives: Primitive dictionary.
        sequence_steps: List of (primitive_name, repeats).

    Returns:
        np.ndarray: Predicted final pose.
    """
    pose = np.array(start_pose, dtype=float)
    for primitive_name, repeats in sequence_steps:
        primitive = primitives[primitive_name]
        body_step = primitive["body_local_step"]
        for _ in range(repeats * primitive["steps"]):
            pose = apply_local_body_step(pose, body_step)
    return pose


def execute_sequence(start_pose, primitives, sequence_steps, render=False, prefix="sequence"):
    """Execute a primitive sequence and merge all segment trajectories.

    Args:
        start_pose: Initial pose [x, y, theta].
        primitives: Primitive dictionary.
        sequence_steps: List of (primitive_name, repeats).
        render: If True, render while executing.
        prefix: Prefix for per-segment logging/progress labels.

    Returns:
        tuple:
            pose: Final pose after all steps.
            merged_trajectory: Combined trajectory array.
    """
    pose = np.array(start_pose, dtype=float)
    merged_trajectory = [pose.copy()]

    for idx, (primitive_name, repeats) in enumerate(sequence_steps, start=1):
        pose, segment_traj = execute_primitive(
            pose,
            primitives[primitive_name],
            repeats=repeats,
            render=render,
            primitive_name=f"{prefix}_{idx}_{primitive_name}",
            show_progress=SHOW_PROGRESS,
        )
        merged_trajectory.extend(segment_traj[1:, :])

    return pose, np.array(merged_trajectory)


def format_sequence(sequence_steps):
    """Format sequence list for concise logging output.

    Args:
        sequence_steps: List of (primitive_name, repeats).

    Returns:
        str: Joined sequence summary.
    """
    return ", ".join(f"{repeats}x{primitive_name}" for primitive_name, repeats in sequence_steps)


def build_part1_test_cases():
    """Build sequence-driven definitions for all required Part 1 tests.

    Returns:
        list[dict]: Test definitions with title, sequence, and target.
    """
    return [
        {
            "name": "test1_A_to_B",
            "title": "Part1 Test1: A to B (100cm forward)",
            "output_file": "part1_test1_A_to_B.png",
            "sequence": [("forward_10cm", 10)],
            "target": np.array([1.0, 0.0, 0.0], dtype=float),
        },
        {
            "name": "test2_A_to_C_ccw",
            "title": "Part1 Test2: A to C (+10deg)",
            "output_file": "part1_test2_A_to_C_ccw.png",
            "sequence": [("turn_1deg_ccw", 10)],
            "target": np.array([0.0, 0.0, np.deg2rad(10.0)], dtype=float),
        },
        {
            "name": "test2_A_to_C_cw",
            "title": "Part1 Test2: A to C (-10deg)",
            "output_file": "part1_test2_A_to_C_cw.png",
            "sequence": [("turn_1deg_cw", 10)],
            "target": np.array([0.0, 0.0, np.deg2rad(-10.0)], dtype=float),
        },
        {
            "name": "test3_A_to_D_ccw",
            "title": "Part1 Test3: A to D (100cm and +10deg)",
            "output_file": "part1_test3_A_to_D_ccw.png",
            "sequence": [("forward_10cm", 10), ("turn_1deg_ccw", 10)],
            "target": np.array([1.0, 0.0, np.deg2rad(10.0)], dtype=float),
        },
        {
            "name": "test3_A_to_D_cw",
            "title": "Part1 Test3: A to D (100cm and -10deg)",
            "output_file": "part1_test3_A_to_D_cw.png",
            "sequence": [("forward_10cm", 10), ("turn_1deg_cw", 10)],
            "target": np.array([1.0, 0.0, np.deg2rad(-10.0)], dtype=float),
        },
        {
            "name": "test4_A_to_E_sequence",
            "title": "Part1 Test4: A to E (4F,10CW,4F,10CCW,4F,10CW,4F)",
            "output_file": "part1_test4_A_to_E_sequence.png",
            "sequence": [
                ("forward_10cm", 4),
                ("turn_1deg_cw", 10),
                ("forward_10cm", 4),
                ("turn_1deg_ccw", 10),
                ("forward_10cm", 4),
                ("turn_1deg_cw", 10),
                ("forward_10cm", 4),
            ],
            # Leave as None so target is derived from sequence and primitives.
            "target": None,
        },
        {
            "name": "test4_A_to_F_sequence",
            "title": "Part1 Test4: A to F (4F,10CW,4F,10CCW,4F,10CW,4F)",
            "output_file": "part1_test4_A_to_F_sequence.png",
            "sequence": [
                ("forward_10cm", 4),
                ("turn_1deg_cw", 360),
                ("forward_10cm", 3),
                ("turn_1deg_ccw", 90),
                ("forward_10cm", 2),
                ("turn_1deg_cw", 45),
                ("forward_10cm", 4),
            ],
            # Leave as None so target is derived from sequence and primitives.
            "target": None,
        },
    ]


#=============================================================
# Part 1 Runner
#=============================================================

def run_part1_required_tests(base_dir, render=False, hold_window=False):
    """Run assignment-required Part 1 tests and export artifacts.

    Args:
        base_dir: Base folder containing output and primitive directories.
        render: If True, execute with interactive rendering enabled.
        hold_window: If True, keep visualization open after execution.

    Returns:
        None.
    """
    output_dir = base_dir / "output"
    primitives_dir = base_dir / "primitives"
    output_dir.mkdir(parents=True, exist_ok=True)

    LOGGER.info("Running Part 1 tests. render=%s hold_window=%s", render, hold_window)
    primitives = create_motion_primitives(primitives_dir)

    metrics = []

    if render:
        start_robot_environment(initial_pose=np.array([0.0, 0.0, 0.0]))

    start_pose = np.array([0.0, 0.0, 0.0], dtype=float)

    test_cases = build_part1_test_cases()
    for test_case in test_cases:
        sequence_steps = test_case["sequence"]
        target = test_case["target"]
        if target is None:
            target = predict_sequence_target(start_pose, primitives, sequence_steps)

        reset_render_to_start(render, primitives, start_pose)
        pose, traj = execute_sequence(
            start_pose,
            primitives,
            sequence_steps,
            render=render,
            prefix=test_case["name"],
        )

        save_trajectory_plot(
            traj,
            target,
            test_case["title"],
            output_dir / test_case["output_file"],
        )
        append_metrics_row(metrics, test_case["name"], pose, target)
        log_test_summary(test_case["name"], pose, target)
        LOGGER.debug("%s sequence: %s", test_case["name"], format_sequence(sequence_steps))

    if render:
        if hold_window and env is not None:
            print("Simulation complete. Close the PyPlot window to continue.")
            env.hold()
        stop_robot_environment()

    metrics_file = output_dir / "part1_metrics.csv"
    with metrics_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "test_name",
                "final_x_m",
                "final_y_m",
                "final_theta_deg",
                "error_x_m",
                "error_y_m",
                "error_theta_deg",
            ]
        )
        writer.writerows(metrics)

    LOGGER.info("Part 1 artifacts written to: %s", output_dir)
    LOGGER.info("Primitives written to: %s", primitives_dir)
    for name, primitive in primitives.items():
        LOGGER.info("%s: min_support_legs=%d", name, primitive["min_support_legs"])


#=============================================================
# Part 3 Path Planning Helpers
#=============================================================

def PRMPlanner_use():
    """Create and pre-plan a PRM planner from the house occupancy grid.

    Returns:
        tuple:
            prm: Configured and planned PRMPlanner instance.
            places: Named place definitions loaded from dataset.
    """
    house = rtb_load_matfile("data/house.mat")

    floorplan = house["floorplan"]
    places = house["places"]

    prm = PRMPlanner(occgrid=floorplan, seed=0)

    prm.plan(300)

    return prm, places


def createPath(prm, start, goal):
    """Query PRM for a path between start and goal points.

    Args:
        prm: PRMPlanner instance.
        start: Start coordinate in planner map frame.
        goal: Goal coordinate in planner map frame.

    Returns:
        np.ndarray: Sequence of path points.
    """
    path = prm.query(start=start, goal=goal)
    return path


def pathToSeq(path, start_heading_deg=0.0):
    """Convert path points to turn angles and forward primitive counts.

    Args:
        path: Path point sequence returned by PRM query.
        start_heading_deg: Initial heading of the robot in degrees.

    Returns:
        tuple:
            angles: Signed heading changes in whole degrees.
            dists: Forward counts in 10 cm units.
    """
    angles = []
    dists = []
    start_heading_rad = np.deg2rad(start_heading_deg)
    vStart = np.array([np.cos(start_heading_rad), np.sin(start_heading_rad)])
    v2 = np.array(path[1] - path[0])

    anglerad = np.arctan2(vStart[0] * v2[1] - vStart[1] * v2[0], np.dot(vStart, v2))
    angle = np.rad2deg(anglerad)
    angles.append(round(angle))
    dists.append(round(np.linalg.norm(v2) / 10))

    for i in range(2, len(path)):
        xPrev, yPrev = path[i - 2]
        xCurr, yCurr = path[i - 1]
        xNext, yNext = path[i]

        vPrev = np.array([xCurr - xPrev, yCurr - yPrev])
        vNext = np.array([xNext - xCurr, yNext - yCurr])

        anglerad = np.arctan2(vPrev[0] * vNext[1] - vPrev[1] * vNext[0], np.dot(vPrev, vNext))
        angle = np.rad2deg(anglerad)
        angles.append(round(angle))

        dists.append(round(np.linalg.norm(vNext) / 10))
    return angles, dists


def followPath(angles, dists):
    """Build primitive sequence from turn and distance lists.

    Args:
        angles: Signed turn values in degrees.
        dists: Forward counts in 10 cm units.

    Returns:
        list[tuple[str, int]]: Primitive sequence ready for execute_sequence.

    Raises:
        AssertionError: If angles and dists lengths differ.
    """
    assert len(angles) == len(dists)  # Number of angles and distances not the same
    sequence = []
    for i, angle in enumerate(angles):
        if angle >= 0:
            primName = "turn_1deg_ccw"
        else:
            primName = "turn_1deg_cw"
        sequence.append((primName, np.abs(angle)))
        primName = "forward_10cm"
        sequence.append((primName, dists[i]))
    return sequence


def build_continuous_segment_pairs(
    waypoint_names,
    both_ways=True,
    all_hubs=False,
    return_metadata=False,
):
    """Build segment pairs from selected waypoints.

    Args:
        waypoint_names: Ordered place-name list to visit.
        both_ways: If True, add return leg after each outbound leg.
        all_hubs: If True, iterate hubs from index 0..N-2 and for each hub
            visit all later waypoints with out-and-back pattern.
        return_metadata: If True, also return per-segment hub index and
            total hub count.

    Returns:
        list[tuple[str, str]] or tuple:
            Directed (start_name, goal_name) segment pairs, and optionally
            segment_hub_indices plus total_hubs.
    """
    if len(waypoint_names) < 2:
        if return_metadata:
            return [], [], 0
        return []

    if not all_hubs:
        hub = waypoint_names[0]
        outbounds = waypoint_names[1:]
        if both_ways:
            segment_pairs = []
            for waypoint in outbounds:
                segment_pairs.append((hub, waypoint))
                segment_pairs.append((waypoint, hub))
        else:
            segment_pairs = [(hub, waypoint) for waypoint in outbounds]

        if return_metadata:
            return segment_pairs, [1] * len(segment_pairs), 1
        return segment_pairs

    segment_pairs = []
    segment_hub_indices = []
    total_hubs = max(1, len(waypoint_names) - 1)

    for hub_idx0, hub in enumerate(waypoint_names[:-1]):
        hub_idx = hub_idx0 + 1
        if segment_pairs and segment_pairs[-1][1] != hub:
            segment_pairs.append((segment_pairs[-1][1], hub))
            segment_hub_indices.append(hub_idx)

        for waypoint in waypoint_names[hub_idx0 + 1 :]:
            segment_pairs.append((hub, waypoint))
            segment_hub_indices.append(hub_idx)
            if both_ways:
                segment_pairs.append((waypoint, hub))
                segment_hub_indices.append(hub_idx)

    if return_metadata:
        return segment_pairs, segment_hub_indices, total_hubs
    return segment_pairs


def apply_predictive_goal_correction(start_pose, goal_xy_m, primitives, sequence_steps):
    """Append a short correction sequence if prediction shows reduced goal error.

    Args:
        start_pose: Pose before execution [x, y, theta].
        goal_xy_m: Goal XY position in meters.
        primitives: Primitive dictionary.
        sequence_steps: Base sequence list.

    Returns:
        tuple:
            corrected_sequence: Possibly augmented sequence.
            predicted_pose: Predicted final pose for corrected sequence.
            predicted_goal_error_norm: Predicted XY error norm in meters.
            correction_used: True if an extra correction was appended.
    """
    base_sequence = list(sequence_steps)
    best_sequence = base_sequence
    best_pose = predict_sequence_target(start_pose, primitives, best_sequence)
    best_goal_error = float(np.linalg.norm(best_pose[:2] - goal_xy_m))

    error_vector = goal_xy_m - best_pose[:2]
    if np.linalg.norm(error_vector) <= 1e-9:
        return best_sequence, best_pose, best_goal_error, False

    heading_to_goal = np.arctan2(error_vector[1], error_vector[0])
    heading_delta = np.arctan2(
        np.sin(heading_to_goal - best_pose[2]),
        np.cos(heading_to_goal - best_pose[2]),
    )
    turn_deg = int(np.round(np.rad2deg(heading_delta)))
    turn_deg = int(np.clip(turn_deg, -45, 45))

    forward_nominal = int(np.round(np.linalg.norm(error_vector) / FORWARD_DISTANCE_M))
    forward_nominal = max(0, min(4, forward_nominal))
    forward_candidates = sorted(
        {
            0,
            max(0, forward_nominal - 1),
            forward_nominal,
            min(4, forward_nominal + 1),
        }
    )

    correction_candidates = []
    for forward_repeats in forward_candidates:
        if forward_repeats > 0:
            correction_candidates.append([("forward_10cm", forward_repeats)])

        if abs(turn_deg) > 0 and forward_repeats > 0:
            turn_name = "turn_1deg_ccw" if turn_deg > 0 else "turn_1deg_cw"
            correction_candidates.append(
                [(turn_name, abs(turn_deg)), ("forward_10cm", forward_repeats)]
            )

    for correction in correction_candidates:
        candidate_sequence = base_sequence + correction
        candidate_pose = predict_sequence_target(start_pose, primitives, candidate_sequence)
        candidate_goal_error = float(np.linalg.norm(candidate_pose[:2] - goal_xy_m))
        if candidate_goal_error + 1e-12 < best_goal_error:
            best_sequence = candidate_sequence
            best_pose = candidate_pose
            best_goal_error = candidate_goal_error

    correction_used = len(best_sequence) > len(base_sequence)
    return best_sequence, best_pose, best_goal_error, correction_used


#=============================================================
# Entry Points
#=============================================================

def main_part1():
    """Run Part 1 test suite using module-level runtime configuration.

    Returns:
        None.
    """
    setup_logging(DEBUG_MODE)
    LOGGER.info(
        "Config: simspeed=%.3f simdetail=%.2f robotspeed=%.3f robotrotspeed=%.3f render=%s debug=%s progress=%s primitive_logs=%s realtime_catchup=%s",
        SIM_SPEED,
        SIM_DETAIL,
        ROBOT_SPEED,
        ROBOT_ROT_SPEED,
        RENDER,
        DEBUG_MODE,
        SHOW_PROGRESS,
        LOG_PRIMITIVE_DETAILS,
        REALTIME_CATCHUP,
    )

    base_dir = Path(__file__).resolve().parent
    render = RENDER
    hold_window = render and HOLD_WINDOW
    run_part1_required_tests(base_dir=base_dir, render=render, hold_window=hold_window)
    stop_robot_environment()



def main_part2():
    """Run random PRM navigation trials and export trajectory plots.

    Returns:
        None.
    """
    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "output"
    primitives_dir = base_dir / "primitives"
    output_dir.mkdir(parents=True, exist_ok=True)
    prm, places = PRMPlanner_use()
    placelist = list(places)

    for i in range(5):
        start, goal = random.sample(placelist, 2)
        startroom = getattr(places, start)
        goalroom = getattr(places, goal)

        pose = np.array([0.0, 0.0, 0.0], dtype=float)
        if RENDER:
            start_robot_environment(initial_pose=pose)

        primitives = create_motion_primitives(primitives_dir)

        path = createPath(prm, startroom, goalroom)

        angles, dists = pathToSeq(path)
        sequence = followPath(angles, dists)

        target = predict_sequence_target(pose, primitives, sequence)
        pose, traj = execute_sequence(
            pose,
            primitives,
            sequence,
            render=RENDER,
            prefix="part3_draw",
        )

        save_trajectory_plot(
            traj,
            target,
            f"part3_{i}_draw",
            output_dir / f"part3_{i}_draw.png",
        )
        log_test_summary("part3_draw", pose, target)

        if RENDER:
            stop_robot_environment()


def main_part2_2(
    num_waypoints=4,
    both_ways=True,
    all_hubs=True,
    segment_goal_tolerance_m=0.05,
    max_refine_loops=4,
):
    """Demonstrate continuous multi-path navigation without resetting between paths.

    The robot traverses planner-defined path segments continuously without resetting
    between segments. Outputs include per-segment metrics, full-run metrics,
    planner-map plot, and executed-trajectory plot.

    Args:
        num_waypoints: Selection count. When all_hubs=True, this is the number
            of hubs. When all_hubs=False, this is the number of places.
            Use None or <=0 to include all available hubs/places.
        both_ways: If True, run out-and-back between each hub and target.
            p0->p1->p0->p2->p0...
        all_hubs: If True, run hub phases for all waypoint indices:
            hub0 phase, then hub1 phase, then hub2 phase, etc.
        segment_goal_tolerance_m: Desired XY tolerance to segment goal.
        max_refine_loops: Max extra replans per segment when outside tolerance.

    Returns:
        None.
    """
    setup_logging(DEBUG_MODE)
    cm_to_m = 0.01

    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "output"
    primitives_dir = base_dir / "primitives"
    output_dir.mkdir(parents=True, exist_ok=True)

    if all_hubs:
        mode_text = "all-hub out-and-back" if both_ways else "all-hub out-only"
    else:
        mode_text = "single-hub out-and-back" if both_ways else "single-hub out-only"
    LOGGER.info("Running Part 2.2: Continuous multi-path navigation (%s)", mode_text)

    house = rtb_load_matfile("data/house.mat")
    floorplan = house["floorplan"]

    prm, places = PRMPlanner_use()
    placelist = list(places)
    total_places = len(placelist)

    if total_places < 2:
        raise RuntimeError("Part 2.2 requires at least two places in data/house.mat")

    if all_hubs:
        max_available_hubs = total_places - 1
        if (
            num_waypoints is None
            or int(num_waypoints) <= 0
            or int(num_waypoints) > max_available_hubs
        ):
            selected_hubs = max_available_hubs
        else:
            selected_hubs = int(num_waypoints)
        selected_count = selected_hubs + 1
    else:
        if num_waypoints is None or int(num_waypoints) <= 0 or int(num_waypoints) > total_places:
            selected_count = total_places
        else:
            selected_count = int(num_waypoints)
        selected_hubs = 1

    waypoint_names = placelist[:selected_count]
    segment_pairs, segment_hub_indices, total_hubs = build_continuous_segment_pairs(
        waypoint_names,
        both_ways=both_ways,
        all_hubs=all_hubs,
        return_metadata=True,
    )

    if not segment_pairs:
        raise RuntimeError("No segment pairs were generated for Part 2.2")

    total_segments = len(segment_pairs)

    if all_hubs:
        LOGGER.info(
            "Part 2.2 using %d/%d hubs (%d/%d places) -> %d segments (all_hubs=%s, tolerance=%.3f m, max_refines=%d)",
            total_hubs,
            total_places - 1,
            selected_count,
            total_places,
            total_segments,
            all_hubs,
            segment_goal_tolerance_m,
            max_refine_loops,
        )
    else:
        LOGGER.info(
            "Part 2.2 using %d/%d places -> %d segments (all_hubs=%s, tolerance=%.3f m, max_refines=%d)",
            selected_count,
            total_places,
            total_segments,
            all_hubs,
            segment_goal_tolerance_m,
            max_refine_loops,
        )

    primitives = create_motion_primitives(primitives_dir)

    first_place_xy_cm = np.asarray(getattr(places, waypoint_names[0]), dtype=float)
    start_pose = np.array(
        [first_place_xy_cm[0] * cm_to_m, first_place_xy_cm[1] * cm_to_m, 0.0],
        dtype=float,
    )
    pose = start_pose.copy()

    if RENDER:
        start_robot_environment(initial_pose=pose)

    segment_metrics = []
    segment_goal_error_norms = []
    total_predictive_corrections = 0
    full_sequence = []
    full_trajectory = [pose.copy()]
    path_segments = []

    for segment_idx, ((start_name, goal_name), hub_idx) in enumerate(
        zip(segment_pairs, segment_hub_indices),
        start=1,
    ):
        goal_xy_cm = np.asarray(getattr(places, goal_name), dtype=float)
        goal_xy_m = goal_xy_cm * cm_to_m

        segment_start_pose = pose.copy()
        segment_sequence = []
        planned_path_points = 0
        replans_used = 0
        predictive_corrections_used = 0

        for attempt_idx in range(max_refine_loops + 1):
            planner_start_xy_cm = pose[:2] / cm_to_m

            path_query = createPath(prm, planner_start_xy_cm, goal_xy_cm)
            if path_query is None:
                path = np.vstack((planner_start_xy_cm, goal_xy_cm))
            else:
                path = np.asarray(path_query, dtype=float)

            if path.ndim != 2 or path.shape[0] < 2:
                path = np.vstack((planner_start_xy_cm, goal_xy_cm))

            planned_path_points += int(path.shape[0])
            replan_start_name = start_name if attempt_idx == 0 else f"{start_name}[r{attempt_idx}]"
            path_segments.append((replan_start_name, goal_name, path))

            heading_deg = float(np.rad2deg(pose[2]))
            angles, dists = pathToSeq(path, start_heading_deg=heading_deg)
            raw_sequence = followPath(angles, dists)
            sequence = [
                (primitive_name, int(repeats))
                for primitive_name, repeats in raw_sequence
                if int(repeats) > 0
            ]

            sequence, pose_prediction, _, correction_used = apply_predictive_goal_correction(
                pose,
                goal_xy_m,
                primitives,
                sequence,
            )
            if correction_used:
                predictive_corrections_used += 1

            if sequence:
                pose, segment_traj = execute_sequence(
                    pose,
                    primitives,
                    sequence,
                    render=RENDER,
                    prefix=f"part2_2_h{hub_idx}of{total_hubs}_seg{segment_idx}of{total_segments}_r{attempt_idx}_{start_name}_to_{goal_name}",
                )

                if segment_traj.shape[0] > 1:
                    full_trajectory.extend(segment_traj[1:, :])

                full_sequence.extend(sequence)
                segment_sequence.extend(sequence)

            goal_error_xy = pose[:2] - goal_xy_m
            goal_error_norm = float(np.linalg.norm(goal_error_xy))
            replans_used = attempt_idx

            if goal_error_norm <= segment_goal_tolerance_m:
                break

            if attempt_idx < max_refine_loops:
                LOGGER.info(
                    "hub %d of %d, segment %d of %d, %s -> %s refine %d/%d | goal_err=%.4f m > tol=%.4f m",
                    hub_idx,
                    total_hubs,
                    segment_idx,
                    total_segments,
                    start_name,
                    goal_name,
                    attempt_idx + 1,
                    max_refine_loops,
                    goal_error_norm,
                    segment_goal_tolerance_m,
                )

        segment_predicted_pose = predict_sequence_target(segment_start_pose, primitives, segment_sequence)
        sequence_error = pose - segment_predicted_pose

        segment_metrics.append(
            [
                segment_idx,
                start_name,
                goal_name,
                planned_path_points,
                int(len(segment_sequence)),
                replans_used,
                predictive_corrections_used,
                segment_start_pose[0],
                segment_start_pose[1],
                np.rad2deg(segment_start_pose[2]),
                goal_xy_m[0],
                goal_xy_m[1],
                pose[0],
                pose[1],
                np.rad2deg(pose[2]),
                goal_error_xy[0],
                goal_error_xy[1],
                goal_error_norm,
                sequence_error[0],
                sequence_error[1],
                np.rad2deg(sequence_error[2]),
            ]
        )
        segment_goal_error_norms.append(goal_error_norm)
        total_predictive_corrections += predictive_corrections_used

        LOGGER.info(
            "hub %d of %d, segment %d of %d, %s -> %s | replans=%d pred_corr=%d path_points=%d seq=%s | goal_err=(%.4f m, %.4f m, %.4f m)",
            hub_idx,
            total_hubs,
            segment_idx,
            total_segments,
            start_name,
            goal_name,
            replans_used,
            predictive_corrections_used,
            planned_path_points,
            format_sequence(segment_sequence) if segment_sequence else "no-op",
            goal_error_xy[0],
            goal_error_xy[1],
            goal_error_norm,
        )

    if RENDER:
        if HOLD_WINDOW and env is not None:
            print("Simulation complete. Close the PyPlot window to continue.")
            env.hold()
        stop_robot_environment()

    full_trajectory = np.array(full_trajectory, dtype=float)

    final_goal_name = segment_pairs[-1][1]
    final_goal_xy_cm = np.asarray(getattr(places, final_goal_name), dtype=float)
    final_goal_xy_m = final_goal_xy_cm * cm_to_m
    final_goal_pose = np.array([final_goal_xy_m[0], final_goal_xy_m[1], 0.0], dtype=float)

    combined_predicted_final_pose = predict_sequence_target(start_pose, primitives, full_sequence)
    combined_sequence_error = pose - combined_predicted_final_pose
    final_goal_error_xy = pose[:2] - final_goal_xy_m
    final_goal_error_norm = float(np.linalg.norm(final_goal_error_xy))

    segment_error_norms = np.array(segment_goal_error_norms, dtype=float)
    mean_segment_goal_error = float(np.mean(segment_error_norms)) if segment_error_norms.size else 0.0
    max_segment_goal_error = float(np.max(segment_error_norms)) if segment_error_norms.size else 0.0

    combined_sequence_match = np.allclose(pose, combined_predicted_final_pose, atol=1e-10)
    if combined_sequence_match:
        LOGGER.info("Combined sequence replay check passed")
    else:
        LOGGER.warning(
            "Combined sequence replay mismatch: dx=%.6e, dy=%.6e, dtheta=%.6e rad",
            combined_sequence_error[0],
            combined_sequence_error[1],
            combined_sequence_error[2],
        )

    segment_metrics_file = output_dir / "part2_2_segment_metrics.csv"
    with segment_metrics_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "segment_index",
                "start_place",
                "goal_place",
                "path_points",
                "primitive_sequence_items",
                "replans_used",
                "predictive_corrections_used",
                "segment_start_x_m",
                "segment_start_y_m",
                "segment_start_theta_deg",
                "goal_x_m",
                "goal_y_m",
                "final_x_m",
                "final_y_m",
                "final_theta_deg",
                "goal_error_x_m",
                "goal_error_y_m",
                "goal_error_norm_m",
                "sequence_error_x_m",
                "sequence_error_y_m",
                "sequence_error_theta_deg",
            ]
        )
        writer.writerows(segment_metrics)

    combined_metrics_file = output_dir / "part2_2_combined_metrics.csv"
    with combined_metrics_file.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "selected_waypoints",
                "selected_hubs",
                "segment_count",
                "both_ways",
                "full_sequence_items",
                "predictive_corrections_used",
                "full_trajectory_points",
                "start_place",
                "final_goal_place",
                "start_x_m",
                "start_y_m",
                "final_goal_x_m",
                "final_goal_y_m",
                "final_x_m",
                "final_y_m",
                "final_theta_deg",
                "final_goal_error_x_m",
                "final_goal_error_y_m",
                "final_goal_error_norm_m",
                "mean_segment_goal_error_m",
                "max_segment_goal_error_m",
                "combined_sequence_error_x_m",
                "combined_sequence_error_y_m",
                "combined_sequence_error_theta_deg",
                "combined_sequence_match",
            ]
        )
        writer.writerow(
            [
                selected_count,
                selected_hubs,
                total_segments,
                both_ways,
                len(full_sequence),
                total_predictive_corrections,
                len(full_trajectory),
                waypoint_names[0],
                final_goal_name,
                start_pose[0],
                start_pose[1],
                final_goal_xy_m[0],
                final_goal_xy_m[1],
                pose[0],
                pose[1],
                np.rad2deg(pose[2]),
                final_goal_error_xy[0],
                final_goal_error_xy[1],
                final_goal_error_norm,
                mean_segment_goal_error,
                max_segment_goal_error,
                combined_sequence_error[0],
                combined_sequence_error[1],
                np.rad2deg(combined_sequence_error[2]),
                combined_sequence_match,
            ]
        )

    map_plot_file = output_dir / "part2_2_path_plan_map.png"
    save_path_planning_map_plot(
        floorplan,
        path_segments,
        f"Part 2.2 planned paths ({selected_count} waypoints, {mode_text})",
        map_plot_file,
    )

    trajectory_plot_file = output_dir / "part2_2_continuous_navigation.png"
    save_trajectory_plot(
        full_trajectory,
        final_goal_pose,
        f"Part 2.2 continuous navigation ({total_segments} segments)",
        trajectory_plot_file,
    )

    LOGGER.info("Part 2.2 segment metrics: %s", segment_metrics_file)
    LOGGER.info("Part 2.2 combined metrics: %s", combined_metrics_file)
    LOGGER.info("Part 2.2 path map plot: %s", map_plot_file)
    LOGGER.info("Part 2.2 trajectory plot: %s", trajectory_plot_file)


if __name__ == "__main__":
    # main_part1()
    # main_part2()
    main_part2_2(num_waypoints=None, both_ways=True, all_hubs=True)
    # main_part3()
