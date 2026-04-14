"""Part 1 testbed for TEL200 walking robot.

This module builds motion primitives, runs the required Part 1 tests,
and exports trajectory plots plus numeric pose errors.
"""

from pathlib import Path
import csv
import logging
import sys
import time

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
RENDER = True

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
    """Configure module logger for normal and debug execution."""
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

    # Keep third-party logs quiet in debug mode.
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    logging.getLogger("PIL").setLevel(logging.WARNING)


class SimpleProgressBar:
    """Minimal terminal progress bar without external dependencies."""

    def __init__(self, total, prefix="", enabled=True):
        self.total = max(1, int(total))
        self.prefix = prefix
        self.enabled = enabled
        self.current = 0
        self.last_percent = -1
        if self.enabled:
            self._render()

    def update(self, step=1):
        self.current = min(self.total, self.current + step)
        if not self.enabled:
            return
        percent = int((self.current / self.total) * 100)
        if percent != self.last_percent or self.current == self.total:
            self.last_percent = percent
            self._render()

    def _render(self):
        bar_width = 30
        filled = int(bar_width * self.current / self.total)
        bar = "#" * filled + "-" * (bar_width - filled)
        sys.stdout.write(
            f"\r{self.prefix} [{bar}] {self.current}/{self.total} ({100.0 * self.current / self.total:5.1f}%)"
        )
        sys.stdout.flush()

    def close(self):
        if self.enabled:
            sys.stdout.write("\n")
            sys.stdout.flush()

#=============================================================
# setup
#=============================================================

def gait(cycle, k, offset, flip):
    """Return a single leg joint configuration from the cyclic gait."""
    k = (k + offset) % cycle.shape[0]
    q = cycle[k, :].copy()
    if flip:
        q[0] = -q[0]
    return q


def build_leg_model():
    """Create the 3-DOF leg model used in the assignment baseline."""
    return ERobot(ET.Rz() * ET.Rx() * ET.ty(-L1) * ET.Rx() * ET.tz(-L2))


def build_gait_cycle(leg):
    """Rebuild the same gait profile as the assignment baseline."""
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
    # This avoids abrupt joint flips (teleporting) between consecutive samples.
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
# utils
#=============================================================

def start_robot_environment(initial_pose):
    """Create and launch the interactive robot scene."""
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
    """Close interactive figures and release rendering handles."""
    global env, legs, body, leg_local_offsets
    env = None
    legs = None
    body = None
    leg_local_offsets = None
    plt.close("all")


def pose_to_se3(pose):
    """Convert [x, y, theta] pose to SE3 transform."""
    return SE3(pose[0], pose[1], 0) * SE3.Rz(pose[2])


def set_render_state(pose, leg_joint_angles):
    """Update rendered body and legs from world pose and local joint-space state."""
    if env is None:
        return

    robot_pose = pose_to_se3(pose)
    body.base = robot_pose

    for i, robot_leg in enumerate(legs):
        robot_leg.base = robot_pose * leg_local_offsets[i]
        robot_leg.q = leg_joint_angles[i]

    update_camera(pose)


def update_camera(pose):
    """Center the camera around robot pose to avoid geometry distortion."""
    if env is None or not CAMERA_FOLLOW:
        return

    if not hasattr(env, "ax"):
        return

    x, y, _ = pose
    env.ax.set_xlim(x - CAMERA_HALF_WIDTH_X, x + CAMERA_HALF_WIDTH_X)
    env.ax.set_ylim(y - CAMERA_HALF_WIDTH_Y, y + CAMERA_HALF_WIDTH_Y)
    env.ax.set_zlim(CAMERA_Z_MIN, CAMERA_Z_MAX)
def support_leg_count(xcycle, gait_step_index, support_threshold_z):
    """Estimate number of legs supporting based on z-height in the foot cycle."""
    count = 0
    cycle_len = xcycle.shape[0]
    for phase in PHASE_OFFSETS:
        idx = (gait_step_index + phase) % cycle_len
        if xcycle[idx, 2] <= support_threshold_z:
            count += 1
    return count

def build_joint_sequence(qcycle, steps, start_index=0):
    """Build local joint-space commands for all 4 legs over given step count."""
    seq = np.zeros((steps, 4, 3), dtype=float)
    cycle_len = qcycle.shape[0]

    for s in range(steps):
        gait_idx = (start_index + s) % cycle_len
        for leg_i, (phase, flip) in enumerate(zip(PHASE_OFFSETS, PHASE_FLIPS)):
            seq[s, leg_i, :] = gait(qcycle, gait_idx, phase, flip)

    return seq


def validate_support_constraint(xcycle, steps, start_index, support_threshold_z):
    """Check the minimum support legs across a primitive execution window."""
    min_support = 4
    cycle_len = xcycle.shape[0]
    for s in range(steps):
        gait_idx = (start_index + s) % cycle_len
        support = support_leg_count(xcycle, gait_idx, support_threshold_z)
        min_support = min(min_support, support)
    return min_support


def create_motion_primitives(primitives_dir):
    """
    Build and save assignment primitives in local joint-space.

    Files generated:
    - forward_10cm.npz
    - backward_10cm.npz
    - turn_1deg_ccw.npz
    - turn_1deg_cw.npz
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
def apply_local_body_step(pose, body_local_step):
    """Apply [dx_local, dy_local, dtheta] to [x, y, theta] in world frame."""
    x, y, theta = pose
    dx_local, dy_local, dtheta = body_local_step

    dx_global = dx_local * np.cos(theta) - dy_local * np.sin(theta)
    dy_global = dx_local * np.sin(theta) + dy_local * np.cos(theta)
    return np.array([x + dx_global, y + dy_global, theta + dtheta], dtype=float)


def primitive_step_sim_dt(primitive):
    """
    Compute simulated time per primitive step.

    Translational components are constrained by ROBOT_SPEED (m/s).
    Rotational components are constrained by ROBOT_ROT_SPEED (deg/s).
    If a primitive has both translation and rotation, we use the slower constraint.
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
    """Compute real-time viewer step duration for a primitive step."""
    return primitive_step_sim_dt(primitive) / SIM_SPEED


def execute_primitive(
    pose,
    primitive,
    repeats=1,
    render=False,
    primitive_name="primitive",
    show_progress=True,
):
    """Execute one primitive multiple times and return final pose and trajectory."""
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


def save_trajectory_plot(trajectory, target_pose, title, output_path):
    """Store a simple world-frame trajectory plot with start and target markers."""
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


def format_pose_m_deg(pose):
    """Format pose [x, y, theta] as readable meters/degrees string."""
    return f"x={pose[0]:.4f} m, y={pose[1]:.4f} m, theta={np.rad2deg(pose[2]):.2f} deg"


def append_metrics_row(metrics, test_name, pose, target):
    """Append one metrics row and return error vector pose-target."""
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
    """Log compact one-line summary for a completed test."""
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
    """Reset viewer state to start pose between tests to keep visuals consistent."""
    if render and env is not None:
        set_render_state(np.array(start_pose, dtype=float), primitives["forward_10cm"]["joint_sequence"][0])
        env.step(dt=primitive_step_render_dt(primitives["forward_10cm"]))


def predict_sequence_target(start_pose, primitives, sequence_steps):
    """Predict final pose by integrating primitive body steps without rendering."""
    pose = np.array(start_pose, dtype=float)
    for primitive_name, repeats in sequence_steps:
        primitive = primitives[primitive_name]
        body_step = primitive["body_local_step"]
        for _ in range(repeats * primitive["steps"]):
            pose = apply_local_body_step(pose, body_step)
    return pose


def execute_sequence(start_pose, primitives, sequence_steps, render=False, prefix="sequence"):
    """Execute a list of (primitive_name, repeats) and return final pose and merged trajectory."""
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
    """Format a sequence list for concise logging."""
    return ", ".join(f"{repeats}x{primitive_name}" for primitive_name, repeats in sequence_steps)


def build_part1_test_cases():
    """Return sequence-driven definitions for all required Part 1 tests."""
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


def run_part1_required_tests(base_dir, render=False, hold_window=False):
    """Run assignment-required Part 1 tests and export evidence artifacts."""
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


def PRMPlanner_use():
    house = rtb_load_matfile("data/house.mat")

    floorplan = house["floorplan"]
    places = house["places"]

    prm = PRMPlanner(occgrid=floorplan, seed=0)

    prm.plan(300)
    path = prm.query(start=places.br1, goal=places.br2)


def followPath(path):
    for i in range(2, len(path)):
        x1, y1 = path[i-2]
        x2, y2 = path[i-1]
        x3, y3 = path[i]

        v1 = [x2 - x1, y2 - y1]
        v2 = [x3 - x2, y3 - y2]

        lenv1 = np.sqrt(v1[0]**2 + v1[1]**2)
        lenv2 = np.sqrt(v2[0]**2 + v2[1]**2)
        print(lenv2)
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        AngleTemp = dot / (lenv1 * lenv2)
        AngleRad =np.arccos(AngleTemp)
        Angle = np.degrees(AngleRad)
        print(Angle)

        Angle = np.round(Angle).astype(int)
        lenv2 = np.round(lenv2).astype(int)
        

        print(f"v1: {v1}, v2: {v2}")
        execute_sequence()
        for _ in range(int(lenv2)):
            pass #Activate function for walking forward 10cm

def main_part1():
    """Run Part 1 tests using constants defined at the top of this file."""
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
    followPath(path)
    stop_robot_environment()


def main_part2():
    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "output"
    primitives_dir = base_dir / "primitives"
    output_dir.mkdir(parents=True, exist_ok=True)

    pose = np.array([0.0, 0.0, 0.0], dtype=float)

    start_robot_environment(initial_pose=pose)
    primitives = create_motion_primitives(primitives_dir)
    
    repeats = 5
    pose = np.array([0.0, 0.0, 0.0], dtype=float)

    # primitive_name ["forward_10cm", "turn_1deg_ccw", "turn_1deg_cw"]:


    for primitive_name in ["forward_10cm", "turn_1deg_ccw", "turn_1deg_cw"]:

        execute_primitive(
            pose,
            primitives[primitive_name],
            repeats=repeats,
            render=True,
            primitive_name=f"{primitive_name}_{repeats}x",
            show_progress=SHOW_PROGRESS,
            )
    stop_robot_environment()

def main_part3():
    base_dir = Path(__file__).resolve().parent
    output_dir = base_dir / "output"
    primitives_dir = base_dir / "primitives"
    output_dir.mkdir(parents=True, exist_ok=True)

    pose = np.array([0.0, 0.0, 0.0], dtype=float)
    if RENDER:
        start_robot_environment(initial_pose=pose)

    primitives = create_motion_primitives(primitives_dir)

    
    
    sequence = [
        ("turn_1deg_ccw", 180),
        ("forward_10cm", 8),
        ("turn_1deg_ccw", 90),
        ("forward_10cm", 12),
        ("turn_1deg_ccw", 90),
        ("forward_10cm", 8),
        ("turn_1deg_ccw", 90),
        ("forward_10cm", 6),
        ("turn_1deg_ccw", 45),
        ("forward_10cm", 2),
        ("turn_1deg_ccw", 45),
        ("forward_10cm", 3),
    ]

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
        "part3_draw",
        output_dir / "part3_draw.png",
    )
    log_test_summary("part3_draw", pose, target)

    if RENDER:
        stop_robot_environment()

if __name__ == "__main__":
    #main_part1()
    #main_part2()
    main_part3()