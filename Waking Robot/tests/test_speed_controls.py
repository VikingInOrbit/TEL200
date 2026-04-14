import unittest
import numpy as np
import tempfile
from pathlib import Path

import walkingTEL200_TestBed as tb


class _DummyEnv:
    def __init__(self):
        self.dts = []

    def step(self, dt=None):
        self.dts.append(dt)


class SpeedControlTests(unittest.TestCase):
    def setUp(self):
        self.orig_sim_speed = tb.SIM_SPEED
        self.orig_sim_detail = tb.SIM_DETAIL
        self.orig_robot_speed = tb.ROBOT_SPEED
        self.orig_robot_rot_speed = tb.ROBOT_ROT_SPEED
        self.orig_realtime_catchup = tb.REALTIME_CATCHUP
        self.orig_env = tb.env
        self.orig_set_render_state = tb.set_render_state

    def tearDown(self):
        tb.SIM_SPEED = self.orig_sim_speed
        tb.SIM_DETAIL = self.orig_sim_detail
        tb.ROBOT_SPEED = self.orig_robot_speed
        tb.ROBOT_ROT_SPEED = self.orig_robot_rot_speed
        tb.REALTIME_CATCHUP = self.orig_realtime_catchup
        tb.env = self.orig_env
        tb.set_render_state = self.orig_set_render_state

    @staticmethod
    def _primitive(step_dx=0.01, steps=5):
        return {
            "body_local_step": np.array([step_dx, 0.0, 0.0], dtype=float),
            "steps": steps,
            "joint_sequence": np.zeros((steps, 4, 3), dtype=float),
        }

    def test_robot_speed_does_not_change_commanded_distance(self):
        primitive = self._primitive(step_dx=0.01, steps=5)
        repeats = 10
        expected_x = 0.01 * 5 * repeats

        tb.SIM_SPEED = 1.0
        tb.ROBOT_SPEED = 0.5
        slow_pose, _ = tb.execute_primitive(
            np.array([0.0, 0.0, 0.0]),
            primitive,
            repeats=repeats,
            render=False,
            primitive_name="slow",
            show_progress=False,
        )

        tb.ROBOT_SPEED = 30.0
        fast_pose, _ = tb.execute_primitive(
            np.array([0.0, 0.0, 0.0]),
            primitive,
            repeats=repeats,
            render=False,
            primitive_name="fast",
            show_progress=False,
        )

        self.assertAlmostEqual(slow_pose[0], expected_x, places=10)
        self.assertAlmostEqual(fast_pose[0], expected_x, places=10)
        self.assertAlmostEqual(slow_pose[0], fast_pose[0], places=10)

    def test_one_meter_at_one_mps_takes_one_real_second(self):
        # 0.01m * 10 steps * 10 repeats = 1.0m
        primitive = self._primitive(step_dx=0.01, steps=10)
        dummy_env = _DummyEnv()

        tb.env = dummy_env
        tb.set_render_state = lambda pose, joint_angles: None

        tb.SIM_SPEED = 1.0
        tb.SIM_DETAIL = 1.0
        tb.ROBOT_SPEED = 1.0
        tb.REALTIME_CATCHUP = False

        tb.execute_primitive(
            np.array([0.0, 0.0, 0.0]),
            primitive,
            repeats=10,
            render=True,
            primitive_name="timing_1m_1mps",
            show_progress=False,
        )

        self.assertEqual(len(dummy_env.dts), 100)
        real_elapsed = sum(dummy_env.dts)
        self.assertAlmostEqual(real_elapsed, 1.0, places=6)

    def test_sim_half_robot_double_still_one_real_second_for_one_meter(self):
        # 0.01m * 10 steps * 10 repeats = 1.0m
        primitive = self._primitive(step_dx=0.01, steps=10)
        dummy_env = _DummyEnv()

        tb.env = dummy_env
        tb.set_render_state = lambda pose, joint_angles: None

        tb.SIM_SPEED = 0.5
        tb.SIM_DETAIL = 1.0
        tb.ROBOT_SPEED = 2.0
        tb.REALTIME_CATCHUP = False

        tb.execute_primitive(
            np.array([0.0, 0.0, 0.0]),
            primitive,
            repeats=10,
            render=True,
            primitive_name="timing_sim_half_robot_double",
            show_progress=False,
        )

        self.assertEqual(len(dummy_env.dts), 100)
        real_elapsed = sum(dummy_env.dts)
        self.assertAlmostEqual(real_elapsed, 1.0, places=6)

    def test_sim_detail_half_renders_half_frames_but_keeps_timing(self):
        # 0.01m * 10 steps * 10 repeats = 1.0m
        primitive = self._primitive(step_dx=0.01, steps=10)
        dummy_env = _DummyEnv()

        tb.env = dummy_env
        tb.set_render_state = lambda pose, joint_angles: None

        tb.SIM_SPEED = 1.0
        tb.SIM_DETAIL = 0.5
        tb.ROBOT_SPEED = 1.0
        tb.REALTIME_CATCHUP = False

        tb.execute_primitive(
            np.array([0.0, 0.0, 0.0]),
            primitive,
            repeats=10,
            render=True,
            primitive_name="timing_sim_detail_half",
            show_progress=False,
        )

        self.assertEqual(len(dummy_env.dts), 50)
        real_elapsed = sum(dummy_env.dts)
        self.assertAlmostEqual(real_elapsed, 1.0, places=6)

    def test_run_part1_required_tests_uses_expected_render_timing(self):
        steps = 10
        repeats = 10

        forward = {
            "joint_sequence": np.zeros((steps, 4, 3), dtype=float),
            "body_local_step": np.array([tb.FORWARD_DISTANCE_M / steps, 0.0, 0.0], dtype=float),
            "steps": steps,
            "min_support_legs": 3,
        }
        turn_ccw = {
            "joint_sequence": np.zeros((steps, 4, 3), dtype=float),
            "body_local_step": np.array([0.0, 0.0, np.deg2rad(tb.TURN_ANGLE_DEG) / steps], dtype=float),
            "steps": steps,
            "min_support_legs": 3,
        }
        turn_cw = {
            "joint_sequence": np.zeros((steps, 4, 3), dtype=float),
            "body_local_step": np.array([0.0, 0.0, -np.deg2rad(tb.TURN_ANGLE_DEG) / steps], dtype=float),
            "steps": steps,
            "min_support_legs": 3,
        }

        orig_show_progress = tb.SHOW_PROGRESS
        orig_start = tb.start_robot_environment
        orig_stop = tb.stop_robot_environment
        orig_create = tb.create_motion_primitives
        orig_plot = tb.save_trajectory_plot
        orig_set_render_state = tb.set_render_state

        dummy_env = _DummyEnv()

        try:
            tb.SIM_SPEED = 0.5
            tb.SIM_DETAIL = 1.0
            tb.ROBOT_SPEED = 2.0
            tb.REALTIME_CATCHUP = False
            tb.SHOW_PROGRESS = False

            def fake_start(initial_pose):
                tb.env = dummy_env

            def fake_stop():
                return None

            def fake_create(_):
                return {
                    "forward_10cm": forward,
                    "turn_1deg_ccw": turn_ccw,
                    "turn_1deg_cw": turn_cw,
                }

            tb.start_robot_environment = fake_start
            tb.stop_robot_environment = fake_stop
            tb.create_motion_primitives = fake_create
            tb.save_trajectory_plot = lambda *args, **kwargs: None
            tb.set_render_state = lambda pose, joint_angles: None

            with tempfile.TemporaryDirectory() as temp_dir:
                tb.run_part1_required_tests(Path(temp_dir), render=True, hold_window=False)

            # run_part1 now executes:
            # - Test1: 1 forward run (10 repeats)
            # - Test2: 2 turn runs (10 repeats each)
            # - Test3: 2x(forward + turn) runs (10 repeats each)
            # - Test4 sequence: forward/turn repeats = 4,10,4,10,4,10,4
            # plus 6 reset env.step calls (one per test case).
            forward_repeats_total = 10 + 20 + 16
            turn_repeats_total = 20 + 20 + 30

            expected_step_calls = ((forward_repeats_total + turn_repeats_total) * steps) + 6
            self.assertEqual(len(dummy_env.dts), expected_step_calls)

            dt_forward = tb.primitive_step_render_dt(forward)
            dt_turn = tb.primitive_step_render_dt(turn_ccw)

            expected_real_elapsed = (
                (forward_repeats_total * steps * dt_forward)
                + (turn_repeats_total * steps * dt_turn)
                + (6 * dt_forward)
            )
            self.assertAlmostEqual(sum(dummy_env.dts), expected_real_elapsed, places=9)
        finally:
            tb.SHOW_PROGRESS = orig_show_progress
            tb.start_robot_environment = orig_start
            tb.stop_robot_environment = orig_stop
            tb.create_motion_primitives = orig_create
            tb.save_trajectory_plot = orig_plot
            tb.set_render_state = orig_set_render_state

    def test_realtime_catchup_drops_waits_when_behind(self):
        primitive = self._primitive(step_dx=0.01, steps=10)
        dummy_env = _DummyEnv()

        tb.env = dummy_env
        tb.set_render_state = lambda pose, joint_angles: None

        tb.SIM_SPEED = 1.0
        tb.SIM_DETAIL = 1.0
        tb.ROBOT_SPEED = 1.0
        tb.REALTIME_CATCHUP = True

        orig_perf_counter = tb.time.perf_counter
        call_count = {"n": 0}

        def fake_perf_counter():
            call_count["n"] += 1
            if call_count["n"] == 1:
                return 0.0
            return 1.0

        tb.time.perf_counter = fake_perf_counter
        try:
            tb.execute_primitive(
                np.array([0.0, 0.0, 0.0]),
                primitive,
                repeats=1,
                render=True,
                primitive_name="realtime_catchup",
                show_progress=False,
            )
        finally:
            tb.time.perf_counter = orig_perf_counter

        # In catch-up mode, at least one frame should render and each frame uses
        # a tiny nonzero pause to keep backend event loops stable.
        self.assertGreaterEqual(len(dummy_env.dts), 1)
        self.assertLessEqual(len(dummy_env.dts), 2)
        self.assertTrue(
            all((dt is not None) and (0.0 < dt <= tb.CATCHUP_MIN_STEP_DT) for dt in dummy_env.dts)
        )

    def test_robot_rot_speed_controls_rotation_timing(self):
        steps = 10
        repeats = 10
        primitive = {
            "body_local_step": np.array([0.0, 0.0, np.deg2rad(0.1)], dtype=float),
            "steps": steps,
            "joint_sequence": np.zeros((steps, 4, 3), dtype=float),
        }

        tb.set_render_state = lambda pose, joint_angles: None
        tb.SIM_SPEED = 1.0
        tb.SIM_DETAIL = 1.0
        tb.ROBOT_SPEED = 1.0
        tb.REALTIME_CATCHUP = False

        slow_env = _DummyEnv()
        tb.env = slow_env
        tb.ROBOT_ROT_SPEED = 1.0
        tb.execute_primitive(
            np.array([0.0, 0.0, 0.0]),
            primitive,
            repeats=repeats,
            render=True,
            primitive_name="rot_speed_slow",
            show_progress=False,
        )

        fast_env = _DummyEnv()
        tb.env = fast_env
        tb.ROBOT_ROT_SPEED = 2.0
        tb.execute_primitive(
            np.array([0.0, 0.0, 0.0]),
            primitive,
            repeats=repeats,
            render=True,
            primitive_name="rot_speed_fast",
            show_progress=False,
        )

        self.assertEqual(len(slow_env.dts), repeats * steps)
        self.assertEqual(len(fast_env.dts), repeats * steps)
        self.assertAlmostEqual(sum(slow_env.dts), 10.0, places=6)
        self.assertAlmostEqual(sum(fast_env.dts), 5.0, places=6)


if __name__ == "__main__":
    unittest.main()
