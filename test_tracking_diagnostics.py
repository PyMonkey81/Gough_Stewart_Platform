"""Regression tests: TIK/PIK pose vs MATLAB, single L0 subtraction, and log lines."""

import os
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np

from gui.dialog.parameterdiag import DEFAULT_PARAMETERS, load_parameters
from gui.screen.MainWindow import MainWindow, length_to_percent
from kinematics.inverse import PIK, TIK, inverse_kinematics

L0_TEST = 247.41
STROKE_TEST = 100.0
RT_TEST = 90.0
Y_FIXED = np.deg2rad([9.9, 10.10])
Q_MATLAB = np.array([247.3, 283.0, 361.2, 373.4, 308.0, 258.9])
EXT_MATLAB = np.array([-0.1, 35.6, 113.8, 126.0, 60.6, 11.5])


def make_target(l0=L0_TEST, stroke=STROKE_TEST, rt=RT_TEST):
    target = SimpleNamespace(l0=l0, stroke=stroke, rt=rt, a0=0.0, log_event=Mock())
    target._ik = lambda y: MainWindow._ik(target, y)
    target._to_percent = lambda q: MainWindow._to_percent(target, q)
    target._log_lengths = lambda *a: MainWindow._log_lengths(target, *a)
    return target


def matlab_R(a, b, a0=0.0):
    return np.array([
        [np.sin(a)*np.sin(a-a0)+np.cos(a)*np.cos(a-a0)*np.sin(b),
         np.cos(a-a0)*np.sin(a)-np.cos(a)*np.sin(a-a0)*np.sin(b), np.cos(a)*np.cos(b)],
        [np.cos(a)*np.sin(a-a0)-np.cos(a-a0)*np.sin(a)*np.sin(b),
         np.cos(a-a0)*np.cos(a)+np.sin(a)*np.sin(a-a0)*np.sin(b), -np.cos(b)*np.sin(a)],
        [-np.cos(a-a0)*np.cos(b), np.cos(b)*np.sin(a-a0), np.sin(b)],
    ])


class PoseMatchesMatlabTests(unittest.TestCase):
    def test_rotation_is_matlab_tik_with_beta_as_is(self):
        _, R = TIK(Y_FIXED, rt=RT_TEST)
        np.testing.assert_allclose(R, matlab_R(*Y_FIXED))

    def test_fixed_pose_lengths_match_matlab_with_rt_90(self):
        target = make_target()
        q_mm, da, R = target._ik(Y_FIXED)
        np.testing.assert_allclose(da, [0, 0, 331.94] + RT_TEST * R[:, 2])
        np.testing.assert_allclose(q_mm, PIK(da, R))
        np.testing.assert_allclose(q_mm, Q_MATLAB, atol=0.1)
        np.testing.assert_allclose(q_mm - L0_TEST, EXT_MATLAB, atol=0.1)

    def test_saved_parameters_supply_rt_and_l0(self):
        params = load_parameters()
        self.assertEqual(params["RT"], RT_TEST)
        self.assertEqual(params["L0"], L0_TEST)

    def test_inverse_kinematics_forwards_rt(self):
        q0, _, _ = inverse_kinematics(Y_FIXED, rt=0.0)
        q90, _, _ = inverse_kinematics(Y_FIXED, rt=RT_TEST)
        self.assertFalse(np.allclose(q0, q90))


class LengthToPercentTests(unittest.TestCase):
    def test_single_subtraction_and_saturation(self):
        pct = length_to_percent(Q_MATLAB, L0_TEST, STROKE_TEST)
        np.testing.assert_allclose(pct, np.clip(EXT_MATLAB, 0, 100), atol=0.1)
        self.assertEqual(pct[2], 100.0)
        self.assertEqual(pct[3], 100.0)
        self.assertEqual(pct[0], 0.0)

    def test_dialog_default_has_no_legacy_offset(self):
        self.assertNotIn("OFFSET_ACTUADOR", DEFAULT_PARAMETERS)


class LogLineTests(unittest.TestCase):
    def test_line_has_r_col_real_q_and_saturated_pct(self):
        target = make_target()
        q_mm, _, R = target._ik(Y_FIXED)
        MainWindow._log_lengths(target, "JOG TAREA", q_mm, target._to_percent(q_mm), R)
        line = target.log_event.call_args.args[0]
        self.assertTrue(line.startswith("JOG TAREA: R[:,2]=[0.9698, -0.1693, 0.1754] q_mm=["))
        self.assertIn("361.2", line)
        self.assertIn("373.4", line)
        self.assertIn("100.0, 100.0", line)
        self.assertTrue(line.endswith("L0=247.41 carrera=100.00"))

    def test_approach_tick_uses_rt_and_logs_once(self):
        target = make_target()
        target.traj_gen = SimpleNamespace(
            cfg=SimpleNamespace(t_tracking_end=490.0, t_home_end=60.0),
            phase="home", step=Mock(return_value=(Y_FIXED, np.zeros(2))),
        )
        target.t, target.dt, target._last_draw = 30.0, 0.03, 30.0
        target.lbl_track_phase = Mock()
        target.update_actuator_status = Mock()
        target._maybe_send = Mock()
        target.paro = Mock()
        MainWindow._auto_tick(target)
        target.paro.assert_not_called()
        np.testing.assert_allclose(target.q_mm, Q_MATLAB, atol=0.1)
        expected = np.clip((Q_MATLAB - L0_TEST) / STROKE_TEST * 100, 0, 100)
        np.testing.assert_allclose(target._maybe_send.call_args.args[0], expected, atol=0.1)
        self.assertEqual(target.log_event.call_count, 1)
        self.assertTrue(target.log_event.call_args.args[0].startswith("APPROACH: R[:,2]="))


if __name__ == "__main__":
    unittest.main()
