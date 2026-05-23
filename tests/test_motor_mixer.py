# test_motor_mixer.py
# unit tests for motor_mixer.c
# each test sends a single PID command and checks the right motors respond
#
# layout reminder:
# M1 rear-left  (CCW)   M2 front-right (CCW)
# M3 rear-right (CW)    M4 front-left  (CW)

import ctypes
import math

import pytest

from drone_structs import Estimator, PIDOutput, MotorOutput


def test_positive_roll_command_lifts_right_side_motors(drone_lib):
    # roll+ tilts the drone right -> right side motors (M2, M3) push more
    # symmetry check: M2 == M3, M1 == M4
    est = Estimator()

    pids = PIDOutput()
    pids.roll_pid = 1.0

    out = MotorOutput()
    drone_lib.motor_mixer(
        ctypes.byref(out),
        ctypes.byref(est),
        ctypes.byref(pids),
    )

    # right side higher than left
    assert out.spinMotorTwo   > out.spinMotorOne
    assert out.spinMotorThree > out.spinMotorFour
    # symmetric pairs match
    assert out.spinMotorTwo  == pytest.approx(out.spinMotorThree, abs=1e-5)
    assert out.spinMotorOne  == pytest.approx(out.spinMotorFour,  abs=1e-5)


def test_positive_pitch_command_lifts_rear_motors(drone_lib):
    # pitch+ tilts nose up -> rear motors (M1, M3) push more
    est = Estimator()

    pids = PIDOutput()
    pids.pitch_pid = 1.0

    out = MotorOutput()
    drone_lib.motor_mixer(
        ctypes.byref(out),
        ctypes.byref(est),
        ctypes.byref(pids),
    )

    assert out.spinMotorOne   > out.spinMotorTwo
    assert out.spinMotorThree > out.spinMotorFour
    assert out.spinMotorOne  == pytest.approx(out.spinMotorThree, abs=1e-5)
    assert out.spinMotorTwo  == pytest.approx(out.spinMotorFour,  abs=1e-5)


def test_positive_yaw_command_lifts_ccw_motors(drone_lib):
    # yaw+ rotates the drone CCW (viewed from above) -> CCW-spinning motors (M1, M2) push more
    est = Estimator()

    pids = PIDOutput()
    pids.yaw_pid = 1.0

    out = MotorOutput()
    drone_lib.motor_mixer(
        ctypes.byref(out),
        ctypes.byref(est),
        ctypes.byref(pids),
    )

    assert out.spinMotorOne   > out.spinMotorThree
    assert out.spinMotorTwo   > out.spinMotorFour
    assert out.spinMotorOne   == pytest.approx(out.spinMotorTwo,   abs=1e-5)
    assert out.spinMotorThree == pytest.approx(out.spinMotorFour, abs=1e-5)


def test_tilt_compensation_doubles_thrust_at_60_degrees(drone_lib):
    # when tilted, we need more total thrust to stay airborne
    # compensation factor = 1 / (cos(roll) * cos(pitch))
    # at 60 deg roll, cos(60) = 0.5, so thrust doubles
    est_level = Estimator()
    pids = PIDOutput()
    out_level = MotorOutput()
    drone_lib.motor_mixer(
        ctypes.byref(out_level),
        ctypes.byref(est_level),
        ctypes.byref(pids),
    )

    # second call - 60 deg roll, otherwise identical
    est_tilted = Estimator()
    est_tilted.cur_roll = math.radians(60.0)
    out_tilted = MotorOutput()
    drone_lib.motor_mixer(
        ctypes.byref(out_tilted),
        ctypes.byref(est_tilted),
        ctypes.byref(pids),
    )

    # tilted commands should be 2x level commands
    assert out_tilted.spinMotorOne == pytest.approx(2.0 * out_level.spinMotorOne, abs=1e-4)
    assert out_tilted.spinMotorTwo == pytest.approx(2.0 * out_level.spinMotorTwo, abs=1e-4)


def test_motor_outputs_cannot_go_negative(drone_lib):
    # motors can't push down - extreme PID commands that would go negative must clamp at 0
    est = Estimator()

    pids = PIDOutput()
    pids.roll_pid  = 1000.0
    pids.pitch_pid = -1000.0
    pids.yaw_pid   = 1000.0

    out = MotorOutput()
    drone_lib.motor_mixer(
        ctypes.byref(out),
        ctypes.byref(est),
        ctypes.byref(pids),
    )

    assert out.spinMotorOne   >= 0.0
    assert out.spinMotorTwo   >= 0.0
    assert out.spinMotorThree >= 0.0
    assert out.spinMotorFour  >= 0.0