# test_pid.py
# unit tests for pid.c
# we test through roll_PID for most cases - all 6 PIDs share the same internal function
# yaw_PID is tested separately because it has the angle-wrap step

import ctypes
import math

import pytest

from drone_structs import Estimator, PIDOutput, PIDState, PIDGain


def test_proportional_term_equals_kp_times_error(drone_lib):
    # with I = 0 and D = 0, output is just P * error
    # error = 0.5 - 0.2 = 0.3, output = 10 * 0.3 = 3.0
    gains = PIDGain()
    gains.p_gain = 10.0
    gains.i_gain = 0.0
    gains.d_gain = 0.0

    state = PIDState()

    est = Estimator()
    est.cur_roll = 0.2
    est.roll_setpoint = 0.5
    est.dt = 0.01

    out = PIDOutput()
    drone_lib.roll_PID(
        ctypes.byref(gains),
        ctypes.byref(est),
        ctypes.byref(out),
        ctypes.byref(state),
    )

    assert out.roll_pid == pytest.approx(3.0, abs=1e-6)


def test_output_is_clamped_to_plus_minus_200(drone_lib):
    # output is hard-clamped to [-200, +200] so extreme errors don't produce extreme commands
    # with Kp = 1000 and error = 1.0, raw output would be 1000 -> expect 200
    gains = PIDGain()
    gains.p_gain = 1000.0
    gains.i_gain = 0.0
    gains.d_gain = 0.0

    state = PIDState()

    est = Estimator()
    est.cur_roll = 0.0
    est.roll_setpoint = 1.0
    est.dt = 0.01

    out = PIDOutput()
    drone_lib.roll_PID(
        ctypes.byref(gains),
        ctypes.byref(est),
        ctypes.byref(out),
        ctypes.byref(state),
    )

    # 1000 * 1.0 = 1000, clamped to 200
    assert out.roll_pid == pytest.approx(200.0, abs=1e-6)


def test_integral_is_clamped_to_prevent_windup(drone_lib):
    # large persistent error would let the integral grow huge (windup)
    # the integral is clamped to [-1000, +1000] to prevent this
    # one tick with error = 100000 and dt = 1 - integral should hit the clamp at 1000
    gains = PIDGain()
    gains.p_gain = 0.0      # I-only PID
    gains.i_gain = 1.0
    gains.d_gain = 0.0

    state = PIDState()

    est = Estimator()
    est.cur_roll = 0.0
    est.roll_setpoint = 100000.0    # huge error
    est.dt = 1.0

    out = PIDOutput()
    drone_lib.roll_PID(
        ctypes.byref(gains),
        ctypes.byref(est),
        ctypes.byref(out),
        ctypes.byref(state),
    )

    # state struct stores the integral - check it's clamped
    assert state.prev_integral == pytest.approx(1000.0, abs=1e-6)


def test_derivative_responds_negatively_to_a_rising_measurement(drone_lib):
    # D works on the rate of change of the measurement (not the error)
    # contribution is negative-signed, so a rising measurement -> negative output
    #
    # walk through:
    #   1st call sets prev_measurement to 0
    #   2nd call: d_meas = (1 - 0) / 1 = 1
    #   low-pass: d_filtered = alpha*0 + (1-alpha)*1 = (1 - alpha)
    #   output = -Kd * (1 - alpha)
    #
    # must match D_FILTER_ALPHA in pid.c - if you change it there, update here too
    D_FILTER_ALPHA_FROM_PID_C = 0.3
    expected_output = -(1.0 - D_FILTER_ALPHA_FROM_PID_C) # = -0.7

    gains = PIDGain()
    gains.p_gain = 0.0      # D-only PID
    gains.i_gain = 0.0
    gains.d_gain = 1.0

    state = PIDState()

    est = Estimator()
    est.roll_setpoint = 0.0
    est.dt = 1.0

    # first call - sets prev_measurement = 0
    est.cur_roll = 0.0
    out = PIDOutput()
    drone_lib.roll_PID(
        ctypes.byref(gains),
        ctypes.byref(est),
        ctypes.byref(out),
        ctypes.byref(state),
    )

    # second call - measurement jumps to 1.0
    est.cur_roll = 1.0
    drone_lib.roll_PID(
        ctypes.byref(gains),
        ctypes.byref(est),
        ctypes.byref(out),
        ctypes.byref(state),
    )

    assert out.roll_pid == pytest.approx(expected_output, abs=1e-6)


def test_yaw_error_wraps_to_the_short_way_around(drone_lib):
    # yaw is circular - +179 and -179 are 2 deg apart, not 358
    # yaw_PID should detect this and use the +2 deg error
    gains = PIDGain()
    gains.p_gain = 1.0      # P-only, gain 1 so output = error
    gains.i_gain = 0.0
    gains.d_gain = 0.0

    state = PIDState()

    est = Estimator()
    est.yaw_setpoint = math.radians(-179.0)
    est.cur_yaw      = math.radians(179.0)
    est.dt = 0.01

    out = PIDOutput()
    drone_lib.yaw_PID(
        ctypes.byref(gains),
        ctypes.byref(est),
        ctypes.byref(out),
        ctypes.byref(state),
    )

    # wrapped error is +2 deg, output should be +2 deg in radians
    expected_output = math.radians(2.0)
    assert out.yaw_pid == pytest.approx(expected_output, abs=1e-5)