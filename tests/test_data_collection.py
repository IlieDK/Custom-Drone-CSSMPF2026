# test_data_collection.py
# unit tests for data_collection.py
# one test per metric the function returns - overshoot, settling time, RMSE
# plus one for the divergent-response edge case

import math

import numpy as np
import pytest

from data_collection import data_collection


def _make_time_axis(duration_seconds=5.0, dt=0.01):
    # uniformly-spaced time axis from 0 to duration_seconds
    n_samples = int(duration_seconds / dt) + 1
    return np.linspace(0.0, duration_seconds, n_samples)


def test_known_overshoot_is_measured_correctly():
    # build a step that overshoots by exactly 20% of the step size, then settles
    # step size = 0.5, peak = 0.6, so overshoot = 0.1/0.5 = 20%
    t = _make_time_axis()
    target = np.where(t >= 1.0, 0.5, 0.0)

    # response matches target except for a brief excursion to 0.6 just after the step
    angle = target.astype(float).copy()
    overshoot_window = (t > 1.0) & (t < 1.2)
    angle[overshoot_window] = 0.6

    settling, overshoot, rmse = data_collection(target, angle, t)

    assert overshoot == pytest.approx(20.0, abs=0.5)


def test_settling_time_matches_a_known_first_order_response():
    # build a first-order response: angle(t) = setpoint * (1 - exp(-(t-1)/tau))
    # with tau = 0.1, the 2% settling time is roughly 4*tau = 0.4 s
    t = _make_time_axis()
    setpoint_value = 0.5
    target = np.where(t >= 1.0, setpoint_value, 0.0)

    tau = 0.1
    angle = target.astype(float).copy()
    after_step = t >= 1.0
    angle[after_step] = setpoint_value * (1.0 - np.exp(-(t[after_step] - 1.0) / tau))

    settling, overshoot, rmse = data_collection(target, angle, t)

    # solve for when |0.5 - 0.5*(1 - exp(-t/tau))| < 5e-3
    # -> t > tau * ln(100) ~ 0.46 s
    # allow some tolerance for sample timing
    assert 0.40 < settling < 0.55


def test_post_settle_rmse_of_a_clean_response_is_essentially_zero():
    # response is delayed by a few samples then matches exactly
    # post-settle RMSE should be ~0
    t = _make_time_axis()
    target = np.where(t >= 1.0, 0.5, 0.0)

    angle = target.astype(float).copy()
    transient = (t >= 1.0) & (t < 1.05) # response lags for 5 samples
    angle[transient] = 0.0

    settling, overshoot, rmse = data_collection(target, angle, t)

    # response is exact after t = 1.05 so RMSE is essentially zero
    assert rmse == pytest.approx(0.0, abs=1e-6)


def test_response_that_never_settles_returns_nan_for_settling_time():
    # if the response never settles inside the tolerance band, settling_time is NaN
    # catches the case where a divergent controller produces a log we can't score
    t = _make_time_axis()
    target = np.where(t >= 1.0, 0.5, 0.0)

    # persistent oscillation way bigger than the tolerance band
    angle = target.astype(float).copy()
    oscillation = 0.1 * np.sin(10.0 * t)
    angle[t >= 1.0] += oscillation[t >= 1.0]

    settling, overshoot, rmse = data_collection(target, angle, t)

    assert math.isnan(settling)