# test_estimator.py
# unit tests for estimator.c
# each test sets up an IMU reading, runs one update, checks the result

import ctypes
import math

import pytest

from drone_structs import IMUData, EstimatorConfig, Estimator


G = 9.81 # gravity m/s^2


def test_level_and_stationary_gives_zero_attitude(drone_lib):
    # level + not rotating means all three angles should stay at 0
    drone_lib.Estimator_Init()

    # accelerometer reads (0, 0, g) when level, gyro reads (0, 0, 0) when still
    imu = IMUData()
    imu.ax = 0.0
    imu.ay = 0.0
    imu.az = G
    imu.gx = 0.0
    imu.gy = 0.0
    imu.gz = 0.0

    # run one tick
    est = Estimator()
    est.dt = 0.01
    drone_lib.Estimator_Update(ctypes.byref(imu), ctypes.byref(est))

    # all three angles should be ~zero
    assert est.cur_roll  == pytest.approx(0.0, abs=1e-6)
    assert est.cur_pitch == pytest.approx(0.0, abs=1e-6)
    assert est.cur_yaw   == pytest.approx(0.0, abs=1e-6)


def test_accelerometer_recovers_roll_angle(drone_lib):
    # with alpha = 0 the filter trusts accel only
    # at 30 deg roll, ay = g*sin(30), az = g*cos(30) - estimator should report 30 deg
    drone_lib.Estimator_Init()

    # set alpha to 0 so the gyro path doesn't contribute
    config = EstimatorConfig()
    config.alpha = 0.0
    drone_lib.Estimator_SetConfig(ctypes.byref(config))

    # build the IMU reading for a drone rolled 30 deg
    roll_in_radians = math.radians(30.0)
    imu = IMUData()
    imu.ax = 0.0
    imu.ay = G * math.sin(roll_in_radians)
    imu.az = G * math.cos(roll_in_radians)

    est = Estimator()
    est.dt = 0.01
    drone_lib.Estimator_Update(ctypes.byref(imu), ctypes.byref(est))

    # should recover exactly 30 deg roll, 0 deg pitch
    assert est.cur_roll  == pytest.approx(roll_in_radians, abs=1e-5)
    assert est.cur_pitch == pytest.approx(0.0, abs=1e-5)


def test_accelerometer_recovers_pitch_angle(drone_lib):
    # same idea but on pitch
    # at -20 deg pitch (nose down), ax = -g*sin(-20), az = g*cos(-20)
    drone_lib.Estimator_Init()

    config = EstimatorConfig()
    config.alpha = 0.0
    drone_lib.Estimator_SetConfig(ctypes.byref(config))

    pitch_in_radians = math.radians(-20.0)
    imu = IMUData()
    imu.ax = -G * math.sin(pitch_in_radians)
    imu.ay = 0.0
    imu.az = G * math.cos(pitch_in_radians)

    est = Estimator()
    est.dt = 0.01
    drone_lib.Estimator_Update(ctypes.byref(imu), ctypes.byref(est))

    assert est.cur_roll  == pytest.approx(0.0, abs=1e-5)
    assert est.cur_pitch == pytest.approx(pitch_in_radians, abs=1e-5)


def test_accelerometer_bias_is_subtracted(drone_lib):
    # configure a bias of 1.0 on ax, send a reading of 1.5
    # cur_accX should be 0.5 (1.5 - 1.0)
    drone_lib.Estimator_Init()

    config = EstimatorConfig()
    config.alpha = 0.0
    config.ax_bias = 1.0
    config.ay_bias = -2.0
    config.az_bias = 3.0
    drone_lib.Estimator_SetConfig(ctypes.byref(config))

    # raw reading - bias = expected corrected value
    imu = IMUData()
    imu.ax = 1.5        # 1.5 - 1.0 = 0.5
    imu.ay = -1.5       # -1.5 - (-2.0) = 0.5
    imu.az = G + 3.0    # (g + 3.0) - 3.0 = g

    est = Estimator()
    est.dt = 0.01
    drone_lib.Estimator_Update(ctypes.byref(imu), ctypes.byref(est))

    # check the corrected values
    assert est.cur_accX == pytest.approx(0.5, abs=1e-6)
    assert est.cur_accY == pytest.approx(0.5, abs=1e-6)
    assert est.cur_accZ == pytest.approx(G,   abs=1e-6)


def test_yaw_is_pure_gyro_integration(drone_lib):
    # yaw has no accel reference (gravity is along yaw) so it's just gyro integration
    # gz = 1 rad/s for dt = 0.1 s should give yaw = 0.1 rad
    drone_lib.Estimator_Init()

    imu = IMUData()
    imu.az = G          # keep accel sane
    imu.gz = 1.0        # 1 rad/s of yaw rate

    est = Estimator()
    est.dt = 0.1
    drone_lib.Estimator_Update(ctypes.byref(imu), ctypes.byref(est))

    assert est.cur_yaw == pytest.approx(0.1, abs=1e-6)


def test_zero_timestep_freezes_the_estimate(drone_lib):
    # if dt is 0 the gyro can't integrate - the estimator should hold the previous values
    # check by running a normal update then a dt = 0 update with a huge gyro signal
    drone_lib.Estimator_Init()

    # use alpha = 1 so only the gyro path contributes
    config = EstimatorConfig()
    config.alpha = 1.0
    drone_lib.Estimator_SetConfig(ctypes.byref(config))

    # first update - sets up a non-zero estimate
    imu_first = IMUData()
    imu_first.az = G
    imu_first.gz = 1.0
    est_first = Estimator()
    est_first.dt = 0.1
    drone_lib.Estimator_Update(ctypes.byref(imu_first), ctypes.byref(est_first))

    # second update - dt = 0, huge gyro signal that WOULD cause big change if dt > 0
    imu_second = IMUData()
    imu_second.az = G
    imu_second.gx = 10.0
    imu_second.gy = 10.0
    imu_second.gz = 10.0
    est_second = Estimator()
    est_second.dt = 0.0
    drone_lib.Estimator_Update(ctypes.byref(imu_second), ctypes.byref(est_second))

    # second estimate should equal the first
    assert est_second.cur_roll  == pytest.approx(est_first.cur_roll,  abs=1e-6)
    assert est_second.cur_pitch == pytest.approx(est_first.cur_pitch, abs=1e-6)
    assert est_second.cur_yaw   == pytest.approx(est_first.cur_yaw,   abs=1e-6)


def test_complementary_filter_blends_gyro_and_accelerometer(drone_lib):
    # the filter combines gyro and accel: cur = alpha*gyro + (1-alpha)*accel
    # with alpha = 0.5 and zero gyro, output should be half the accel-derived angle
    drone_lib.Estimator_Init()

    config = EstimatorConfig()
    config.alpha = 0.5
    drone_lib.Estimator_SetConfig(ctypes.byref(config))

    # drone tilted 20 deg, no rotation
    roll_in_radians = math.radians(20.0)
    imu = IMUData()
    imu.ay = G * math.sin(roll_in_radians)
    imu.az = G * math.cos(roll_in_radians)
    # gyro stays at 0

    est = Estimator()
    est.dt = 0.01
    drone_lib.Estimator_Update(ctypes.byref(imu), ctypes.byref(est))

    expected = 0.5 * roll_in_radians # half the accel-derived value
    assert est.cur_roll == pytest.approx(expected, abs=1e-5)


def test_gyro_to_euler_transformation_at_nonzero_roll(drone_lib):
    # when the drone is rolled, body rates don't equal Euler rates one-to-one
    # this test catches a regression to the small-angle approximation
    # full transformation:
    #   roll_dot  = gx + sin(r)*tan(p)*gy + cos(r)*tan(p)*gz
    #   pitch_dot = cos(r)*gy - sin(r)*gz
    #   yaw_dot   = sin(r)/cos(p)*gy + cos(r)/cos(p)*gz
    drone_lib.Estimator_Init()

    # pure gyro mode so we can predict the integration exactly
    config = EstimatorConfig()
    config.alpha = 1.0
    drone_lib.Estimator_SetConfig(ctypes.byref(config))

    # step 1 - drive prev_roll to 30 deg by sending gx for one tick
    # at zero prior attitude, roll_dot = gx (sin/tan terms vanish)
    imu_setup = IMUData()
    imu_setup.az = G
    imu_setup.gx = math.radians(30.0) # 30 deg/s for dt = 1 s -> 30 deg
    est_setup = Estimator()
    est_setup.dt = 1.0
    drone_lib.Estimator_Update(ctypes.byref(imu_setup), ctypes.byref(est_setup))

    # sanity check - should be at 30 deg roll now
    assert est_setup.cur_roll == pytest.approx(math.radians(30.0), abs=1e-5)

    # step 2 - apply gy = 1 rad/s, see how pitch and yaw evolve
    imu_test = IMUData()
    imu_test.az = G
    imu_test.gy = 1.0
    est_test = Estimator()
    est_test.dt = 1.0
    drone_lib.Estimator_Update(ctypes.byref(imu_test), ctypes.byref(est_test))

    # at roll = 30 deg, pitch = 0:
    #   pitch_dot = cos(30 deg) * 1 = 0.866 rad/s
    #   yaw_dot = sin(30 deg) / cos(0) * 1 = 0.5 rad/s
    # small-angle would give pitch = 1.0, yaw = 0 (wrong)
    expected_pitch = math.cos(math.radians(30.0))
    expected_yaw   = math.sin(math.radians(30.0))

    assert est_test.cur_pitch == pytest.approx(expected_pitch, abs=1e-4)
    assert est_test.cur_yaw   == pytest.approx(expected_yaw,   abs=1e-4)


def test_gyro_bias_is_subtracted_from_each_axis(drone_lib):
    # configure a non-zero bias on each gyro axis, send a reading equal to the bias
    # corrected gyro is then zero, so nothing should integrate
    drone_lib.Estimator_Init()

    config = EstimatorConfig()
    config.alpha = 1.0      # pure gyro mode
    config.gx_bias = 0.5    # rad/s
    config.gy_bias = -0.3
    config.gz_bias = 0.2
    drone_lib.Estimator_SetConfig(ctypes.byref(config))

    # raw gyro = bias, so corrected = 0 on every axis
    imu = IMUData()
    imu.az = G              # keep accel sane
    imu.gx = 0.5
    imu.gy = -0.3
    imu.gz = 0.2

    est = Estimator()
    est.dt = 1.0            # large dt - any uncorrected bias would be obvious
    drone_lib.Estimator_Update(ctypes.byref(imu), ctypes.byref(est))

    # no rotation should have integrated
    assert est.cur_roll  == pytest.approx(0.0, abs=1e-6)
    assert est.cur_pitch == pytest.approx(0.0, abs=1e-6)
    assert est.cur_yaw   == pytest.approx(0.0, abs=1e-6)