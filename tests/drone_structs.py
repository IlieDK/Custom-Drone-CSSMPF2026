# drone_structs.py
# Python copies of the C structs - lets ctypes pass them into the C functions
# field order and types must match the C structs exactly

import ctypes


# mirror of IMU_Data in imu_types.h
class IMUData(ctypes.Structure):
    _fields_ = [
        ("ax", ctypes.c_float),
        ("ay", ctypes.c_float),
        ("az", ctypes.c_float),
        ("gx", ctypes.c_float),
        ("gy", ctypes.c_float),
        ("gz", ctypes.c_float),
    ]


# mirror of Estimator_Config in imu_types.h
class EstimatorConfig(ctypes.Structure):
    _fields_ = [
        ("ax_bias", ctypes.c_float),
        ("ay_bias", ctypes.c_float),
        ("az_bias", ctypes.c_float),
        ("gx_bias", ctypes.c_float),
        ("gy_bias", ctypes.c_float),
        ("gz_bias", ctypes.c_float),
        ("alpha",   ctypes.c_float),
    ]


# mirror of `estimator` in pid_motor_mixer.h - the big shared struct
class Estimator(ctypes.Structure):
    _fields_ = [
        ("cur_roll",       ctypes.c_float), # current estimated angles
        ("cur_pitch",      ctypes.c_float),
        ("cur_yaw",        ctypes.c_float),
        ("roll_setpoint",  ctypes.c_float), # what we want
        ("pitch_setpoint", ctypes.c_float),
        ("yaw_setpoint",   ctypes.c_float),
        ("cur_accX",       ctypes.c_float), # current body accels
        ("cur_accY",       ctypes.c_float),
        ("cur_accZ",       ctypes.c_float),
        ("accX_setpoint",  ctypes.c_float), # reserved - not used yet
        ("accY_setpoint",  ctypes.c_float),
        ("accZ_setpoint",  ctypes.c_float),
        ("dt",             ctypes.c_float), # timestep
    ]


# mirror of `pid` in pid_motor_mixer.h - PID outputs
class PIDOutput(ctypes.Structure):
    _fields_ = [
        ("roll_pid",  ctypes.c_float),
        ("pitch_pid", ctypes.c_float),
        ("yaw_pid",   ctypes.c_float),
        ("accX_pid",  ctypes.c_float),
        ("accY_pid",  ctypes.c_float),
        ("accZ_pid",  ctypes.c_float),
    ]


# mirror of `state` in pid_motor_mixer.h - internal PID memory
class PIDState(ctypes.Structure):
    _fields_ = [
        ("prev_integral",    ctypes.c_float),
        ("prev_measurement", ctypes.c_float),
        ("d_filtered",       ctypes.c_float),
    ]


# mirror of `gain` in pid_motor_mixer.h - PID tuning
class PIDGain(ctypes.Structure):
    _fields_ = [
        ("p_gain", ctypes.c_float),
        ("i_gain", ctypes.c_float),
        ("d_gain", ctypes.c_float),
    ]


# mirror of `output` in pid_motor_mixer.h - motor mixer output
class MotorOutput(ctypes.Structure):
    _fields_ = [
        ("spinMotorOne",   ctypes.c_float),
        ("spinMotorTwo",   ctypes.c_float),
        ("spinMotorThree", ctypes.c_float),
        ("spinMotorFour",  ctypes.c_float),
    ]