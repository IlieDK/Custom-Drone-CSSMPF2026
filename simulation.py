# simulation.py
# software-in-the-loop test rig
# simulates the drone's physics, talks to the firmware over serial

import numpy as np
import matplotlib.pyplot as plt
import math
import random
# from noise import imuNoise, bias_instability  # optional - if we add IMU noise later

import serial
import time

ser = serial.Serial('/dev/cu.usbmodem2103', 115200, timeout=0.05) # change the port to match your machine

# simulation timing
dt          = 0.01     # timestep in seconds (100 Hz)
time_total  = 10       # how long to run

# drone physical constants - must match motor_mixer.c
drone_mass  = 4.0      # kg
g           = 9.81     # m/s^2
L           = 0.5      # arm length, centre to motor, in metres
arm_x       = L / np.sqrt(2)   # in X-config each motor is at (+/- arm_x, +/- arm_x)

# moments of inertia (rough estimates)
Ix = Iy     = drone_mass * L * L / 2.0
Iz          = 2.0 * Ix

# motor model constants
k_thrust    = 0.124    # thrust = k * command (command is omega^2, see motor_mixer.c)
k_yaw       = 0.01     # yaw reaction torque coefficient - much weaker than the roll/pitch lever

# setpoint slew rate - max change per second
MAX_SLEW = np.deg2rad(90.0)

# max tilt setpoint - keep the controller in its linear regime
# real flight controllers in "angle mode" cap at around this value
MAX_TILT = np.deg2rad(60.0)

# slew-limited setpoints - what the firmware actually sees
target_roll  = 0.0
target_pitch = 0.0
target_yaw   = 0.0

# motor commands - start at hover
hover_motor = (drone_mass * g / 4.0) / k_thrust # each motor produces m*g/4 at hover
m1 = m2 = m3 = m4 = hover_motor

# motor command clamp - 3x hover gives plenty of headroom
M_MIN, M_MAX = 0.0, 3.0 * hover_motor
saturated_warned = False # only print the saturation warning once

# physical state
roll = pitch = yaw = 0.0
wx = wy = wz = 0.0

# log buffers - one entry per tick
time_log         = []
roll_log         = []
pitch_log        = []
yaw_log          = []
target_roll_log  = []
target_pitch_log = []
target_yaw_log   = []


# main simulation loop
t = 0.0
while t < time_total:
    loop_start = time.time() # used for real-time pacing at the bottom

    # motor thrusts (linear model: T = k * command)
    T1 = k_thrust * m1
    T2 = k_thrust * m2
    T3 = k_thrust * m3
    T4 = k_thrust * m4

    # body torques - signs follow the X-config layout in motor_mixer.c
    # roll+ = right side up (M2, M3 push)
    # pitch+ = nose up (M1, M3 push)
    # yaw+ = CCW from above (M1, M2 push)
    roll_torque  = arm_x * ( (T2 + T3) - (T1 + T4) )
    pitch_torque = arm_x * ( (T1 + T3) - (T2 + T4) )
    yaw_torque   = k_yaw * ( (T1 + T2) - (T3 + T4) )

    # rigid-body rotational dynamics - integrate angular acceleration
    wx_dot = roll_torque  / Ix
    wy_dot = pitch_torque / Iy
    wz_dot = yaw_torque   / Iz

    wx += wx_dot * dt
    wy += wy_dot * dt
    wz += wz_dot * dt

    # convert body angular velocity to Euler angle rates (ZYX convention)
    # must match the transformation in estimator.c
    sin_roll, cos_roll = np.sin(roll), np.cos(roll)
    sin_pitch, cos_pitch = np.sin(pitch), np.cos(pitch)
    if abs(cos_pitch) < 0.017: # gimbal lock guard
        cos_pitch = 0.017 if cos_pitch >= 0 else -0.017
    roll_dot  = wx + sin_roll * (sin_pitch/cos_pitch) * wy + cos_roll * (sin_pitch/cos_pitch) * wz
    pitch_dot = cos_roll * wy - sin_roll * wz
    yaw_dot   = (sin_roll/cos_pitch) * wy + (cos_roll/cos_pitch) * wz

    # integrate Euler rates into Euler angles
    roll  += roll_dot  * dt
    pitch += pitch_dot * dt
    yaw   += yaw_dot   * dt

    # what the accelerometer would read at this attitude (gravity in body frame)
    ax = -g * np.sin(pitch)
    ay =  g * np.sin(roll) * np.cos(pitch)
    az =  g * np.cos(roll) * np.cos(pitch)

    # operator setpoint - what we want the drone to do
    raw_roll  = 0.0 if t < 2 else np.deg2rad(25)
    raw_pitch = 0.0
    raw_yaw   = 0.0 

    # clamp tilt setpoints to safe range (yaw is fine to be anything)
    raw_roll  = float(np.clip(raw_roll,  -MAX_TILT, MAX_TILT))
    raw_pitch = float(np.clip(raw_pitch, -MAX_TILT, MAX_TILT))

    # slew-limit the setpoint - max change per tick
    target_roll  += np.clip(raw_roll  - target_roll,  -MAX_SLEW*dt, MAX_SLEW*dt)
    target_pitch += np.clip(raw_pitch - target_pitch, -MAX_SLEW*dt, MAX_SLEW*dt)
    target_yaw   += np.clip(raw_yaw   - target_yaw,   -MAX_SLEW*dt, MAX_SLEW*dt)

    # build and send the IMU packet over serial
    packet = np.array([
        wx, wy, wz,                                     # gyro
        ax, ay, az,                                     # accel
        target_roll, target_pitch, target_yaw,          # setpoints
        dt                                              # timestep
    ], dtype=np.float32)
    ser.write(b'IMU' + packet.tobytes())                # 'IMU' header + 40 bytes of payload

    # read the firmware's response (4 motor commands = 16 bytes)
    data = ser.read(16)
    if len(data) == 16:
        m1, m2, m3, m4 = np.frombuffer(data, dtype=np.float32)
    # else - timeout, keep the previous motor commands and let the next tick catch up

    # check for motor saturation - print a warning the first time it happens
    raw = np.array([m1, m2, m3, m4])
    if not saturated_warned and (np.any(raw < M_MIN) or np.any(raw > M_MAX)):
        print(f"[t={t:5.2f}s] motor saturation: {raw}  -> clamped to [{M_MIN:.1f}, {M_MAX:.1f}]")
        saturated_warned = True

    # clamp motor commands to their physical range
    m1 = float(np.clip(m1, M_MIN, M_MAX))
    m2 = float(np.clip(m2, M_MIN, M_MAX))
    m3 = float(np.clip(m3, M_MIN, M_MAX))
    m4 = float(np.clip(m4, M_MIN, M_MAX))

    # log this tick
    time_log.append(t)
    roll_log.append(np.rad2deg(roll))
    pitch_log.append(np.rad2deg(pitch))
    yaw_log.append(np.rad2deg(yaw))
    target_roll_log.append(np.rad2deg(target_roll))
    target_pitch_log.append(np.rad2deg(target_pitch))
    target_yaw_log.append(np.rad2deg(target_yaw))

    # real-time pacing - sleep out the rest of the dt window
    elapsed = time.time() - loop_start
    time.sleep(max(0, dt - elapsed))

    t += dt


# step response metrics - settling time, overshoot, RMSE per axis
from data_collection import data_collection

print()
print("axis   | settling [s] | overshoot [%] | RMSE [rad]")
print("-" * 50)
for name, target_log_, actual_log_ in [
    ("roll",  target_roll_log,  roll_log),
    ("pitch", target_pitch_log, pitch_log),
    ("yaw",   target_yaw_log,   yaw_log),
]:
    # convert back to radians - the logs are stored in degrees for the plot
    s, o, r = data_collection(
        np.deg2rad(target_log_),
        np.deg2rad(actual_log_),
        time_log,
    )
    print(f"{name:<6} | {s:>11.3f}  | {o:>12.2f}  | {r:>9.5f}")


# plotting - one subplot per axis, response vs target
fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
axs[0].plot(time_log, roll_log, label="roll"); axs[0].plot(time_log, target_roll_log, "--", label="target")
axs[1].plot(time_log, pitch_log, label="pitch"); axs[1].plot(time_log, target_pitch_log, "--", label="target")
axs[2].plot(time_log, yaw_log, label="yaw"); axs[2].plot(time_log, target_yaw_log, "--", label="target")
for ax_ in axs:
    ax_.set_ylabel("deg")
    ax_.legend(); ax_.grid()
axs[2].set_xlabel("time [s]")
plt.tight_layout()
plt.show()