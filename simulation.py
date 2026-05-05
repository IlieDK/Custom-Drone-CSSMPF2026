import numpy as np
import matplotlib.pyplot as plt
import math
import random
# from noise import imuNoise, bias_instability  # only needed if noise is enabled

import serial
import time

ser = serial.Serial('/dev/cu.usbmodem2103', 115200, timeout=0.05)

# Constants
dt          = 0.01     # Timestep [s]
time_total  = 20      # Time limit [s]

drone_mass  = 4.0      # [kg]   matches motor_mixer.c
g           = 9.81     # [m/s^2]
L           = 0.5      # arm length from frame centre to a motor [m]
arm_x       = L / np.sqrt(2)   # X-config: each motor sits at (±arm_x, ±arm_x)

Ix = Iy     = drone_mass * L * L / 2.0  # rough estimate
Iz          = 2.0 * Ix

k_thrust    = 0.124    # T = k_thrust * motor_command  (matches motor_mixer.c k)
k_yaw       = 0.01     # reaction-torque coefficient

# Setpoint slew rate. Limits how fast the commanded angle can
# change per second
MAX_SLEW = np.deg2rad(90.0)

# Maximum tilt setpoint. The complementary filter blends a gyro
# integral (unbounded) with an atan2-derived accel angle (bounded to
# ±pi). Near ±pi the accel value wraps and the blend stops tracking,
# letting the drone spin freely while the firmware thinks it's at the
# setpoint. Clamping the operator command well below that singularity
# keeps every axis inside the regime where the single-loop PID is valid.
# 60 deg is a typical "angle mode" tilt limit on real flight controllers.
MAX_TILT = np.deg2rad(60.0)

# Slew-limited setpoints (the values actually sent to the firmware)
target_roll  = 0.0
target_pitch = 0.0
target_yaw   = 0.0

# Each motor must produce m*g/4 of thrust at hover.
hover_motor = (drone_mass * g / 4.0) / k_thrust
m1 = m2 = m3 = m4 = hover_motor

# Realistic max motor command: ~3x hover gives plenty of control headroom
# without entering wildly nonphysical territory (fix #10).
M_MIN, M_MAX = 0.0, 3.0 * hover_motor
saturated_warned = False

# State
roll = pitch = yaw = 0.0
wx = wy = wz = 0.0

# Logging
time_log         = []
roll_log         = []
pitch_log        = []
yaw_log          = []
target_roll_log  = []
target_pitch_log = []
target_yaw_log   = []

# Simulation loop
t = 0.0
while t < time_total:
    loop_start = time.time()

    # Motor thrusts (linear model: T = k * m)
    T1 = k_thrust * m1
    T2 = k_thrust * m2
    T3 = k_thrust * m3
    T4 = k_thrust * m4

    # Motor layout and torque directions:
    # M1 rear-left  (CCW)  M2 front-right (CCW)
    # M3 rear-right (CW)   M4 front-left  (CW)
    #
    # Roll+ (right side up): right-side motors (M2, M3) up, left (M1, M4) down
    # Pitch+ (nose up):      rear motors (M1, M3) up, front (M2, M4) down
    # Yaw+ (CCW from above): CCW motors (M1, M2) up, CW motors (M3, M4) down
    roll_torque  = arm_x * ( (T2 + T3) - (T1 + T4) )
    pitch_torque = arm_x * ( (T1 + T3) - (T2 + T4) )
    yaw_torque   = k_yaw * ( (T1 + T2) - (T3 + T4) )

    # Rigid-body rotational dynamics
    wx_dot = roll_torque  / Ix
    wy_dot = pitch_torque / Iy
    wz_dot = yaw_torque   / Iz

    wx += wx_dot * dt
    wy += wy_dot * dt
    wz += wz_dot * dt

    # The proper ZYX kinematic transformation
    sin_roll, cos_roll = np.sin(roll), np.cos(roll)
    sin_pitch, cos_pitch = np.sin(pitch), np.cos(pitch)
    if abs(cos_pitch) < 0.017:                    # gimbal-lock guard, ±89°
        cos_pitch = 0.017 if cos_pitch >= 0 else -0.017
    roll_dot  = wx + sin_roll * (sin_pitch/cos_pitch) * wy + cos_roll * (sin_pitch/cos_pitch) * wz
    pitch_dot = cos_roll * wy - sin_roll * wz
    yaw_dot   = (sin_roll/cos_pitch) * wy + (cos_roll/cos_pitch) * wz

    roll  += roll_dot  * dt
    pitch += pitch_dot * dt
    yaw   += yaw_dot   * dt

    # Body-frame gravity vector (what an accelerometer reads at rest)
    ax = -g * np.sin(pitch)
    ay =  g * np.sin(roll) * np.cos(pitch)
    az =  g * np.cos(roll) * np.cos(pitch)

    # Raw (operator) setpoint — what the user wants:
    raw_roll  = 0.0 if t < 2 else np.deg2rad(25)
    raw_pitch = 0.0 if t < 2 else np.deg2rad(25)
    raw_yaw   = 0.0 

    # Clamp tilt setpoints to the controller's valid envelope (fix #21).
    # Yaw is intentionally not clamped — heading can be any angle.
    raw_roll  = float(np.clip(raw_roll,  -MAX_TILT, MAX_TILT))
    raw_pitch = float(np.clip(raw_pitch, -MAX_TILT, MAX_TILT))

    # Slew-limited setpoint — what the controller actually sees (fix #20).
    # Each axis can change at most MAX_SLEW * dt radians per tick.
    target_roll  += np.clip(raw_roll  - target_roll,  -MAX_SLEW*dt, MAX_SLEW*dt)
    target_pitch += np.clip(raw_pitch - target_pitch, -MAX_SLEW*dt, MAX_SLEW*dt)
    target_yaw   += np.clip(raw_yaw   - target_yaw,   -MAX_SLEW*dt, MAX_SLEW*dt)

    # Send IMU + setpoints + dt
    packet = np.array([
        wx, wy, wz,
        ax, ay, az,
        target_roll, target_pitch, target_yaw,
        dt
    ], dtype=np.float32)
    ser.write(b'IMU' + packet.tobytes())

    # Receive motor commands
    data = ser.read(16)  # 4 floats
    if len(data) == 16:
        m1, m2, m3, m4 = np.frombuffer(data, dtype=np.float32)
    # else: keep previous m1..m4 and let the loop catch up

  
    raw = np.array([m1, m2, m3, m4])
    if not saturated_warned and (np.any(raw < M_MIN) or np.any(raw > M_MAX)):
        print(f"[t={t:5.2f}s] motor saturation: {raw}  -> clamped to [{M_MIN:.1f}, {M_MAX:.1f}]")
        saturated_warned = True
    m1 = float(np.clip(m1, M_MIN, M_MAX))
    m2 = float(np.clip(m2, M_MIN, M_MAX))
    m3 = float(np.clip(m3, M_MIN, M_MAX))
    m4 = float(np.clip(m4, M_MIN, M_MAX))

    print(m1, m2, m3, m4, t)  # for debugging

    # Log
    time_log.append(t)
    roll_log.append(np.rad2deg(roll))
    pitch_log.append(np.rad2deg(pitch))
    yaw_log.append(np.rad2deg(yaw))
    target_roll_log.append(np.rad2deg(target_roll))
    target_pitch_log.append(np.rad2deg(target_pitch))
    target_yaw_log.append(np.rad2deg(target_yaw))

    # Real-time pacing
    elapsed = time.time() - loop_start
    time.sleep(max(0, dt - elapsed))

    t += dt

# Plot
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