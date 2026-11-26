# Custom Drone - CSSMPF2026

A quadcopter attitude controller built as a Hardware-in-the-Loop (HIL) system: C firmware that runs on a microcontroller board (**TODO: exact board**), and a Python simulation that pretends to be the drone. The two talk to each other over a USB serial cable, so the real control code runs on real hardware while the "drone" lives entirely in software.

The point of the HIL setup is that you can test and tune the actual firmware without crashing an actual drone.

Team project for Computer Science Subject Module Project F2026 (CSSMPF2026). Who did what is at the bottom.

## How it works

The whole thing is a closed loop running at 100 Hz (dt = 0.01 s). Each tick:

1. The simulation computes the drone's physics and generates fake IMU readings (gyro + accelerometer), plus the current setpoints.
2. It sends all of that to the firmware over serial.
3. The firmware estimates the drone's attitude with a complementary filter, runs three PID controllers (roll, pitch, yaw), and mixes the outputs into four motor commands.
4. The motor commands come back over serial.
5. The simulation applies them to the physics model, and the loop repeats.

So the firmware never knows it isn't flying. It just sees sensor data in and sends motor commands out, exactly like it would on a real drone.

```
  simulation.py                        firmware (board)
 ┌──────────────┐    IMU + setpoints  ┌──────────────────┐
 │  physics     │ ──────────────────► │  estimator       │
 │  model       │                     │  (comp. filter)  │
 │              │                     │       │          │
 │              │                     │  3x PID          │
 │              │                     │       │          │
 │              │ ◄────────────────── │  motor mixer     │
 └──────────────┘    4 motor cmds     └──────────────────┘
```

## What's in here

**Firmware (C, runs on the board)**
- `CSSMPF2026.ino` - the Arduino entry point. Reads the serial packet, runs the pipeline, sends motor commands back. Also handles the session reset (see below).
- `estimator.c` / `estimator.h` - complementary filter. Blends the gyro (fast, drifts) and the accelerometer (slow, noisy but absolute) into one attitude estimate.
- `pid.c` - one generic PID used by all axes, with anti-windup, output clamping, derivative-on-measurement, a low-pass on the D term, and yaw angle wrapping.
- `motor_mixer.c` - turns the three PID outputs into four motor commands using the X-configuration geometry, with tilt compensation.
- `imu_types.h`, `pid_motor_mixer.h` - the shared struct definitions.

**Simulation (Python, runs on your computer)**
- `simulation.py` - the physics model, sensor simulation, serial communication, and plotting. This is the file you run.
- `noise.py` - an IMU noise model (white noise + bias drift). Not used yet: the import in `simulation.py` is commented out, so the simulated sensors are perfect.

## Running the simulation

You need Python 3, plus numpy, matplotlib and pyserial:

```bash
pip3 install numpy matplotlib pyserial
```

Flash `CSSMPF2026.ino` to the board, then plug it in and check which serial port it shows up as. On Mac it's something like `/dev/cu.usbmodem2103`; on Windows it's a `COM` port. Update the port at the top of `simulation.py`:

```python
ser = serial.Serial('/dev/cu.usbmodem2103', 115200, timeout=0.05)  # change this
```

Then run it:

```bash
python3 simulation.py
```

It runs the closed loop for 10 seconds and then shows three plots: roll, pitch and yaw against their setpoints. By default the drone holds level for 2 s and then gets a 25° roll command. To try something else, change the `raw_roll`, `raw_pitch` and `raw_yaw` lines in the main loop.

## Design notes

A few things that aren't obvious from the code:

**Complementary filter.** Yaw is pure gyro integration. There's no magnetometer, so nothing gives yaw an absolute reference, and it slowly drifts over a long flight. Roll and pitch are fine because gravity gives the accelerometer an absolute reference for them.

**Yaw is the slow axis.** Roll and pitch are turned by motor thrust acting on the arms. Yaw is only turned by the propellers' drag torque, and in `simulation.py` that is about 35x weaker (`k_yaw` = 0.01 vs. the arm lever `arm_x` ≈ 0.35 m). So a 25° step settles in about 0.4 s on roll, but takes about 7 s on yaw. Higher yaw gains don't fix this: during a yaw step, two motors already drop to zero thrust.

**Session reset.** The board doesn't always reset when the serial port closes and reopens (it depends on the board and OS). Without a reset, the firmware would keep stale state between runs and behave differently each time. So the firmware treats a gap of more than 200 ms between packets as the start of a new session and wipes all state. This makes runs reproducible.

**Slew limiting.** Real drones never see instant setpoint jumps - a pilot's stick or a guidance layer always ramps gradually. The simulation limits how fast its setpoints change to 90°/s, so the firmware never gets a step input.

**Motor command convention.** The motor command represents omega² (propeller speed squared), not RPM. That's why thrust = k * command is linear - the squaring is folded into what the command means, and the ESC would take the square root.

## Known limitations

- **Single-loop architecture.** The controller uses one PID loop per axis (attitude only). A command on one axis works up to the 60° tilt limit in `simulation.py`, but combined roll + pitch commands overshoot badly from about 40° each, and at about 50° each the drone tumbles. The proper fix is a cascade controller (an inner rate loop plus an outer attitude loop), which isn't built here.
- **Unused acceleration PIDs.** `pid.c` also has PIDs for acceleration in x, y and z. They are written but not called, left in place for a future position controller.
- **Yaw drift.** No magnetometer, so yaw has no absolute reference and drifts slowly over time.
- **Perfect sensors.** The simulated IMU has no noise or bias (see `noise.py`).
- **Linear thrust model.** Fine under the omega² convention above, but it assumes the ESC does the square-root conversion.
- **`motor_radius` is not a real size.** The constant in `motor_mixer.c` is 3.3, which would be a 3.3 m propeller. It works because `k` is the same on both sides of the loop (firmware and simulation agree on what a motor command means), so treat it as a scaling constant, not a physical one. For real flight you'd calibrate `k` against a thrust stand.

## Team

Team of five: Elena, Gabriel, Ilie, Vsevolod and Yeongyun.

- **Ilie Baciu (Team Leader):** team coordination, PID development, simulation QC.
