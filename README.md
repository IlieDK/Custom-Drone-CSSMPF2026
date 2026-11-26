# Custom Drone - CSSMPF2026

A quadcopter attitude controller built as a Hardware-in-the-Loop (HIL) system: C firmware that runs on an STM32/Arduino board, and a Python simulation that pretends to be the drone. The two talk to each other over a serial cable, so the real control code runs on real hardware while the "drone" lives entirely in software.

The point of the HIL setup is that you can test and tune the actual firmware without crashing an actual drone.

## How it works

The whole thing is a closed loop running at 100 Hz (dt = 0.01 s). Each tick:

1. The simulation computes the drone's physics and generates fake IMU readings (gyro + accelerometer), plus the current setpoints.
2. It sends all of that to the firmware over serial.
3. The firmware estimates the drone's attitude with a complementary filter, runs three PID controllers (roll, pitch, yaw), and mixes the outputs into four motor commands.
4. The motor commands come back over serial.
5. The simulation applies them to the physics model, and the loop repeats.

So the firmware never knows it isn't flying. It just sees sensor data in and sends motor commands out, exactly like it would on a real board.

```
  simulation.py                        firmware (Arduino)
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
- `CSSMPF2026.ino` - the Arduino entry point. Reads the serial packet, runs the pipeline, sends motor commands back. Also handles the session-reset logic (more on that below).
- `estimator.c` / `estimator.h` - complementary filter. Blends the gyro (fast, drifts) and the accelerometer (slow, noisy but absolute) into a clean attitude estimate.
- `pid.c` - one generic PID used by all axes, with anti-windup, output clamping, derivative-on-measurement, a low-pass on the D term, and yaw angle wrapping.
- `motor_mixer.c` - turns the three PID outputs into four motor commands using the X-configuration geometry, with tilt compensation.
- `imu_types.h`, `pid_motor_mixer.h` - the shared struct definitions.

**Simulation (Python, runs on your computer)**
- `simulation.py` - the physics model, sensor simulation, serial communication, and plotting. This is the thing you actually run.

## Running the simulation

You need Python 3, plus numpy, matplotlib and pyserial:

```bash
pip3 install numpy matplotlib pyserial
```

Flash `CSSMPF2026.ino` to the board, then plug it in and check which serial port it shows up as. On Mac it's something like `/dev/cu.usbmodem2103`; on Windows it's a `COM` port. Update the port at the top of `simulation.py`:

```python
ser = serial.Serial('/dev/cu.usbmodem2103', 115200, timeout=0.05)  # change this
```

Then just run it:

```bash
python3 simulation.py
```

It runs the closed loop and pops up three plots - roll, pitch and yaw tracking their setpoints.

## Some design notes worth knowing

A few things that aren't obvious from the code:

**Complementary filter.** Yaw is pure gyro integration - there's no magnetometer, so there's nothing to give yaw an absolute reference. That means yaw will slowly drift over a long flight. Roll and pitch are fine because gravity gives the accelerometer an absolute reference for them.

**Yaw is the slow axis.** The yaw reaction torque coefficient is about 35x weaker than the roll/pitch lever arm, so yaw settles roughly 13x slower than roll or pitch. That's structural, not a tuning problem - you can't just crank the gains because the motors saturate on thrust, not torque.

**Session reset.** The Arduino doesn't always auto-reset when the serial port closes and reopens (depends on the board and OS). Without a reset, the firmware would keep stale state between runs and behave differently each time. So the firmware now detects a gap of more than 200 ms between packets and treats the next packet as a fresh session, wiping all state. This is what makes runs reproducible.

**Slew limiting.** Real drones never see instant setpoint jumps - a pilot's stick or a guidance layer always ramps gradually. The simulation slew-limits its setpoints to 90°/s so the firmware never gets a step input it can't handle.

**Motor command convention.** The motor command represents omega² (propeller speed squared), not RPM. That's why thrust = k * command is linear - the squaring is folded into what the command means, and the ESC would take the square root.

## Known limitations

Being honest about what this doesn't do:

- **Single-loop architecture.** The controller uses one PID loop per axis (attitude only). This works well within its design envelope but combined commands past roughly 22° of tilt can make it tumble. The proper fix is a cascade controller (an inner rate loop plus an outer attitude loop), which isn't built here. The acceleration PIDs in `pid.c` are stubs left in place for exactly this future extension.
- **Yaw drift.** No magnetometer, so yaw has no absolute reference and drifts slowly over time.
- **Linear thrust model.** Fine under the omega² convention above, but it assumes the ESC does the square-root conversion.
- **`motor_radius` is dimensionally loose.** The constant in `motor_mixer.c` doesn't cleanly correspond to SI metres. It works because `k` is used consistently on both sides of the loop (firmware and simulation agree on what a motor command means), but it's best treated as an empirical scaling constant rather than a physically-derived one. For real flight you'd calibrate `k` against an actual thrust stand.

## Context

This is a course project for CSSMPF2026. The firmware is written to run on real hardware; the simulation exists so the control logic can be developed and validated without a physical drone.
