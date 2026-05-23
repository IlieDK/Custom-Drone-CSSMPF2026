# data_collection.py
# post-process a logged step response into three step-response metrics:
# settling time, overshoot %, and post-settle RMSE
# returns the average across all setpoint changes in the log

import numpy as np


# what counts as a setpoint change - anything bigger than floating-point noise
SETPOINT_CHANGE_THRESHOLD = 1e-6

# tolerance band around the setpoint - a sample inside this is "settled"
# textbook uses +/- 2%, for typical 10 deg steps 5e-3 rad is ~2.9% which is close enough
SETTLE_TOLERANCE = 5e-3


def data_collection(target_log, angle_log, logged_time):

    # convert inputs to arrays in case they came in as lists
    target_log = np.asarray(target_log)
    angle_log = np.asarray(angle_log)
    logged_time = np.asarray(logged_time)

    # find where the setpoint changes - look for non-zero gradient
    target_gradient = np.gradient(target_log, logged_time)
    target_changing = np.abs(target_gradient) > SETPOINT_CHANGE_THRESHOLD

    # a step event = sample is changing but the previous one wasn't (leading edge)
    setpoint_changes = np.where(
        target_changing & np.concatenate(([True], ~target_changing[:-1]))
    )[0]

    # accumulators - one entry per detected step
    settling_times = []
    overshoots = []
    rmse_values = []

    for i, change_idx in enumerate(setpoint_changes):

        # define the window for this step - from the change to the next change (or end of log)
        start = max(change_idx - 1, 0)
        end = setpoint_changes[i + 1] - 1 if i + 1 < len(setpoint_changes) else len(logged_time) - 1

        # find the last sample that's outside the tolerance band
        # everything after that is "settled"
        window_indices = np.arange(start, end + 1)
        window_errors = angle_log[window_indices] - target_log[window_indices]
        out_of_band = np.where(np.abs(window_errors) >= SETTLE_TOLERANCE)[0]

        if len(out_of_band) == 0:
            continue # the response was inside the band the whole time - nothing to measure
        if out_of_band[-1] == len(window_indices) - 1:
            continue # never settled inside this window - skip

        # the response is settled from this index onwards
        first_settled = window_indices[out_of_band[-1] + 1]

        # settling time
        settling_time = logged_time[first_settled] - logged_time[start]
        settling_times.append(settling_time)

        # RMSE over the settled window
        post_settle_errors = (
            angle_log[first_settled:end + 1] - target_log[first_settled:end + 1]
        )
        rmse = float(np.sqrt(np.mean(post_settle_errors ** 2)))
        rmse_values.append(rmse)

        # overshoot - largest excursion beyond the new setpoint, as % of step size
        pre_step_setpoint = target_log[start]      # value just before the step
        new_setpoint = target_log[first_settled]   # value we settled at
        step_size = new_setpoint - pre_step_setpoint

        if step_size == 0.0: # zero step - no overshoot to measure
            overshoots.append(0.0)
            continue

        transient = angle_log[start:first_settled] # samples between step and settle
        if len(transient) == 0:
            overshoots.append(0.0)
            continue

        # peak excursion beyond the new setpoint, in the direction of the step
        if step_size > 0:
            peak_beyond = float(np.max(transient)) - new_setpoint
        else:
            peak_beyond = new_setpoint - float(np.min(transient))

        if peak_beyond > 0:
            overshoots.append(peak_beyond / abs(step_size) * 100.0) # as percentage
        else:
            overshoots.append(0.0)

    # average across all detected steps - NaN if we found nothing
    mean_settling_time = float(np.mean(settling_times)) if settling_times else float('nan')
    mean_overshoot = float(np.mean(overshoots)) if overshoots else float('nan')
    mean_rmse = float(np.mean(rmse_values)) if rmse_values else float('nan')

    return mean_settling_time, mean_overshoot, mean_rmse