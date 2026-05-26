import math
import random

#list of bias and correlation values
bias_instability = [0.0] * 6

def imuNoise(true_value, time_step, white_deviation, bias_deviation,
             time_correlation, index, angle_units = True):
    """
    angle_units = True  → applies deg→rad scaling (gyroscope, °/s input)
    angle_units = False → no unit conversion (accelerometer, m/s² input)
    """
    scale = (math.pi / 180) if angle_units else 1.0

    a0 = 1.0 / time_correlation
    a1 = math.exp(-a0 * time_step)

    b0_white = (white_deviation * scale) / math.sqrt(time_step)
    b0_bias  = (bias_deviation * scale) / math.sqrt(time_step)

    white_noise = b0_white * random.gauss(0, 1)
    bias_instability[index] = (a1 * bias_instability[index]) + (b0_bias * random.gauss(0, 1))

    return float(true_value + white_noise + bias_instability[index])