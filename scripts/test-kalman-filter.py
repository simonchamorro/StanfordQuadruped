# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055
from adafruit_extended_bus import ExtendedI2C as I2C
import numpy as np
from scripts.kalman import Kalman



i2c = I2C(4)
sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

last_val = 0xFFFF


def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result

history_x = []
history_y = []
measurements = []

time_history = []
start_time = time.time()
prev_time = time.time()
kalman_filter = Kalman()
finished = False
while not finished:
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time
    accel = np.array(list(sensor.acceleration)) - np.array(list(sensor.gravity))
    vel_measurement = accel * dt
    velocity = kalman_filter.compute_kalman(vel_measurement)
    history_x.append(velocity[0])
    history_y.append(velocity[1])
    measurements.append(vel_measurement[0])
    finished = current_time > start_time + 20
    time_history.append(current_time - start_time)

np.savez("imu_test.npz", x=np.array(history_x), y=np.array(history_y), t=np.array(time_history))
