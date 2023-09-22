import numpy as np
import pandas as pd

def calculate_uav_positions(num_uavs, radius, speed, time):
    if num_uavs <= 0 or radius <= 0 or speed <= 0 or time < 0:
        return []

    positions = []
    angular_speed = speed / radius  # Calculate angular speed in radians per second

    for i in range(num_uavs):
        angle = (2 * np.pi * i) / num_uavs  # Calculate the angle for each UAV
        x = radius * np.cos(angular_speed * time + angle)
        y = radius * np.sin(angular_speed * time + angle)
        positions.append((x, y))

    return positions

position_uavs = calculate_uav_positions(3, 200, 6, 0.5)
print(position_uavs)











