import numpy as np


def circular_difference(angle1, angle2):
    signed_difference = angle2 - angle1
    normalized_difference = (signed_difference + np.pi) % (2 * np.pi) - np.pi
    return normalized_difference

angles1 = np.array([7 *np.pi/8, np.pi/3, -np.pi/6])
angles2 = np.array([-np.pi, np.pi/6, -np.pi/4])

normalized_differences = circular_difference(angles1, angles2)
rmse = np.sqrt(np.mean(normalized_differences**2))

print(normalized_differences)
print(rmse)