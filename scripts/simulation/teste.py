import numpy as np

t = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

v = np.zeros_like(t)
while(((5.5 <= v) & (v <= 6.5)).all() != True):
    r = np.random.random() * 10
    theta0 = np.random.randn() * 10
    x = 30 * np.cos( 0.02 * t + theta0) + np.random.random() * 10
    y = 30 * np.sin(0.02 * t + theta0) + np.random.randn() * 10
    v_x = - 30 * 0.02 * np.sin(0.02 * t + theta0)
    v_y = 30 * 0.02 * np.cos(0.02 * t + theta0)
    w = (v_x * v_x * 0.02 - v_y * v_y * -0.02) / (v_x ** 2 + v_y ** 2)
    v = np.sqrt( v_x ** 2 + v_y ** 2)


print(x)
print(y)
print(v_x)
print(v_y)
print(w)
print(v)
