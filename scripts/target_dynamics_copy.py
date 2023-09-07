import numpy as np
import random

def circular_path(t):
    v = np.zeros_like(t)
    while(((3.5 <= v) & (v <= 50)).all() != True):
        r = 20
        theta0 = np.random.randn() * 10
        w0 = random.uniform(0.05, 0.8)
        x = r * np.cos( w0 * t + theta0)
        y = r * np.sin(w0 * t + theta0)
        v_x = - r * w0 * np.sin(w0 * t + theta0)
        v_y = r * w0 * np.cos(w0 * t + theta0)
        w = (v_x * v_x * w0 - v_y * v_y * (-w0)) / (v_x ** 2 + v_y ** 2)
        v = np.sqrt( v_x ** 2 + v_y ** 2)

    
    return x, y, v_x, v_y, w

def sin_path(t):
    v = np.zeros_like(t)
    while(((5.5 <= v) & (v <= 6.5)).all() != True):
        amp = 30
        v_l = random.uniform(0.005,10)
        theta0 = np.random.randn() * 10
        theta1 = np.random.randn() * 10
        w0 = random.uniform(0.008, 0.2)
        x_ = amp * np.cos(w0 * t + theta0)
        y_ = v_l * t 
        x = x_ * np.cos(theta1) - y_ * np.sin(theta1)
        y = y_ * np.cos(theta1) + x_ * np.sin(theta1)
        v_x_ = - amp * w0 * np.cos(w0 * t + theta0)
        v_y_ = v_l * np.ones_like(t)
        v_x = v_x_ * np.cos(theta1) - v_y_ * np.sin(theta1)
        v_y = v_y_ * np.cos(theta1) + v_x_ * np.sin(theta1)
        w = (- v_y * amp * w0 * w0 * np.sin(w0 * t + theta0)) / (v_x ** 2 + v_y ** 2)
        v = np.sqrt( v_x ** 2 + v_y ** 2)

    
    return x, y, v_x, v_y, w



def linear_path(t):
    v = np.zeros_like(t)
    while(((2.5 <= v) & (v <= 6.5)).all() != True):
        w = 0 * np.ones_like(t)
        theta0 = np.random.randn() * 10
        h = w * t + theta0
        v = random.uniform(2,7) * np.ones_like(t)
        v_x = v * np.cos(h)
        v_y = v * np.sin(h) 
        x = v_x * t
        y =  v_y * t
    
    return x, y, v_x, v_y, w


def stoped_path(t):
    w = 0 * np.ones_like(t)
    h = w * t
    v = 0 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v * np.sin(h)
    y = - v * np.cos(h)
    
    return x, y, v_x, v_y, w

def combined_path(t):
    period = np.array_split(t,3)
    x1, y1, _, _, _ = circular_path(period[0])
    v_x1 = np.gradient(x1, period[0])
    v_y1 = np.gradient(y1, period[0])



    x2, y2, _, _, _ = linear_path(period[1] - period[1][0])
    v_x2 = np.gradient(x2, period[1])
    v_y2 = np.gradient(y2, period[1])
    while(abs(v_x1[-1] - v_x2[0]) >= 0.05 and abs(v_y1[-1] - v_y2[0]) >= 0.05):
        x2, y2, _, _, _ = linear_path(period[1] - period[0][-1])
        v_x2 = np.gradient(x2, period[1])
        v_y2 = np.gradient(y2, period[1])


    x3, y3, _, _, _ = sin_path(period[2] - period[2][0])
    v_x3 = np.gradient(x3, period[2])
    v_y3 = np.gradient(y3, period[2])
    while(abs(v_x2[-1] - v_x3[0]) >= 0.05 and abs(v_y2[-1] - v_y3[0]) >= 0.05):
        x3, y3, _, _, _ = sin_path(period[2] - period[2][0])
        v_x3 = np.gradient(x3, period[2])
        v_y3 = np.gradient(y3, period[2])




    x = np.concatenate((x1, x2 + x1[-1]))
    x = np.concatenate((x, x3 + x[-1]))
    y = np.concatenate((y1, y2 + y1[-1]))
    y = np.concatenate((y, y3 + y[-1]))
    v_x = np.gradient(x, t)
    v_y = np.gradient(y, t)
    theta = np.arctan2(v_y, v_x)
    w = np.gradient(theta, t)

    return x, y, v_x, v_y, w 





