import numpy as np
import random
from scipy.interpolate import CubicSpline

def circular_path(t):
    v = np.zeros_like(t)
    while(((5.5 <= v) & (v <= 6.5)).all() != True):
        r = random.uniform(5,100)
        theta0 = np.random.randn() * 10
        w0 = random.uniform(0.001, 1)
        x = r * np.cos( w0 * t + theta0)
        y = r * np.sin(w0 * t + theta0)
        v_x = - r * w0 * np.sin(w0 * t + theta0)
        v_y = r * w0 * np.cos(w0 * t + theta0)
        w = (v_x * v_x * w0 - v_y * v_y * (-w0)) / (v_x ** 2 + v_y ** 2)
        v = np.sqrt( v_x ** 2 + v_y ** 2)

    
    return x, y, v_x, v_y, w

def sin_path(t):
    v = np.zeros_like(t)
    while(((5 <= v) & (v <= 7)).all() != True):
        amp = random.uniform(5,100)
        v_l = random.uniform(3,6)
        theta0 = np.random.randn() * 10
        theta1 = np.random.randn() * 10
        w0 = random.uniform(0.001, 1)
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
    while(((5.5 <= v) & (v <= 6.5)).all() != True):
        w = 0 * np.ones_like(t)
        theta0 = np.random.randn() * 10
        h = w * t + theta0
        v = random.uniform(5.5,6.5) * np.ones_like(t)
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






