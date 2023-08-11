import numpy as np

def circular_path(t):
    x = 30 * np.cos( 0.02 * t)
    y = 30 * np.sin(0.02 * t)
    v_x = - 30 * 0.02 * np.sin(0.02 * t)
    v_y = 30 * 0.02 * np.cos(0.02 * t)
    w = (v_x * v_x * 0.02 - v_y * v_y * -0.02) / (v_x ** 2 + v_y ** 2)

    
    return x, y, v_x, v_y, w

def sin_path(t):
    x = 10 * np.cos(0.08 * t)
    y = 3 * t
    v_x = - 10 * 0.3 * np.cos( 0.3 * t)
    v_y = 3 * np.ones_like(t)
    w = (- v_y * 10 * 0.3 * 0.3 * np.sin(0.3 * t)) / (v_x ** 2 + v_y ** 2)
    
    return x, y, v_x, v_y, w



def linear_path(t):
    w = 0 * np.ones_like(t)
    h = w * t + 0.3
    v = 6 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v * np.sin(h) 
    y = - v * np.cos(h)
    
    return x, y, v_x, v_y, w


def circular_xy_path(t):
    x = 10 * np.cos( 0.08 * t + 0.3)
    y = 10 * np.sin(0.08 * t + 0.3)
    v_x = - 10 * 0.08 * np.sin(0.08 * t + 0.3)
    v_y = 10 * 0.08 * np.cos(0.08 * t + 0.3)
    w = (v_x * v_x * 0.08 - v_y * v_y * -0.08) / (v_x ** 2 + v_y ** 2)
    
    return x, y, v_x, v_y, w

def sin_xy_path(t):
    x = - 5 * t
    y = 5 * np.sin(0.05 * t)
    v_x = - 5 * np.ones_like(t)
    v_y = 5 * 0.05 * np.cos(0.05 * t)
    w = (v_x * - 5 * 0.05 * 0.05 * np.sin(0.05 * t)) / (v_x ** 2 + v_y ** 2)
    
    return x, y, v_x, v_y, w



def linear_xy_path(t):
    w = 0 * np.ones_like(t)
    h = w * t - 0.8
    v = 6
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v * np.sin(h) 
    y = - v * np.cos(h)
    
    return x, y, v_x, v_y, w

def stoped_xy_path(t):
    w = 0 * np.ones_like(t)
    h = w * t
    v = 0 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v * np.sin(h) - 100
    y = - v * np.cos(h) - 20
    
    return x, y, v_x, v_y, w


def stoped_path(t):
    w = 0 * np.ones_like(t)
    h = w * t
    v = 0 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v * np.sin(h) + 5
    y = - v * np.cos(h) - 25
    
    return x, y, v_x, v_y, w