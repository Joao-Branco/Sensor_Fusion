import numpy as np

def circular_path(t):
    r = 5
    w = 0.09 * np.ones_like(t)
    v = w * r
    theta = w * t
    v_x = - v * np.sin(theta)
    v_y =   v * np.cos(theta)
    x = - r * np.sin(theta)
    y = r * np.cos(theta)
    
    return x,y,v_x,v_y, v, theta, w

def sin_path(t):
    w = 0.09 * np.ones_like(t)
    theta = w * t
    v_x = - 3 * np.sin(theta)
    v_y =  2 * np.ones_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    x = - 3 * np.sin(theta)
    y = v_y * t
    
    return x,y,v_x,v_y, v, theta, w

def linear_path(t):
    w = np.zeros_like(t)
    theta = w * t
    v_x = 2 * np.ones_like(t)
    v_y =  2 * np.ones_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    x = v_x * t
    y = v_y * t
    
    return x,y,v_x,v_y, v, theta, w

