import numpy as np

def circular_path(t):
    w = 0.5 * np.ones_like(t)
    h = w * t
    v = 6 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t
    y = v_y * t
    
    return x, y, v_x, v_y, w

def sin_path(t):
    w = 2 * np.sin(0.3 * t)
    h = w * t
    v = 6 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t
    y = v_y * t
    
    return x, y, v_x, v_y, w



def linear_path(t):
    w = 0 * np.ones_like(t)
    h = w * t - 1
    v = -6
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t
    y = v_y * t
    
    return x, y, v_x, v_y, w


def circular_xy_path(t):
    w = 0.5 * np.ones_like(t)
    h = w * t
    v = 6
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t
    y = v_y * t
    
    return x, y, v_x, v_y, w

def sin_xy_path(t):
    w = 5 * np.cos(0.1 * t)
    h = w * t
    v = 6 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t
    y = v_y * t
    
    return x, y, v_x, v_y, w



def linear_xy_path(t):
    w = 0 * np.ones_like(t)
    h = w * t
    v = 6
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t
    y = v_y * t
    
    return x, y, v_x, v_y, w

def stoped_xy_path(t):
    w = 0.5 * np.ones_like(t)
    h = w * t
    v = 0 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t
    y = v_y * t
    
    return x, y, v_x, v_y, w


def stoped_path(t):
    w = 0.5 * np.ones_like(t)
    h = w * t
    v = 0 * np.ones_like(t)
    v_x = v * np.cos(h)
    v_y = v * np.sin(h) 
    x = v_x * t + 200
    y = v_y * t - 200
    
    return x, y, v_x, v_y, w