import numpy as np

def circular_path(t):
    r = 40
    psi = 0.07
    x = r * np.cos(psi * t)
    y = r * np.sin(psi * t)
    theta = np.zeros_like(t)
    for i, (xx, yy) in enumerate(zip(x,y)):
        if ((yy >= 0 and xx >= 0) or (yy < 0 and xx > 0)):
            theta[i] = np.arctan(yy / xx)
        elif ((yy > 0 and xx < 0)):
            theta[i] = np.arctan(yy / xx) + np.pi
        elif ((yy < 0 and xx < 0)):
            theta[i] = np.arctan(yy / xx) - np.pi
    v_x = -psi * r * np.sin(psi * t)
    v_y = psi * r * np.cos(psi * t)
    w = ((x * v_y) - (y * v_x)) / (x ** 2 + y ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    
    return x,y,v_x,v_y, v, theta, w

def sin_path(t):
    r = 5
    psi = 0.14
    x = r * np.cos(psi * t)
    y = 1 * t
    theta = np.zeros_like(t)
    for i, (xx, yy) in enumerate(zip(x,y)):
        if ((yy >= 0 and xx >= 0) or (yy < 0 and xx > 0)):
            theta[i] = np.arctan(yy / xx)
        elif ((yy > 0 and xx < 0)):
            theta[i] = np.arctan(yy / xx) + np.pi
        elif ((yy < 0 and xx < 0)):
            theta[i] = np.arctan(yy / xx) - np.pi
    v_x = -psi * r * np.sin(psi * t)
    v_y = 1 * np.ones_like(t)
    w = ((x * v_y) - (y * v_x)) / (x ** 2 + y ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    
    return x,y,v_x,v_y, v, theta, w

def linear_path(t):
    w = np.zeros_like(t)
    theta = w * t + (np.pi / 4) * np.ones_like(t)
    v = 6 * np.ones_like(t)
    v_x =  v * np.cos(theta)
    v_y =  v * np.sin(theta)
    x =  v_x * t
    y = v_y * t
    
    return x,y,v_x,v_y, v, theta, w

