import numpy as np

def circular_path(t):
    r = 80
    psi = 0.065
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
    r = 10
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
    v_y = 5 * np.ones_like(t)
    w = ((x * v_y) - (y * v_x)) / (x ** 2 + y ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    
    return x,y,v_x,v_y, v, theta, w

def linear_path(t):
    v_r = -6
    r =  v_r * t 
    psi =  np.pi / 3 
    x = r * np.cos(psi) + 300
    y = r * np.sin(psi) + 300
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


def circular_xy_path(t):
    r = 80
    psi = 0.065
    x = r * np.cos(psi * t) + 300
    y = r * np.sin(psi * t) + 300
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

def sin_xy_path(t):
    r = 10
    psi = 0.14
    y = r * np.cos(psi * t) + 5
    x = 1 * t - 5
    theta = np.zeros_like(t)
    for i, (xx, yy) in enumerate(zip(x,y)):
        if ((yy >= 0 and xx >= 0) or (yy < 0 and xx > 0)):
            theta[i] = np.arctan(yy / xx)
        elif ((yy > 0 and xx < 0)):
            theta[i] = np.arctan(yy / xx) + np.pi
        elif ((yy < 0 and xx < 0)):
            theta[i] = np.arctan(yy / xx) - np.pi
    v_x = -psi * r * np.sin(psi * t)
    v_y = 5 * np.ones_like(t)
    w = ((x * v_y) - (y * v_x)) / (x ** 2 + y ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    return x,y,v_x,v_y, v, theta, w



def linear_xy_path(t):
    v_r = 6
    r =  v_r * t 
    psi = - np.pi / 6
    x = r * np.cos(psi) -200
    y = r * np.sin(psi) -100
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



