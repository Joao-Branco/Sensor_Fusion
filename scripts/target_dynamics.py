import numpy as np

def circular_path(t):
    r = 80
    psi = 0.065
    x = r * np.cos(psi * t)
    y = r * np.sin(psi * t)
    theta = np.zeros_like(t)
    v_x = -psi * r * np.sin(psi * t)
    v_y = psi * r * np.cos(psi * t)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = (psi ** 3 * r ** 2 * np.cos(psi * t) ** 2 + psi ** 3 * r ** 2 * np.sin(psi * t)  ** 2) / (psi ** 2 * r ** 2 * np.cos(psi * t) ** 2 + psi ** 2 * r ** 2 * np.sin(psi * t) ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    
    return x,y,v_x,v_y, v, theta, w

def sin_path(t):
    r = 10
    psi = 0.14
    x = r * np.cos(psi * t)
    vc = 5 * np.ones_like(t)
    y = vc * t
    theta = np.zeros_like(t)
    v_x = - psi * r * np.sin(psi * t)
    v_y = vc * np.ones_like(t)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = (psi ** 2 * r * vc * np.cos(psi * t))/(vc ** 2 + psi ** 2 * r ** 2 * np.sin(psi * t) ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    
    return x,y,v_x,v_y, v, theta, w

def linear_path(t):
    v_r = - 6 * np.ones_like(t)
    r =  v_r * t 
    psi =  np.pi / 3 
    x = r * np.cos(psi) + 300
    y = r * np.sin(psi) + 300
    theta = np.zeros_like(t)
    v_x = v_r  * np.cos(psi)
    v_y = v_r * np.sin(psi)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = np.zeros_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    
    return x,y,v_x,v_y, v, theta, w


def circular_xy_path(t):
    r = 80
    psi = 0.065
    x = r * np.cos(psi * t) + 300
    y = r * np.sin(psi * t) + 300
    theta = np.zeros_like(t)
    v_x = -psi * r * np.sin(psi * t)
    v_y = psi * r * np.cos(psi * t)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = (psi ** 3 * r ** 2 * np.cos(psi * t) ** 2 + psi ** 3 * r ** 2 * np.sin(psi * t)  ** 2) / (psi ** 2 * r ** 2 * np.cos(psi * t) ** 2 + psi ** 2 * r ** 2 * np.sin(psi * t) ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    
    return x,y,v_x,v_y, v, theta, w

def sin_xy_path(t):
    r = 10
    psi = 0.14
    y = r * np.cos(psi * t) + 5
    vc = 5 * np.ones_like(t)
    x = vc * t - 5
    theta = np.zeros_like(t)
    v_y = - psi * r * np.sin(psi * t)
    v_x = vc * np.ones_like(t)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = -( psi ** 2 * r * vc * np.cos(psi * t)) / (vc ** 2 + psi ** 2 * r ** 2 * np.sin(psi * t) ** 2)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    return x,y,v_x,v_y, v, theta, w



def linear_xy_path(t):
    v_r = 6 * np.ones_like(t)
    r =  v_r * t 
    psi = - np.pi / 6
    x = r * np.cos(psi) -200
    y = r * np.sin(psi) -100
    theta = np.zeros_like(t)
    v_x = v_r  * np.cos(psi)
    v_y = v_r * np.sin(psi)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = np.zeros_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    return x,y,v_x,v_y, v, theta, w

def stoped_xy_path(t):
    r =  - 20 * np.ones_like(t)
    psi = - np.pi / 6
    x = r * np.cos(psi)
    y = r * np.sin(psi)
    theta = np.zeros_like(t)
    v_x = np.zeros_like(t)
    v_y =  np.zeros_like(t)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = np.zeros_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    return x,y,v_x,v_y, v, theta, w

def stoped_path(t):
    r =  np.zeros_like(t)
    psi = - np.pi / 6
    x = r * np.cos(psi)
    y = r * np.sin(psi)
    theta = np.zeros_like(t)
    v_x = np.zeros_like(t)
    v_y = np.zeros_like(t)
    for i, (v_xx, v_yy) in enumerate(zip(v_x,v_y)):
        if ((v_yy >= 0 and v_xx > 0) or (v_yy < 0 and v_xx > 0)):
            theta[i] = np.arctan(v_yy / v_xx)
        elif ((v_yy > 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) + np.pi
        elif ((v_yy < 0 and v_xx < 0)):
            theta[i] = np.arctan(v_yy / v_xx) - np.pi
        elif (v_xx == 0):
            theta[i] = 0
    w = np.zeros_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)

    return x,y,v_x,v_y, v, theta, w