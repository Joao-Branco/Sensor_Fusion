import numpy as np
import navpy

def f_nonlinear_w(state, dt, aug):
    x, y, vx, vy, w = state[:5]
    x = x.item()
    y = y.item()
    vx = vx.item()
    vy = vy.item()
    w = w.item()

    f = np.array([  [x + dt * ( vx * (np.sin(w * dt) / (w * dt)) - vy * ((1 - np.cos(w * dt)) / (w * dt)))],
                    [y + dt * ( vx * ((1 - np.cos(w * dt)) / (w * dt)) + vy * (np.sin(w * dt) / (w * dt)))],
                    [vx * np.cos(w * dt) - vy * np.sin(w * dt)],
                    [vx * np.sin(w * dt) + vy * np.cos(w * dt)],
                    [w]])
    
    f_d = []
    for d in range(aug):
        f_d.append(np.array([  [x[0+d*5,0]],
                        [x[1+d*5,0]],
                        [x[2+d*5,0]],
                        [x[3+d*5,0]],
                        [x[4+d*5,0]]]))



    if aug > 0:
        f_d = np.vstack((f_d))
        F = np.vstack((f,f_d))
    else:
        F = f

    return F


def f_nonlinear_w0(state, dt, aug):
    x, y, vx, vy, w = state[:5]
    x = x.item()
    y = y.item()
    vx = vx.item()
    vy = vy.item()
    w = w.item()

    f = np.array([  [x + dt * vx],
                    [y + dt * vy],
                    [vx],
                    [vy],
                    [w]])
    
    f_d = []
    for d in range(aug):
        f_d.append(np.array([  [x[0+d*5,0]],
                        [x[1+d*5,0]],
                        [x[2+d*5,0]],
                        [x[3+d*5,0]],
                        [x[4+d*5,0]]]))



    if aug > 0:
        f_d = np.vstack((f_d))
        F = np.vstack((f,f_d))
    else:
        F = f

    return F

def Jacobian_w0(state, dt, aug):
    x, y, vx, vy, w = state[:5]
    x = x.item()
    y = y.item()
    vx = vx.item()
    vy = vy.item()
    w = w.item()

    J = np.array([  [1, 0, dt, 0, 0],
                    [0, 1, 0, dt, 0],
                    [0, 0, 1, 0, 0],
                    [0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 1]])  


    if aug > 0:
        J_a = np.block([[np.block([J, np.zeros((5,aug*5))])], [np.block([np.identity(5*aug), np.zeros((5*aug,5))])]])  
    else:
        J_a = J 

    return J_a

def Jacobian_w(state, dt, aug):
    x, y, vx, vy, w = state[:5]
    x = x.item()
    y = y.item()
    vx = vx.item()
    vy = vy.item()
    w = w.item()


    J = np.array([[1, 0, np.sin(dt * w) / w       , (np.cos(dt * w) - 1) / w , - dt *( (vy * np.sin(dt * w)) / w - (vx * np.cos(dt * w)) / w + (vx * np.sin(dt * w)) / (dt * w ** 2) + (vy * (np.cos(dt * w) - 1)) / (dt * w ** 2))],
                    [0, 1, -(np.cos(dt * w) - 1) / w, np.sin(dt * w) / w       ,  dt * ( (vy * np.cos(dt * w)) / w + (vx * np.sin(dt * w)) / w - (vy * np.sin(dt * w)) / (dt * w ** 2) + (vx * (np.cos(dt * w) - 1)) / (dt * w ** 2))],
                    [0, 0, np.cos(dt * w)           , -np.sin(dt * w)          , - dt * vy * np.cos(dt * w) - dt * vx * np.sin(dt * w)],
                    [0, 0, np.sin(dt * w)           , -np.cos(dt * w)          , dt * vx * np.cos(dt * w) + dt * vy * np.sin(dt * w)],
                    [0, 0, 0                        ,                         0,                              1]])  


    if aug > 0:
        J_a = np.block([[np.block([J, np.zeros((5,aug*5))])], [np.block([np.identity(5*aug), np.zeros((5*aug,5))])]])  
    else:
        J_a = J 

    return J_a



class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, H_fuse = None, Q = None, R = None, R_fuse = None, R_delay = None, P = None, x0 = None, dt = None, aug = None, EKF = None):


        self.n = x0.shape[0]
        self.m = H.shape[1]
        self.dt = dt
        self.F = F
        self.H = H
        self.H_fuse = H_fuse
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.R_fuse = np.eye(self.n) if R_fuse is None else R_fuse
        self.R_delay = np.eye(self.n) if R_delay is None else R_delay
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        self.J = np.eye(self.n)
        self.aug = 0 if aug is None else aug
        self.EKF = False if EKF is None else EKF


    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

        return self.x
    
    def predict_nonlinear(self, u = 0):

            

        if (self.x[4] == 0):
            self.J =Jacobian_w0(self.x, self.dt, self.aug)
            self.x = f_nonlinear_w0(self.x, self.dt, self.aug)
        else:
            self.J =Jacobian_w(self.x, self.dt, self.aug)
            self.x = f_nonlinear_w(self.x, self.dt, self.aug)


 

        self.P = np.dot(np.dot(self.J, self.P), self.J.T) + self.Q
        return self.x

    def update(self, z):
        self.y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        try:
            self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
            self.x = self.x + np.dot(self.K, self.y)
            I = np.eye(self.n)
            self.P = np.dot(np.dot(I - np.dot(self.K, self.H), self.P), 
                (I - np.dot(self.K, self.H)).T) + np.dot(np.dot(self.K, self.R), self.K.T)
        except np.linalg.LinAlgError as e:
            print('x \n',self.x, self.x.shape)
            print('P \n',self.P, self.P.shape)
            print('R \n',self.R, self.R.shape)
            print('K \n',self.K, self.K.shape)
            print('S \n',S, S.shape)
            print('Q \n',self.Q, self.Q.shape)
            print('y \n',self.y, self.y.shape)

        

             
    def update_fuse(self, z):
        self.y_fuse = z - np.dot(self.H_fuse, self.x)
        S = self.R_fuse + np.dot(self.H_fuse, np.dot(self.P, self.H_fuse.T))
        
        try:
            self.K_fuse = np.dot(np.dot(self.P, self.H_fuse.T), np.linalg.inv(S))
            self.x = self.x + np.dot(self.K_fuse, self.y_fuse)
            I = np.eye(self.n)
            self.P = np.dot(np.dot(I - np.dot(self.K_fuse, self.H_fuse), self.P), 
        	    (I - np.dot(self.K_fuse, self.H_fuse)).T) + np.dot(np.dot(self.K_fuse, self.R_fuse), self.K_fuse.T)
        
        except np.linalg.LinAlgError as e:
            print('x \n',self.x, self.x.shape)
            print('P \n',self.P, self.P.shape)
            print('R \n',self.R, self.R.shape)
            print('K_fuse \n',self.K_fuse, self.K_fuse.shape)
            print('S \n',S, S.shape)
            print('Q \n',self.Q, self.Q.shape)
            print('y_fuse \n',self.y_fuse, self.y_fuse.shape)
        