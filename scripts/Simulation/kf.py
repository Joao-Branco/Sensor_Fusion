import numpy as np
import math

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
        f_d.append(np.array([  [state[0+d*5,0]],
                        [state[1+d*5,0]],
                        [state[2+d*5,0]],
                        [state[3+d*5,0]],
                        [state[4+d*5,0]]]))



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
        f_d.append(np.array([  [state[0+d*5,0]],
                        [state[1+d*5,0]],
                        [state[2+d*5,0]],
                        [state[3+d*5,0]],
                        [state[4+d*5,0]]]))



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
                    [0, 0, np.sin(dt * w)           , np.cos(dt * w)          , dt * vx * np.cos(dt * w) - dt * vy * np.sin(dt * w)],
                    [0, 0, 0                        ,                         0,                              1]])  


    if aug > 0:
        J_a = np.block([[np.block([J, np.zeros((5,aug*5))])], [np.block([np.identity(5*aug), np.zeros((5*aug,5))])]])  
    else:
        J_a = J 

    return J_a



class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, H_fuse = None, Q = None, R = None, P = None, x0 = None, dt = None, aug = None, EKF = None):


        self.n = x0.shape[0]
        self.m = H.shape[1]
        self.dt = dt
        self.F = F
        self.H = H
        self.H_fuse = H_fuse
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        self.J = np.eye(self.n)
        self.aug = 0 if aug is None else aug
        self.EKF = False if EKF is None else EKF

        self.S = np.linalg.inv(P)
        self.s = np.dot(self.S, self.x)



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
            return True

        

             
    def update_fuse(self, z, pi):
        try:
            self.S = np.linalg.inv(self.P) # INVERSE ERROR COVARIANCE MATRIX
            self.s = np.dot(self.S, self.x) # 
            self.I = np.dot(self.H_fuse.T, np.dot(np.linalg.inv(self.R), self.H_fuse)) # INFORMATION MATRIX
            self.i = np.dot(self.H_fuse.T, np.dot(np.linalg.inv(self.R), z)) # INFORMATION VECTOR

            self.s = self.s + pi * self.i 
            self.S = self.S + pi * self.I

            self.P = np.linalg.inv(self.S)
            self.x = np.dot(np.linalg.inv(self.S), self.s)
        except np.linalg.LinAlgError as e: 
            return True
        

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #




class DelayKalmanFilter:
        def __init__(self, kf, delay_strategy=None, pi = 1, centr = None):
            self.kf = kf
            self.predict = self.kf.predict
            self.predict_nonlinear = self.kf.predict_nonlinear
            
            self.update = self.kf.update

            self.pi = pi
            self.centr = centr

            self.last_msgs = {}

            self.last_z_share = None
            self.last_z_obs = None
            
            self.delay_strategy = delay_strategy
            if delay_strategy is None:
                self.update_fuse = self.update_fuse_no_delay
            elif delay_strategy == "extrapolate":
                self.update_fuse = self.update_fuse_extrapolate
            # elif delay_strategy == "extrapolate_plus":
            #     self.update_fuse = self.update_fuse_extrapolate_plus
            elif delay_strategy == "augmented_state":
                self.update_fuse = self.update_fuse_augmented_state
                self.H = []
                H_fuse = self.kf.H_fuse
                aug = self.kf.aug
                n_state = int(self.kf.H_fuse.shape[1])
                z_state = int(self.kf.H_fuse.shape[0])
                for i in range(aug + 1):
                    if i == 0:
                        self.H.append(np.block([H_fuse, np.zeros((z_state, aug * n_state))]))
                    elif i == aug:
                        self.H.append(np.block([np.zeros((z_state, aug * n_state)), H_fuse]))
                    elif 0 < i < aug:
                        self.H.append(np.block([np.zeros((z_state, i * n_state)), H_fuse, np.zeros((z_state, (aug - i) * n_state))]))

            
            self.dt = self.kf.dt






        def update_fuse_no_delay(self, z, *args, **kwargs):
            self.last_z_obs = z
            self.N = 0
            self.delay_est = 0
            if (self.centr == True):
                return self.kf.update_fuse(z, self.pi)
            
            else:
                return self.kf.update_fuse(z, self.pi)

        def update_fuse_extrapolate(self, z, t_z, uav_i, t_now):
            z_corrected = None
            if(self.last_msgs.get(uav_i) != None):
                last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
                self.delay_est = t_now - t_z # delay present in the predict received
                z_corrected = z + (z - last_z) / (t_z - last_t_z) * self.delay_est # extrapolation of all the state vector to the actual time
                self.last_z_share = z_corrected
                self.last_z_obs = z
            else:
                z_corrected = z
                self.N = 0
                self.delay_est = 0


            self.last_msgs[uav_i] = (z, t_z) # saving in memory the predict  

            if (self.centr == True):
                return self.kf.update(z)
            
            else:
                return self.kf.update_fuse(z, self.pi)


        def update_fuse_extrapolate_plus(self, z, t_z, uav_i, t_now):
            z_corrected = None

            if(self.last_msgs.get(uav_i) != None):
                last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
                self.delay_est = t_now - t_z # delay present in the predict received

                z_corrected = z.copy() # copy the predict received

                v_x = self.kf.x[2] 
                v_y = self.kf.x[3] 

                z_corrected[0] = z[0] + v_x * self.delay_est #extrapolation of x, but using the v_x of the state
                z_corrected[1] = z[1] + v_y * self.delay_est #extrapolation of y, but using the v_y of the state

                #Suggestion

                
                self.last_z_share = z_corrected
                self.last_z_obs = z

            else:
                z_corrected = z
                self.N = 0
                self.delay_est = 0


            self.last_msgs[uav_i] = (z, t_z) # saving in memory the predict  

            if (self.centr == True):
                return self.kf.update(z)
            
            else:
                return self.kf.update_fuse(z, self.pi)
        
        def update_fuse_augmented_state(self, z, t_z, uav_i, t_now):
        
        
            self.delay_est = t_now - t_z # delay present in the predict received

            self.N = math.floor(self.delay_est / self.dt)


            if (self.centr == True):
                return self.kf.update(z)
            
            if self.N > self.kf.aug:
                pass
            else:
                self.kf.H_fuse = self.H[self.N]
                return self.kf.update_fuse(z, self.pi)
        