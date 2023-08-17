import numpy as np
import math
from kf import KalmanFilter



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def Kalman_sim(n_uavs, EKF, f_kf, x, y, vx, vy, ww, delay_strategy, aug):

    class DelayKalmanFilter:
        def __init__(self, kf, delay_strategy=None):
            self.kf = kf
            self.predict = self.kf.predict
            self.predict_nonlinear = self.kf.predict_nonlinear
            
            self.update = self.kf.update

            self.last_msgs = {}

            self.last_z_share = None
            self.last_z_obs = None
            
            self.delay_strategy=delay_strategy
            if delay_strategy is None:
                self.update_fuse = self.update_fuse_no_delay
            elif delay_strategy == "extrapolate":
                self.update_fuse = self.update_fuse_extrapolate
            elif delay_strategy == "extrapolate_plus":
                self.update_fuse = self.update_fuse_extrapolate_plus
            elif delay_strategy == "augmented_state":
                self.update_fuse = self.update_fuse_augmented_state

        def update_fuse_no_delay(self, z, *args, **kwargs):
            self.last_z_obs = z
            self.N = 0
            self.delay_est = 0
            return self.kf.update_fuse(z)

        def update_fuse_extrapolate(self, z, t_z, uav_i, t_now):
            z_corrected = None
            if(self.last_msgs.get(uav_i) != None):
                last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
                self.delay_est = t_now - t_z # delay present in the predict received
                self.N = math.floor(self.delay_est / dt)
                z_corrected = z + (z - last_z) / (t_z - last_t_z) * self.delay_est # extrapolation of all the state vector to the actual time
                self.last_z_share = z_corrected
                self.last_z_obs = z
            else:
                z_corrected = z
                self.N = 0
                self.delay_est = 0


            self.last_msgs[uav_i] = (z, t_z) # saving in memory the predict  

            return self.kf.update_fuse(z)


        def update_fuse_extrapolate_plus(self, z, t_z, uav_i, t_now):
            z_corrected = None

            if(self.last_msgs.get(uav_i) != None):
                last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
                self.delay_est = t_now - t_z # delay present in the predict received
                self.N = math.floor(self.delay_est / dt)

                z_corrected = z.copy() # copy the predict received

                v_x = z[2] 
                v_y = z[3]

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

            return self.kf.update_fuse(z)
        
        def update_fuse_augmented_state(self, z, t_z, uav_i, t_now):
        
        
            self.delay_est = t_now - t_z # delay present in the predict received

            self.N = math.floor(self.delay_est / dt)

            

            if self.N == 0:
                H = np.block([H_fuse, np.zeros((n_state, aug * n_state))])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)
            elif self.N == aug:
                H = np.block([np.zeros((n_state, aug * n_state)), H_fuse])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)
            elif 0 < self.N < aug:
                H = np.block([np.zeros((n_state, self.N * n_state)), H_fuse, np.zeros((n_state, (aug - self.N) * n_state))])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)
            else:
                H = np.block([np.zeros((n_state, aug * n_state)), H_fuse])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)





    #Definition of matrixes for EKF and KF

    dt = 1 / f_kf

    F = np.array([      [1, 0, dt, 0],
                        [0, 1, 0, dt],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    
    # F ----State trasition(A) (Dynamic Model)

    if (EKF == True):

        H = np.array([      [1, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0]])
        
        H_fuse = np.array([     [1, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0],
                            [0, 0, 1, 0, 0],
                            [0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 1]])
    else:

        H = np.array([   [1, 0, 0, 0],
                        [0, 1, 0, 0]])

        H_fuse = np.array([  [1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
    
    
    # H ----Observation Matrix
    if (EKF == True):

        Q = np.array([      [0.5, 0, 0, 0, 0],
                            [0, 0.5, 0, 0, 0],
                            [0, 0, 0.2, 0, 0],
                            [0, 0, 0, 0.2, 0],
                            [0, 0, 0, 0, 0.1]])
    else:

        Q = np.array([   [0.3604, 0, 0, 0],
                            [0, 0.3604, 0, 0],
                            [0, 0, 0.005578, 0],
                            [0, 0, 0, 0.005578]])
    

    # Q ----Process Noise

    R = np.array([ [10, 0],
                [0, 10]])
    
    if (EKF == True):

        R_fuse = np.array([     [1, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0],
                                [0, 0, 1, 0, 0],
                                [0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 1]])
    
    else:


        R_fuse = np.array([  [1.026, 0, 0, 0],
                                [0, 1.026, 0, 0],
                                [0, 0, 5.837, 0],
                                [0, 0, 0, 5.837]])
    if (EKF == True):
        P = np.array([     [0.2, 0, 0, 0, 0],
                                [0, 0.2, 0, 0, 0],
                                [0, 0, 0.4, 0, 0],
                                [0, 0, 0, 0.4, 0],
                                [0, 0, 0, 0, 0.4]])
    else:
        P = np.array([          [0.474, 0, 0, 0],
                                [0, 0.474, 0, 0],
                                [0, 0, 0.503, 0],
                                [0, 0, 0, 0.503]])
    
    # R ----Measurement Noise


    if (EKF == True):

        state = np.stack((x,y, vx, vy,  ww), axis=0)
        
        # x ----Initial value for the state

        x0 = np.array([     [state[0,0]],
                            [state[1,0]],
                            [state[2,0]],
                            [state[3,0]],
                            [0.1]])

    else:

        state = np.stack((x,y, vx, vy), axis=0)
        
        # x ----Initial value for the state

        x0 = np.array([  [state[0,0]],
                            [state[1,0]],
                            [state[2,0]],
                            [state[3,0]]])

    if (delay_strategy == 'augmented_state'): 

        m = aug  #number of augmented states
        
        # x ----Initial value for the state

        x0_init = x0
        
        n_state = x0_init.shape[0]

        x0 = np.vstack((x0, np.zeros((n_state* m,1))))
        if (EKF == False):
            F = np.block([[F, np.zeros((n_state,m*n_state))], [np.eye(m*n_state), np.zeros((n_state * m,n_state))]])
        P = np.block([[P, np.zeros((n_state,aug * n_state))], [np.zeros((aug * n_state, n_state)), np.eye(aug * n_state)]])
        H = np.zeros((2,(aug +1) * n_state ))
        np.fill_diagonal(H, 1, wrap=True)

        Q = np.block([[Q, np.zeros((n_state, m * n_state))], [np.zeros((m * n_state, (m + 1) * n_state))]])


    kfs = [KalmanFilter(F = F.copy(), H = H.copy(),
                        H_fuse = H_fuse.copy() , Q = Q.copy() ,
                        R = R.copy(), R_fuse = R_fuse.copy(),
                        x0= x0.copy(), dt = dt, P = P.copy(), aug= aug) for i in range(n_uavs)]

        
    
    kfs = [DelayKalmanFilter(kf, delay_strategy) for kf in kfs]

    return state, kfs, x0


