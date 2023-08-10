import numpy as np
import math
from kalman_filter import KalmanFilter



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def Kalman_sim(n_uavs, EKF, f_kf, x, y, vx, vy, vv, tt, ww, delay_strategy, aug):

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

                v_x = z[3] * np.cos(z[2])
                v_y = z[3] * np.sin(z[2])

                z_corrected[0] = z[0] + v_x * self.delay_est #extrapolation of x, but using the v_x of the state
                z_corrected[1] = z[1] + v_y * self.delay_est #extrapolation of y, but using the v_y of the state

                #Suggestion

                #z_ext[2] = z[2] target_dynamics_linear z[4] * delay_est #extrapolation of theta, but using the w of the state

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
            
            H_sensor = np.zeros(((aug + 1)* 5, (aug + 1)* 5))

            if self.N == 0:
                # state_z = np.zeros((aug * 5, 1))
                # z = np.vstack((z,state_z))
                #H_d = np.block([1, 1, 1, 1, 1, np.zeros(aug * 5)])
                #np.fill_diagonal(H_sensor, H_d)
                H = np.block([H_fuse, np.zeros((5, aug * 5))])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)
            elif self.N == aug:
                # state_z = np.zeros((aug * 5, 1))
                # z = np.vstack((state_z, z))
                #H_d = np.block([np.zeros(aug * 5), 1, 1, 1, 1, 1])
                #np.fill_diagonal(H_sensor, H_d)
                H = np.block([np.zeros((5, aug * 5)), H_fuse])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)
            elif 0 < self.N < aug:
                # state_z = np.zeros((N * 5, 1))
                # z = np.vstack((state_z, z))
                #H_d = np.block([np.zeros(N * 5), 1, 1, 1, 1, 1, np.zeros((aug - N) * 5)])
                #np.fill_diagonal(H_sensor, H_d)
                H = np.block([np.zeros((5, self.N * 5)), H_fuse, np.zeros((5, (aug - self.N) * 5))])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)
            else:
                H = np.block([np.zeros((5, aug * 5)), H_fuse])
                self.kf.H_fuse = H
                return self.kf.update_fuse(z)





    #Definition of matrixes for EKF and KF

    dt = 1 / f_kf


    if (EKF == True):

        state = np.stack((x,y, tt, vv,  ww), axis=0)
        
        # x ----Initial value for the state

        x0 = np.array([     [state[0,0]],
                            [state[1,0]],
                            [state[2,0]],
                            [state[3,0]],
                            [state[4,0]]])

    else:

        state = np.stack((x,y, vx, vy), axis=0)
        
        # x ----Initial value for the state

        x0 = np.array([  [state[0,0]],
                            [state[1,0]],
                            [state[2,0]],
                            [state[3,0]]])

    if (EKF == True and delay_strategy == 'augmented_state'): 

        m = aug  #number of augmented states
        
        # x ----Initial value for the state

        x0_init = np.array([     [state[0,0]],
                            [state[1,0]],
                            [state[2,0]],
                            [state[3,0]],
                            [state[4,0]]])

        x0 = np.vstack((x0, np.zeros((x0_init.shape[0]* m,1))))


    F = np.array([      [1, 0, dt, 0],
                        [0, 1, 0, dt],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])


    # F ----State trasition(A) (Dynamic Model)

    H = np.array([      [1, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0]])

    H_KF = np.array([   [1, 0, 0, 0],
                        [0, 1, 0, 0]])


    H_fuse = np.array([     [1, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0],
                            [0, 0, 1, 0, 0],
                            [0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 1]])

    H_fuse_KF = np.array([  [1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    

    H_sensor = np.zeros((2,(aug +1) *5 ))
    np.fill_diagonal(H_sensor, 1, wrap=True)

    #H_sensor = np.zeros(((aug + 1)* 5, (aug + 1)* 5))
    #H_d = np.block([1, 1, np.zeros(3), np.zeros(aug * 5)])
    #np.fill_diagonal(H_sensor, H_d)
    
    #H_aug = np.zeros(((aug + 1)* 5, (aug + 1)* 5))
    
    

    # H ----Observation Matrix


    Q = np.array([      [0.9133, 0, 0, 0, 0],
                        [0, 0.9133, 0, 0, 0],
                        [0, 0, 0.6894, 0, 0],
                        [0, 0, 0, 0.8501, 0],
                        [0, 0, 0, 0, 0.7565]])

    Q_KF = np.array([   [0.3604, 0, 0, 0],
                        [0, 0.3604, 0, 0],
                        [0, 0, 0.005578, 0],
                        [0, 0, 0, 0.005578]])
    
    
    # Q_aug = np.zeros(((aug +1) *5 ,(aug +1) *5 ))
    # np.fill_diagonal(Q_aug, np.diag(Q), wrap=True)
    Q_aug = np.block([[Q, np.zeros((5, aug * 5))], [np.zeros((aug * 5, (aug + 1) * 5))]])

    # Q ----Process Noise

    R = np.array([ [10, 0],
                [0, 10]])
    
    R_fuse = np.array([     [14.8043, 0, 0, 0, 0],
                            [0, 14.8043, 0, 0, 0],
                            [0, 0, 13.9049, 0, 0],
                            [0, 0, 0, 8.7572, 0],
                            [0, 0, 0, 0, 0.6642]])
    
    R_fuse = np.eye(5) * 5


    R_fuse_KF = np.array([  [1.026, 0, 0, 0],
                            [0, 1.026, 0, 0],
                            [0, 0, 5.837, 0],
                            [0, 0, 0, 5.837]])
    
    P_EKF = np.array([     [0.2166, 0, 0, 0, 0],
                            [0, 0.2166, 0, 0, 0],
                            [0, 0, 0.9076, 0, 0],
                            [0, 0, 0, 0.6118, 0],
                            [0, 0, 0, 0, 0.5038]])
    
    P = np.array([  [0.474, 0, 0, 0],
                            [0, 0.474, 0, 0],
                            [0, 0, 0.503, 0],
                            [0, 0, 0, 0.503]])
    
    #R_aug_fuse = np.zeros((((aug + 1)* 5),(aug + 1)* 5))

    #np.fill_diagonal(R_aug_fuse, np.diag(R_fuse))
    
    #R_aug = np.zeros((((aug + 1)* 5),(aug + 1)* 5))

    #R_d = np.block([2, 2, np.zeros(3), np.zeros(aug * 5)])
    #np.fill_diagonal(R_aug, R_d)
    
    #R_delay = np.block([[np.block([R_fuse, np.zeros((5,5*5))])], [np.block([[np.block([np.zeros((5,n*5)), R_fuse, np.zeros((5,(m-n)*5))])] for n in range(1, m)])], [np.block([np.zeros((5,5*5)), R_fuse])]])




    # R ----Measurement Noise

    if (EKF == True):
        kfs = [KalmanFilter(F = F.copy(), H = H.copy(),
                        H_fuse = H_fuse.copy() , Q = Q.copy() ,
                        R = R.copy(), R_fuse = R_fuse.copy(),
                        x0= x0.copy(), dt = dt, P = P_EKF.copy()) for i in range(n_uavs)]

    else:
        kfs = [KalmanFilter(F = F.copy(), H = H_KF.copy(),
                        H_fuse = H_fuse_KF.copy() , Q = Q_KF.copy() ,
                        R = R.copy(), R_fuse = R_fuse_KF.copy(),
                        x0 = x0.copy(), dt = dt, P = P.copy()) for i in range(n_uavs)]
        
    if (EKF == True and delay_strategy == "augmented_state"):
        kfs = [KalmanFilter(F = F.copy(), H = H_sensor.copy(),
                        H_fuse = H_sensor.copy() , Q = Q_aug.copy() ,
                        R = R.copy(), R_fuse = R_fuse.copy(),
                        x0= x0.copy(), dt = dt, aug = aug, P = P_EKF.copy()) for i in range(n_uavs)]
    
    kfs = [DelayKalmanFilter(kf, delay_strategy) for kf in kfs]

    return state, kfs, x0


