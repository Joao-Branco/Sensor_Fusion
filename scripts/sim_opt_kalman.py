import numpy as np
from kalman_filter import KalmanFilter



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def Kalman_sim(n_uavs, EKF, f_kf, x, y, vx, vy, vv, tt, ww, delay_strategy):

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
            return self.kf.update_fuse(z)

        def update_fuse_extrapolate(self, z, t_z, uav_i, t_now):
            z_corrected = None
            if(self.last_msgs.get(uav_i) != None):
                last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
                delay_est = t_now - t_z # delay present in the predict received
                z_corrected = z + (z - last_z) / (t_z - last_t_z) * delay_est # extrapolation of all the state vector to the actual time
                self.last_z_share = z_corrected
                self.last_z_obs = z
            else:
                z_corrected = z


            self.last_msgs[uav_i] = (z, t_z) # saving in memory the predict  

            return self.kf.update_fuse(z)


        def update_fuse_extrapolate_plus(self, z, t_z, uav_i, t_now):
            z_corrected = None
            if(self.last_msgs.get(uav_i) != None):
                last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
                delay_est = t_now - t_z # delay present in the predict received

                z_corrected = z.copy() # copy the predict received

                v_x = z[3] * np.cos(z[2])
                v_y = z[3] * np.sin(z[2])

                z_corrected[0] = z[0] + v_x * delay_est #extrapolation of x, but using the v_x of the state
                z_corrected[1] = z[1] + v_y * delay_est #extrapolation of y, but using the v_y of the state

                #Suggestion

                #z_ext[2] = z[2] target_dynamics_linear z[4] * delay_est #extrapolation of theta, but using the w of the state

                self.last_z_share = z_corrected
                self.last_z_obs = z

            else:
                z_corrected = z


            self.last_msgs[uav_i] = (z, t_z) # saving in memory the predict  

            return self.kf.update_fuse(z)
        
        def update_fuse_ood(self, z, t_z, uav_i, t_now):
            raise NotImplementedError()
        
        
            delay_est = t_now - t_z # delay present in the predict received

            N = delay_est / dt

            H_delay = np.block([[np.block([R_fuse, np.zeros((5,5*5))])], [np.block([[np.block([np.zeros((5,n*5)), R_fuse, np.zeros((5,(m-n)*5))])] for n in range(1, m)])], [np.block([np.zeros((5,5*5)), R_fuse])]])



            raise NotImplementedError()



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

    if (EKF == 'augmented_state'): 

        m = 5  #number of augmented states
        
        # x ----Initial value for the state

        x0 = np.array([     [state[0,0]],
                            [state[1,0]],
                            [state[2,0]],
                            [state[3,0]],
                            [state[4,0]]])

        x0_delay = np.vstack((x0, np.zeros((x0.shape[0]* m,1))))


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

    # H ----Observation Matrix


    Q = np.array([      [0.5, 0, 0, 0, 0],
                        [0, 0.5, 0, 0, 0],
                        [0, 0, 0.1, 0, 0],
                        [0, 0, 0, 0.5, 0],
                        [0, 0, 0, 0, 0.1]])

    Q_KF = np.array([   [0.5, 0, 0, 0],
                        [0, 0.5, 0, 0],
                        [0, 0, 0.1, 0],
                        [0, 0, 0, 0.1]])


    # Q ----Process Noise

    R = np.array([ [2, 0],
                [0, 2]])

    R_fuse = np.array([     [0.5, 0, 0, 0, 0],
                            [0, 0.5, 0, 0, 0],
                            [0, 0, 0.1, 0, 0],
                            [0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0.1]])


    R_fuse_KF = np.array([  [0.5, 0, 0, 0],
                            [0, 0.5, 0, 0],
                            [0, 0, 0.1, 0],
                            [0, 0, 0, 1]])
    
    R_delay = np.block([[np.block([R_fuse, np.zeros((5,5*5))])], [np.block([[np.block([np.zeros((5,n*5)), R_fuse, np.zeros((5,(m-n)*5))])] for n in range(1, m)])], [np.block([np.zeros((5,5*5)), R_fuse])]])




    # R ----Measurement Noise

    if (EKF == True):
        kfs = [KalmanFilter(F = F.copy(), H = H.copy(),
                        H_fuse = H_fuse.copy() , Q = Q.copy() ,
                        R = R.copy(), R_fuse = R_fuse.copy(),
                        x0= x0.copy(), dt = dt) for i in range(n_uavs)]
    else:
        kfs = [KalmanFilter(F = F.copy(), H = H_KF.copy(),
                        H_fuse = H_fuse_KF.copy() , Q = Q_KF.copy() ,
                        R = R.copy(), R_fuse = R_fuse_KF.copy(),
                        x0 = x0.copy(), dt = dt) for i in range(n_uavs)]
    
    kfs = [DelayKalmanFilter(kf, delay_strategy) for kf in kfs]

    return state, kfs, x0


