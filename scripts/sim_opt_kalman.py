import numpy as np
from kf import KalmanFilter, DelayKalmanFilter


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def Kalman_sim(n_uavs, EKF, f_kf, x, y, vx, vy, ww, delay_strategy, aug):

    


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
                            [state[4,0] + 0.1]])

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


