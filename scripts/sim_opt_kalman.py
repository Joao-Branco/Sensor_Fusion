import numpy as np
from kf import KalmanFilter, DelayKalmanFilter


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def Kalman_sim(n_uavs, EKF, f_kf, x, y, vx, vy, ww, delay_strategy, aug, pi, centr):

    


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
        
        H_fuse = H
        
    else:

        H = np.array([   [1, 0, 0, 0],
                        [0, 1, 0, 0]])
        
        H_fuse = H

    
    
    # H ----Observation Matrix
    if (EKF == True):

        G = np.array([  [0.5 * (dt ** 2),   0,                  dt, 0, 0],
                        [0,                 0.5 * (dt ** 2),    0, dt, 0],
                        [0,                 0,                  0, 0, dt]])
    
        G = np.transpose(G)

        # qv = 0.00614227 ** 2
        # qw = 1.1618515 ** 2

        qv = 0.001 ** 2
        qw = 0.01 ** 2

        q = np.array([[qv, 0, 0],
                        [0, qv, 0],
                        [0, 0, qw]])


        Q = np.dot(np.dot(G, q),np.transpose(G))

    else:

        G = np.array([  [0.5 * (dt ** 2),   0,                  dt, 0],
                        [0,                 0.5 * (dt ** 2),    0, dt]])
    
        G = np.transpose(G)

        # qv = 0.02063591 ** 2
        qv = 0.001 ** 2
            
        q = np.array([[qv, 0],
                        [0, qv]])

        Q = np.dot(np.dot(G, q),np.transpose(G))
    

    # Q ----Process Noise

    R = np.array([ [10, 0],
                [0, 10]])
    
    if (EKF == True):
        P = np.array([     [1.53009997, 0, 0, 0, 0],
                                [0, 1.53009997, 0, 0, 0],
                                [0, 0, 1.81024162, 0, 0],
                                [0, 0, 0, 1.81024162, 0],
                                [0, 0, 0, 0, 0.000005]])
        
    else:
        P = np.array([          [1.53009997, 0, 0, 0],
                                [0, 1.53009997, 0, 0],
                                [0, 0, 1.81024162, 0],
                                [0, 0, 0, 1.81024162]])
    
    # R ----Measurement Noise


    if (EKF == True):

        state = np.stack((x,y, vx, vy,  ww), axis=0)
        
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

    if (delay_strategy == 'augmented_state'): 

        m = aug  #number of augmented states
        
        # x ----Initial value for the state

        x0_init = x0
        
        n_state = x0_init.shape[0]

        x0 = np.vstack((x0, np.zeros((n_state* m,1))))
        if (EKF == False):
            F = np.block([[F, np.zeros((n_state,m*n_state))], [np.eye(m*n_state), np.zeros((n_state * m,n_state))]])
        P = np.block([[P, np.zeros((n_state,aug * n_state))], [np.zeros((aug * n_state, n_state)), np.eye(aug * n_state)]])
        H = np.zeros((2,(aug +1) * n_state))
        np.fill_diagonal(H, 1, wrap=True)
        H_fuse = np.zeros((2, n_state ))
        np.fill_diagonal(H_fuse, 1, wrap=True)

        Q = np.block([[Q, np.zeros((n_state, m * n_state))], [np.zeros((m * n_state, (m + 1) * n_state))]])


    kfs = [KalmanFilter(F = F.copy(), H = H.copy(),
                        H_fuse = H_fuse.copy() , Q = Q.copy() ,
                        R = R.copy(),
                        x0= x0.copy(), dt = dt, P = P.copy(), aug= aug) for i in range(n_uavs)]

        
    
    kfs = [DelayKalmanFilter(kf, delay_strategy, pi= pi, centr = centr) for kf in kfs]

    return state, kfs, x0


