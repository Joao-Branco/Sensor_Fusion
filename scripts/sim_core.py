import numpy as np
import math
import time as pytime
import matplotlib.pyplot as plt
import sklearn.metrics
from typing import Callable

def check_target_ret(func, t):
    args = func(t)
    for i, a in enumerate(args):
        if not isinstance(a, np.ndarray):
            raise TypeError(f"ret value {i} is not numpy array")
    return args
        

def sim(sim_time: int, f_sim: float, target_dynamics: Callable):
    time = np.arange(0, sim_time, 1/f_sim)

    # generate target data (ground truth)
    x,y,vx,vy, vv, tt, ww = check_target_ret(target_dynamics, time)



    # matrix for results
    rows, cols = x0.shape
    predicts = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]
    predict_masks = [np.zeros(f_kf*sim_time, dtype=np.uint32) for i in range(n_uavs)]
    errors = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]

    col_write = 0

    # print(f"time shape {time.shape}")
    # print(f"predicts shape {predicts[0].shape}")

    #periods
    p_predict = 1/f_kf
    p_update = 1/f_sample
    p_share = 1/f_share

    # last times
    t_predict = 0
    t_update = 0
    t_share = [0 for _ in range(n_uavs)]

    #shares queue
    q = [[] for _ in range(n_uavs)]
    q_ts = [0 for _ in range(n_uavs)] # last share times for each uav
    z_obs = [[]  for _ in range(n_uavs)] # z received
    z_corr = [[]  for _ in range(n_uavs)] # z with correction of delay 
    z_masks = [[]  for _ in range(n_uavs)]



    t_start = pytime.time()
    for i, t in enumerate(time):
        # update
        if t-t_update >= p_update:
            for sensor, kf in zip(sensors, kfs):
                x_, y_ = sensor[0][i], sensor[1][i]
                kf.update(np.array([[x_], [y_]]))
            t_update = t
    # matrix for results
    rows, cols = x0.shape
    predicts = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]
    predict_masks = [np.zeros(f_kf*sim_time, dtype=np.uint32) for i in range(n_uavs)]
    errors = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]

    col_write = 0

    # print(f"time shape {time.shape}")
    # print(f"predicts shape {predicts[0].shape}")

    #periods
    p_predict = 1/f_kf
    p_update = 1/f_sample
    p_share = 1/f_share

    # last times
    t_predict = 0
    t_update = 0
    t_share = [0 for _ in range(n_uavs)]

    #shares queue
    q = [[] for _ in range(n_uavs)]
    q_ts = [0 for _ in range(n_uavs)] # last share times for each uav
    z_obs = [[]  for _ in range(n_uavs)] # z received
    z_corr = [[]  for _ in range(n_uavs)] # z with correction of delay 
    z_masks = [[]  for _ in range(n_uavs)]



    t_start = pytime.time()
    for i, t in enumerate(time):
        # update
        if t-t_update >= p_update:
            for sensor, kf in zip(sensors, kfs):
                x_, y_ = sensor[0][i], sensor[1][i]
                kf.update(np.array([[x_], [y_]]))
            t_update = t

        # update share
        for uav_i, kf in enumerate(kfs):
            # if sharing, and queue not empty
            for z_ in q[uav_i]: # for every shared message in my queue, i_z = index of source uav
                t_z, z, i_z = z_
                if (t - t_z) >= 0: # check if the message is not too recent
                    # update
                    kf.update_fuse(z, t_z, uav_i, t)
                    t_array = np.array(t)
                    z_obs[uav_i].append([kf.last_z_obs, t])
                    z_corr[uav_i].append([kf.last_z_share, t])

                    z_masks[uav_i].append(int(i))
                    q[uav_i].remove(z_)
                    


        # predict
        if t-t_predict >= p_predict:
            for uav_i, kf in enumerate(kfs):
                if (EKF == True):
                    x_i = kf.predict_nonlinear()
                else:
                    x_i = kf.predict()


                predicts[uav_i][0,col_write] = t   
                predicts[uav_i][1:,col_write] = x_i[:,0]

                predict_masks[uav_i][col_write] = i

                if t - t_share[uav_i] >= p_share and SHARE_ON:
                    delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # is we don't want out of order messages, recompute delay
                    while not OUT_OF_ORDER and t+delay < q_ts[uav_i]:
                        delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # add this message # matrix for results
    rows, cols = x0.shape
    predicts = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]
    predict_masks = [np.zeros(f_kf*sim_time, dtype=np.uint32) for i in range(n_uavs)]
    errors = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]

    col_write = 0

    # print(f"time shape {time.shape}")
    # print(f"predicts shape {predicts[0].shape}")

    #periods
    p_predict = 1/f_kf
    p_update = 1/f_sample
    p_share = 1/f_share

    # last times
    t_predict = 0
    t_update = 0
    t_share = [0 for _ in range(n_uavs)]

    #shares queue
    q = [[] for _ in range(n_uavs)]
    q_ts = [0 for _ in range(n_uavs)] # last share times for each uav
    z_obs = [[]  for _ in range(n_uavs)] # z received
    z_corr = [[]  for _ in range(n_uavs)] # z with correction of delay 
    z_masks = [[]  for _ in range(n_uavs)]



    t_start = pytime.time()
    for i, t in enumerate(time):
        # update
        if t-t_update >= p_update:
            for sensor, kf in zip(sensors, kfs):
                x_, y_ = sensor[0][i], sensor[1][i]
                kf.update(np.array([[x_], [y_]]))
            t_update = t

        # update share
        for uav_i, kf in enumerate(kfs):
            # if sharing, and queue not empty
            for z_ in q[uav_i]: # for every shared message in my queue, i_z = index of source uav
                t_z, z, i_z = z_
                if (t - t_z) >= 0: # check if the message is not too recent
                    # update
                    kf.update_fuse(z, t_z, uav_i, t)
                    t_array = np.array(t)
                    z_obs[uav_i].append([kf.last_z_obs, t])
                    z_corr[uav_i].append([kf.last_z_share, t])

                    z_masks[uav_i].append(int(i))
                    q[uav_i].remove(z_)
                    


        # predict
        if t-t_predict >= p_predict:
            for uav_i, kf in enumerate(kfs):
                if (EKF == True):
                    x_i = kf.predict_nonlinear()
                else:
                    x_i = kf.predict()


                predicts[uav_i][0,col_write] = t   
                predicts[uav_i][1:,col_write] = x_i[:,0]

                predict_masks[uav_i][col_write] = i

                if t - t_share[uav_i] >= p_share and SHARE_ON:
                    delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # is we don't want out of order messages, recompute delay
                    while not OUT_OF_ORDER and t+delay < q_ts[uav_i]:
                        delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # add this message to every other uav queue
                    for uav_j in range(n_uavs):
                        if uav_i == uav_j:
                            continue # dont add my predicts to my own queue

                        q[uav_j].append((t + delay, x_i, uav_i))

                    # update last share time for this uav
                    q_ts[uav_i] = t + delay
                    t_share[uav_i] = t
            col_write += 1
            t_predict= t
            continueto every other uav queue
                    for uav_j in range(n_uavs):
                        if uav_i == uav_j:
                            continue # dont add my predicts to my own queue

                        q[uav_j].append((t + delay, x_i, uav_i))

                    # update last share time for this uav
                    q_ts[uav_i] = t + delay
                    t_share[uav_i] = t
            col_write += 1
            t_predict= t
            continue
        # update share
        for uav_i, kf in enumerate(kfs):
            # if sharing, and queue not empty
            for z_ in q[uav_i]: # for every shared message in my queue, i_z = index of source uav
                t_z, z, i_z = z_
                if (t - t_z) >= 0: # check if the message is not too recent
                    # update
                    kf.update_fuse(z, t_z, uav_i, t)
                    t_array = np.array(t)
                    z_obs[uav_i].append([kf.last_z_obs, t])
                    z_corr[uav_i].append([kf.last_z_share, t])

                    z_masks[uav_i].append(int(i))
                    q[uav_i].remove(z_)
                    


        # predict
        if t-t_predict >= p_predict:
            for uav_i, kf in enumerate(kfs):
                if (EKF == True):
                    x_i = kf.predict_nonlinear()
                else:
                    x_i = kf.predict()


                predicts[uav_i][0,col_write] = t   
                predicts[uav_i][1:,col_write] = x_i[:,0]

                predict_masks[uav_i][col_write] = i

                if t - t_share[uav_i] >= p_share and SHARE_ON:
                    delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # is we don't want out of order messages, recompute delay
                    while not OUT_OF_ORDER and t+delay < q_ts[uav_i]:
                        delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # add this message to every other uav queue
                    for uav_j in range(n_uavs):
                        if uav_i == uav_j:
                            continue # dont add my predicts to my own queue

                        q[uav_j].append((t + delay, x_i, uav_i))

                    # update last share time for this uav
                    q_ts[uav_i] = t + delay
                    t_share[uav_i] = t
            col_write += 1
            t_predict= t
            continue