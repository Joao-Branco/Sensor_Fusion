import numpy as np
import time as pytime
from pathlib import Path
import sim_opt_kalman
import sim_printing
import os

def calculate_distance_between_vertices(n, r, ring, ring_on = True):
    # Calculate the central angle between each pair of vertices
    central_angle = 2 * np.pi / n

    # Initialize an empty distance matrix
    distance_matrix = np.zeros((n, n), dtype=float)

    # Calculate the distances between vertices
    for i in range(n):
        for j in range(i+1, n):
            # Calculate the difference in angles between the two vertices
            angle_difference = abs(i - j) * central_angle

            # Calculate the distance between the vertices using trigonometry
            distance = 2 * r * np.sin(angle_difference / 2)

            # Fill in both directions since the matrix is symmetric
            if(ring_on == True):
                if(j == ring[i+1] or j == ring[i-1]):
                    distance_matrix[i][j] = distance
                    distance_matrix[j][i] = distance
                else:
                    distance_matrix[i][j] = 10000
                    distance_matrix[j][i] = 10000

            else:
                distance_matrix[i][j] = distance
                distance_matrix[j][i] = distance



    return distance_matrix



def gen_sensor_data(x, y, SENSOR_MEAN, SENSOR_STD, PHASE, time):
    x_noise = x + 6.48 * np.sin(0.01 * 2 * np.pi * time + 1.93 + PHASE) - 2.24 
    y_noise = y + 6.27 * np.sin(0.01 * 2 * np.pi * time + 0.29 + PHASE) + 0.3

    return [x_noise + np.random.normal(SENSOR_MEAN, SENSOR_STD + 0.26, size=time.size) , y_noise + np.random.normal(SENSOR_MEAN, SENSOR_STD - 0.38, size=time.size)]

def kalman_steps(uav, l_d, dir, printt, counter):
    if (printt == True):
        dir_uav = os.path.join(dir, f'uav_{str(uav)}')
        sim_printing.discrete_printing(list_uav = l_d, dir = dir_uav, w = counter)
    return []


        

def sim(dir : Path, state : np, DELAY_STRATEGY : str = None, EKF : bool = True, OUT_OF_ORDER : bool = False, SHARE_ON : bool = False, DELAY_MEAN : float = 0, DELAY_STD : float = 0,  SENSOR_MEAN : float = 2, SENSOR_STD : float = 0,  sim_time: int = 60, n_uavs: int = 3, f_sim: float = 200, f_kf: float = 20, f_sample: float = 10, f_share: float = 5, AUG : int = 0, PI : float = 0, CENTR : bool = True, delay_d : bool = False, printt : bool = False, Ring_on : bool = False):
    time = np.arange(0, sim_time, 1/f_sim)

    # generate target data (ground truth)

    x,y,vx,vy, ww = state

    ring = [i for i in range(n_uavs)]


    sensors = [gen_sensor_data(x,y, SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD= SENSOR_STD, PHASE= 2 * np.pi * i / n_uavs, time= time) for i in range(n_uavs) ]

    state, kfs, x0 = sim_opt_kalman.Kalman_sim(n_uavs= n_uavs, EKF= EKF, f_kf= f_kf, x= x, y= y, vx= vx, vy= vy, ww= ww, delay_strategy= DELAY_STRATEGY, aug= AUG, pi= PI, centr= CENTR)
    dir = os.path.join(dir, f"EKF_{EKF}share_{SHARE_ON}_freq_share_{f_share}_strategy_{DELAY_STRATEGY}_delay_{DELAY_MEAN}_pi_{PI}_nuavs{n_uavs}")
    os.mkdir(dir)
    for uav_i in range(n_uavs):
        dir_uav = os.path.join(dir, f'uav_{uav_i}')
        os.mkdir(dir_uav)

    

    # matrix for results
    if (EKF == True):
        rows = 5
    else:
        rows = 4


    predicts = [np.zeros((rows+1, f_kf*sim_time - 1)) for _ in range(n_uavs)]
    predict_masks = [np.zeros(f_kf*sim_time - 1, dtype=np.uint32) for i in range(n_uavs)]
    sensor_masks = [np.zeros(f_sample*sim_time - 1, dtype=np.uint32) for i in range(n_uavs)]



    col_write = 0
    col_sensor = 0

    #periods
    p_predict = 1/f_kf
    p_update = 1/f_sample
    if (SHARE_ON == True):
        p_share = 1/f_share
    else:
        p_share = 1000000 # value to big do dont get into code of update fuse

    # last times
    t_predict = 0
    t_update = 0
    t_share = [0 for _ in range(n_uavs)]

    #shares queue
    q = [[] for _ in range(n_uavs)]
    q_ts = [[0 for _ in range(n_uavs)] for _ in range(n_uavs)] # last share times for i uav in j
    q_ts_check = [[[] for _ in range(n_uavs)] for _ in range(n_uavs)] # check
    z_obs = [[]  for _ in range(n_uavs)] # z received
    z_corr = [[]  for _ in range(n_uavs)] # z with correction of delay 
    z_masks = [[]  for _ in range(n_uavs)]
    x_obs = [[]  for _ in range(n_uavs)] # 



    if (delay_d == True):
        distance_matrix = calculate_distance_between_vertices(n_uavs, 200, ring= ring, ring_on= Ring_on)
    else:
        distance_matrix = np.ones((n_uavs, n_uavs))

    t_start = pytime.time()
    counter = 0
    stop = False
    for i, t in enumerate(time):
        # update
        if round(t-t_update, 5) >= p_update:
            for uav_i, (sensor, kf) in enumerate(zip(sensors, kfs)):
                x_, y_ = sensor[0][i], sensor[1][i]
                sensor_masks[uav_i][col_sensor] = i


                if round(t - t_share[uav_i], 5) >= p_share and SHARE_ON:
                # add this message to every other uav queue
                    for uav_j in range(n_uavs):

                        delay = np.random.normal(DELAY_MEAN * distance_matrix[uav_i, uav_j], DELAY_STD)

                        # is we don't want out of order messages, recompute delay
                        while not OUT_OF_ORDER and t+delay < q_ts[uav_i][uav_j]:
                            delay = np.random.normal(DELAY_MEAN * distance_matrix[uav_i, uav_j], DELAY_STD)

                        q[uav_j].append((t + delay, t, np.array([[x_], [y_]]) , uav_i))


                        # update last share time for this uav
                        q_ts[uav_i][uav_j] = t + delay
                        q_ts_check[uav_i][uav_j].append(delay)
                    t_share[uav_i] = t

                l_d = []
                l_d.append(t)
                l_d.append('update')
                l_d.append(kf.kf.x)
                x_obs[uav_i].append(np.array([t, x_, y_]))
                stop = kf.update(np.array([[x_], [y_]]))
                l_d.append(np.array([[x[i]],[y[i]],[vx[i]],[vy[i]], [ww[i]]]))
                l_d.append(kf.kf.x)
                l_d.append(kf.kf.P)
                l_d.append(kf.kf.H)
                l_d.append(np.array([[x_], [y_]]))
                l_d.append(kf.kf.K)
                l_d.append(kf.kf.y)
                counter = counter + 1
                l_d = kalman_steps(uav_i, l_d, dir, printt, counter)

            col_sensor += 1
            t_update = t

        # update share
        if(SHARE_ON == True):
            for uav_i, kf in enumerate(kfs):
                # if sharing, and queue not empty
                remove_idx = []
                for idx, z_ in enumerate(q[uav_i]): # for every shared message in my queue, i_z = index of source uav
                    t_z_delay, t_z, z, i_z = z_
                    if (t - t_z_delay) >= 0: # check if the message is not too recent
                        # update
                        if (DELAY_STRATEGY != None):
                            l_d = []
                            l_d.append(t)
                            l_d.append('update_fuse')
                            l_d.append(kf.kf.x)
                            x_obs[uav_i].append(np.array([t, z[0][0], z[1][0]]))
                            stop = kf.update_fuse(z, t_z, i_z, t)
                            z_obs[uav_i].append([kf.last_z_obs, t])
                            z_corr[uav_i].append([kf.last_z_share, t])
                            z_masks[uav_i].append(i)
                            #q[uav_i].remove(z_)
                            remove_idx.append(idx)
                        else:
                            l_d = []
                            l_d.append(t)
                            l_d.append('update_fuse')
                            l_d.append(kf.kf.x)
                            x_obs[uav_i].append(np.array([t, z[0][0], z[1][0]]))
                            kf.update_fuse(z)
                            z_obs[uav_i].append([kf.last_z_obs, t])
                            z_corr[uav_i].append([kf.last_z_obs, t])

                            z_masks[uav_i].append(i)
                            #q[uav_i].remove(z_)
                            remove_idx.append(idx)
                        
                        

                        l_d.append(np.array([[x[i]],[y[i]],[vx[i]],[vy[i]], [ww[i]]]))
                        l_d.append(kf.kf.x)
                        l_d.append(kf.kf.P)
                        l_d.append(kf.kf.H_fuse)
                        l_d.append(z)


                        l_d.append(np.array([kf.N]))
                        l_d.append(np.array([kf.delay_est]))
                        counter = counter + 1
                        l_d = kalman_steps(uav_i, l_d, dir, printt, counter)
                for idx in sorted(remove_idx, reverse=True):
                    q[uav_i].pop(idx)

                


        # predict
        if round(t-t_predict,5) >= p_predict:
            for uav_i, kf in enumerate(kfs):
                if (EKF == True):
                    l_d = []
                    l_d.append(t)
                    l_d.append('predict')
                    l_d.append(kf.kf.x)
                    x_i = kf.predict_nonlinear()
                    predicts[uav_i][0,col_write] = t   
                    predicts[uav_i][1:,col_write] = x_i[:5,0]

                else:
                    l_d = []
                    l_d.append(t)
                    l_d.append('predict')
                    l_d.append(kf.kf.x)
                    x_i = kf.predict()
                    predicts[uav_i][0,col_write] = t   
                    predicts[uav_i][1:,col_write] = x_i[:4,0]




                predict_masks[uav_i][col_write] = i
                l_d.append(np.array([[x[i]],[y[i]],[vx[i]],[vy[i]], [ww[i]]]))
                l_d.append(kf.kf.x)
                l_d.append(kf.kf.P)
                l_d.append(kf.kf.J)
                counter = counter + 1
                l_d = kalman_steps(uav_i, l_d, dir, printt, counter)

            col_write += 1
            t_predict= t
            continue

        if (stop == True):
            break
    
    computer_cost = pytime.time() - t_start
    print(f"predicts last time={predicts[0][0,col_write-1]}")
    print(f"gt last time={time[-1]}")
    print(f"col write={col_write}")
    print(f'finished simulation in {computer_cost} seconds')
    print(np.shape(predicts))
    print(np.shape(predict_masks))

    stats_delay = [[[] for _ in range(n_uavs)] for _ in range(n_uavs)]

    for uav_i in range(n_uavs):
        for uav_j in range(n_uavs):
            stats_delay[uav_i][uav_j].append(np.mean(q_ts_check[uav_i][uav_j]))
            stats_delay[uav_i][uav_j].append(np.std(q_ts_check[uav_i][uav_j]))
    

    return state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time, sensor_masks, x_obs