import numpy as np
import time as pytime
from typing import Callable
from pathlib import Path
import sim_opt_kalman
import sim_printing




def gen_sensor_data(*arrays, SENSOR_MEAN, SENSOR_STD):
    return [array + np.random.normal(SENSOR_MEAN, SENSOR_STD, size=array.size) for array in arrays]

        

def sim(DELAY_STRATEGY : str, EKF : bool, OUT_OF_ORDER : bool, SHARE_ON : bool, DELAY_MEAN : bool, DELAY_STD : bool,  SENSOR_MEAN : bool, SENSOR_STD : bool,  sim_time: int, n_uavs: int, f_sim: float, f_kf: float, f_sample: float, f_share: float, target_dynamics: Callable, AUG : int, dir : Path, state : np):
    time = np.arange(0, sim_time, 1/f_sim)

    # generate target data (ground truth)

    x,y,vx,vy, ww = state


    sensors = [gen_sensor_data(x,y, SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD= SENSOR_STD) for i in range(n_uavs) ]

    state, kfs, x0 = sim_opt_kalman.Kalman_sim(n_uavs= n_uavs, EKF= EKF, f_kf= f_kf, x= x, y= y, vx= vx, vy= vy, ww= ww, delay_strategy= DELAY_STRATEGY, aug= AUG)



    # matrix for results
    if (EKF == True):
        rows = 5
    else:
        rows = 4
    predicts = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]
    predict_masks = [np.zeros(f_kf*sim_time, dtype=np.uint32) for i in range(n_uavs)]
    errors = [np.zeros((rows+1, f_kf*sim_time)) for _ in range(n_uavs)]
    discrete = [[] for _ in range(n_uavs)]


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
            for uav_i, (sensor, kf) in enumerate(zip(sensors, kfs)):
                x_, y_ = sensor[0][i], sensor[1][i]
                l_d = []
                l_d.append(t)
                l_d.append('update')
                l_d.append(kf.kf.x)
                kf.update(np.array([[x_], [y_]]))
                l_d.append(np.array([[x[i]],[y[i]],[vx[i]],[vy[i]], [ww[i]]]))
                l_d.append(kf.kf.x)
                l_d.append(kf.kf.P)
                l_d.append(kf.kf.H)
                l_d.append(np.array([[x_], [y_]]))
                l_d.append(kf.kf.K)
                l_d.append(kf.kf.y)
                discrete[uav_i].append(l_d)
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
                            kf.update_fuse(z, t_z, i_z, t)
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
                        l_d.append(kf.kf.K_fuse)
                        l_d.append(kf.kf.y_fuse)


                        l_d.append(np.array([kf.N]))
                        l_d.append(np.array([kf.delay_est]))
                        discrete[uav_i].append(l_d)
                for idx in sorted(remove_idx, reverse=True):
                    q[uav_i].pop(idx)

                


        # predict
        if t-t_predict >= p_predict:
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
                discrete[uav_i].append(l_d)

                if t - t_share[uav_i] >= p_share and SHARE_ON:
                    delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # is we don't want out of order messages, recompute delay
                    while not OUT_OF_ORDER and t+delay < q_ts[uav_i]:
                        delay = np.random.normal(DELAY_MEAN, DELAY_STD)

                    # add this message to every other uav queue
                    for uav_j in range(n_uavs):
                        if uav_i == uav_j:
                            continue # dont add my predicts to my own queue
                        if (EKF == True):
                            q[uav_j].append((t + delay, t, x_i[:5], uav_i))
                        else:
                            q[uav_j].append((t + delay, t, x_i[:4], uav_i))


                    # update last share time for this uav
                    q_ts[uav_i] = t + delay
                    t_share[uav_i] = t
            col_write += 1
            t_predict= t
            continue
    
    computer_cost = pytime.time() - t_start
    print(f"predicts last time={predicts[0][0,col_write-1]}")
    print(f"gt last time={time[-1]}")
    print(f"col write={col_write}")
    print(f'finished simulation in {computer_cost} seconds')
    print(np.shape(predicts))
    print(np.shape(predict_masks))

    # for uav_i in range(n_uavs):
    #      dir_uav = Path(str(dir) + f'/uav_{uav_i}')
    #      dir_uav.mkdir()
    #      sim_printing.discrete_printing(discrete_kf= discrete, uav_i= uav_i, dir = dir_uav)


    return state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time