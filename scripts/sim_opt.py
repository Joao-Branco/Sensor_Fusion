import numpy as np
import math
from kalman_filter import KalmanFilter
import time as pytime
import matplotlib.pyplot as plt
import sklearn.metrics

f = 100
sim_time = 60
f_sample = 10
f_kf = 20
f_share = 20

std = 2.0
mean = 0

delay_mean = 2
delay_std = 0.05
delay = 2

n_uavs = 3

SHARE_ON = True
EKF = True
DELAY = True


time = np.arange(0, sim_time, 1/f)

#print(time, len(time))

def target_dynamics(t):
    r = 5
    w = 0.09 * np.ones_like(t)
    v = w * r
    theta = w * t
    v_x = - v * np.sin(theta)
    v_y =   v * np.cos(theta)
    x = - r * np.sin(theta)
    y = r * np.cos(theta)
    
    return x,y,v_x,v_y, v, theta, w

def target_dynamics_sin(t):
    w = 0.09 * np.ones_like(t)
    theta = w * t
    v_x = - 3 * np.sin(theta)
    v_y =  2 * np.ones_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    x = - 3 * np.sin(theta)
    y = v_y * t
    
    return x,y,v_x,v_y, v, theta, w

def check_target_ret(func, t):
    args = func(t)
    for i, a in enumerate(args):
        if not isinstance(a, np.ndarray):
            raise TypeError(f"ret value {i} is not numpy array")
    return args
        
x,y,vx,vy, vv, tt, ww = check_target_ret(target_dynamics, time)
x,y,vx,vy, vv, tt, ww = target_dynamics_sin(time)

state = np.stack((x,y, tt, vv,  ww), axis=0)

#print(x, len(x))





def gen_sensor_data(*arrays):
    return [array + np.random.normal(mean,std, size=array.size) for array in arrays]

sensors = [gen_sensor_data(x,y) for i in range(n_uavs)]




# period_sample = np.zeros(int(f/f_sample))
# period_sample[-1] = 1
# print(period_sample[:100], len(period_sample))
# sample = np.tile(period_sample, f_sample*sim_time)

# print(sample[:100], len(sample))

# for x_,y_ in sensors:
#     x_ *= sample
#     y_ *= sample


# print(x_[:100])


dt = 1 / f_kf


x0 = np.array([     [0],
                    [0],
                    [0],
                    [0],
                    [0]])

x0_KF = np.array([  [5],
                    [0],
                    [0],
                    [0]])



# x ----Initial value for the state

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



# R ----Measurement Noise

if (EKF == True):
    kfs = [KalmanFilter(F = F.copy(), H = H.copy(),
                    H_fuse = H_fuse.copy() , Q = Q.copy() ,
                    R = R.copy(), R_fuse = R_fuse.copy(),
                    x0= x0.copy(), dt = dt) for i in range(n_uavs)]
else:
    kfs = [KalmanFilter(F = F.copy(), H_KF = H_KF.copy(),
                    H_fuse_KF = H_fuse_KF.copy() , Q_KF = Q_KF.copy() ,
                    R = R.copy(), R_fuse_KF = R_fuse_KF.copy(),
                    x0_KF = x0_KF.copy(), dt = dt) for i in range(n_uavs)]
    


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
t_share = 0

#shares queue
q = [[] for _ in range(n_uavs)]



t_start = pytime.time()
for i, t in enumerate(time):
    # update
    if t-t_update >= p_update:
        for sensor, kf in zip(sensors, kfs):
            x_, y_ = sensor[0][i], sensor[1][i]
            kf.update(np.array([[x_], [y_]]))
        t_update = t
        continue

    # update share
    for uav_i in range(n_uavs):
        if (t - q[uav_i][0]) >= p_share and SHARE_ON and t_predict != 0:
            for kf1 in kfs:
                for kf2 in kfs:
                    if kf1 is kf2:
                        continue
                    #delay interpolation
                    measurment = q[uav_i][1] 
                    kf2.update_fuse(measurment)
            q[uav_i].pop(0)
            continue

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


            q[uav_i].append([t + delay, kf.x])



        col_write += 1
        t_predict = t
        continue





for pred, pred_mask in zip(predicts, predict_masks):
    state_filtered = state[:,pred_mask]
    err_abs = np.abs(state_filtered - pred[1:,:]) # ignore time row from pred
    euclidean = np.sqrt(err_abs[0,:] ** 2 + err_abs[1,:] ** 2)

dist_uavs = []
for i_uav in range(n_uavs -1):
    for j_uav in range(n_uavs):
        if j_uav > i_uav :
            x_e = predicts[i_uav][1,:] - predicts[j_uav][1,:]
            y_e = predicts[i_uav][2,:] - predicts[j_uav][2,:]
            dist_uavs.append(np.sqrt(x_e ** 2 + y_e ** 2)) 

dist_uavs = np.array(dist_uavs)



err_abs_mean = np.array([   np.mean(err_abs[0,:]),
                            np.mean(err_abs[1,:]),
                            np.mean(err_abs[2,:]),
                            np.mean(err_abs[3,:]),
                            np.mean(err_abs[4,:])])

rmse = np.array([   np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[0,:], pred[0,:])),
                    np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[1,:], pred[1,:])),
                    np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[2,:], pred[2,:])),
                    np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[3,:], pred[3,:])),
                    np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[4,:], pred[4,:]))])



print(f"predicts last time={predicts[0][0,col_write-1]}")
print(f"gt last time={time[-1]}")
print(f"col write={col_write}")
print(f'finished simulation in {pytime.time() - t_start} seconds')
print(np.shape(predicts))

dx = x[-1] - predicts[0][1,-1]
dy = y[-1] - predicts[0][2,-1]
# print(f"x={x[-1]}, px={predicts[0][1,-1]}")
# print(f"y={y[-1]}, py={predicts[0][2,-1]}")
# print(f"dx={dx}, dy={dy}")

# print(kfs[0].P)

plt.plot(x,y, 'k')
for i in range(n_uavs):
    x_, y_ = predicts[i][1,:], predicts[i][2,:]
    plt.plot(x_[:col_write], y_[:col_write])
plt.grid()
plt.show()

print("Absolute error x:  ", err_abs_mean[0])
print("Absolute error y:  ", err_abs_mean[1])
print("Absolute error theta:  ", err_abs_mean[2])
print("Absolute error v:  ", err_abs_mean[3])
print("Absolute error w:  ", err_abs_mean[4])

print("RMSE x:  ", rmse[0])
print("RMSE y:  ", rmse[1])
print("RMSE theta:  ", rmse[2])
print("RMSE v:  ", rmse[3])
print("RMSE w:  ", rmse[4])

print("Accuracy: ", np.mean(euclidean))
print("Precision: ", np.mean(dist_uavs))

print(kfs[0].P)






            




