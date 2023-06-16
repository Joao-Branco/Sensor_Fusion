import numpy as np
import math
from kalman_filter import KalmanFilter
import time as pytime
import matplotlib.pyplot as plt

f = 100
sim_time = 120
f_sample = 10
f_kf = 20
f_share = 15

std = 2.0
mean = 0

n_uavs = 3

SHARE_ON = True
EKF = True


time = np.arange(0, sim_time, 1/f)

print(time, len(time))

def target_dynamics(t):
    v = 5
    w = 0.09
    theta = w * t
    v_x = v * np.cos(theta)
    v_y = v * np.sin(theta)
    x = v_x
    y = v_y
    
    return x,y,v_x,v_y, v, theta, w

def target_dynamics_sin(t):
    v = 5
    w = 0.09
    theta = w * t
    v_x = v * np.cos(theta)
    v_y = 1
    x = v_x
    y = v_y
    
    return x,y,v_x,v_y, v, theta, w
        

x,y,vx,vy, vv, tt, ww = target_dynamics(time)
print(x, len(vx))

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


x0 = np.array([     [5],
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
print(predicts)
col_write = 0

print(f"time shape {time.shape}")
print(f"predicts shape {predicts[0].shape}")

#periods
p_predict = 1/f_kf
p_update = 1/f_sample
p_share = 1/f_share

# last times
t_predict = 0
t_update = 0
t_share = 0

t_start = pytime.time()
for i, t in enumerate(time):
    # update?
    if t-t_update >= p_update:
        for uav_i, kf in enumerate(kfs):
            x_, y_ = sensors[uav_i][0][i], sensors[uav_i][1][i]
            kf.update(np.array([[x_], [y_]]))
        t_update = t
        continue

    # update share
    if t-t_share >= p_share and SHARE_ON:
        for kf1 in kfs:
            for kf2 in kfs:
                if kf1 is kf2:
                    continue
                kf2.update_fuse(kf1.x)
        t_share = t
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
        col_write += 1
        t_predict = t
        continue

print(f"predicts last time={predicts[0][0,col_write-1]}")
print(f"gt last time={time[-1]}")
print(f"col write={col_write}")
print(f'finished simulation in {pytime.time() - t_start} seconds')
print(predicts[0][:,:5])
print(predicts[0][:,-5:])
print(np.shape(predicts))

dx = x[-1] - predicts[0][1,-1]
dy = y[-1] - predicts[0][2,-1]
print(f"x={x[-1]}, px={predicts[0][1,-1]}")
print(f"y={y[-1]}, py={predicts[0][2,-1]}")
print(f"dx={dx}, dy={dy}")

print(kfs[0].P)

plt.plot(x,y, 'k')
for i in range(n_uavs):
    x_, y_ = predicts[i][1,:], predicts[i][2,:]
    #plt.plot(x_[:col_write], y_[:col_write])
plt.grid()
plt.show()

print(predicts[0][1,-1])

for i in enumerate(time):
    for uav_i in enumerate(kfs):
        if t == predicts[uav_i][0,t]:
            error = abs(x - predicts[uav_i][1,t])
            
            
print(error[0][0,:])




