import numpy as np
import math
from kalman_filter import KalmanFilter
import time as pytime
import matplotlib.pyplot as plt
import sklearn.metrics

import target_dynamics

import sim_core

# simulation parameters
f = 200
sim_time = 60
f_sample = 10
f_kf = 20
f_share = 5

SENSOR_STD = 2.0
SENSOR_MEAN = 0


n_uavs = 3
DELAY_MEAN = 0.5
DELAY_STD = 0.01  # 0=guarantees no out of order, but removes randomness in delay
SHARE_ON = True
EKF = True
DELAY = True
OUT_OF_ORDER = True

DELAY_STRATEGY = "extrapolate"


#### FOR LATER DON'T USE YET
delay_lst = [True, False]
share_lst = [True, False]
dynamics_lst = [target_dynamics.circular_path,
                target_dynamics.linear_path]



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

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
        elif delay_strategy == "out_of_order":
            self.update_fuse = self.update_fuse_ood

    def update_fuse_no_delay(self, z, *args, **kwargs):
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

            z_corrected = z # copy the predict received

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



#print(time, len(time))



state = np.stack((x,y, tt, vv,  ww), axis=0)

print(np.shape(state))





def gen_sensor_data(*arrays):
    return [array + np.random.normal(SENSOR_MEAN, SENSOR_STD, size=array.size) for array in arrays]

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


x0 = np.array([     [state[0,0]],
                    [state[1,0]],
                    [state[2,0]],
                    [state[3,0]],
                    [state[4,0]]])

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


kfs = [DelayKalmanFilter(kf, delay_strategy = DELAY_STRATEGY) for kf in kfs]


sim_outputs = sim_core.sim(...)

#Converting list with observations and extrapolations to an array

for i in range(n_uavs):
    z_obs[i].pop(0)
    z_corr[i].pop(0)
    z_masks[i].pop(0)
    for t, t_value in enumerate(z_obs[i]):
        t_array = np.array([z_obs[i][t][1]])
        z_array = z_obs[i][t][0].flatten()
        z_obs[i][t] = np.concatenate((t_array, z_obs[i][t][0].flatten()))
        z_corr[i][t] = np.concatenate((t_array, z_corr[i][t][0].flatten()))
    z_obs[i] = np.array(z_obs[i])
    z_corr[i] = np.array(z_corr[i])
    z_masks[i] = np.array(z_masks[i])
    z_obs[i] = z_obs[i].T
    z_corr[i] = z_corr[i].T

for i in range(n_uavs):
    z_obs[i].pop(0)
    z_corr[i].pop(0)
    z_masks[i].pop(0)
    for t, t_value in enumerate(z_obs[i]):
        t_array = np.array([z_obs[i][t][1]])
        z_array = z_obs[i][t][0].flatten()
        z_obs[i][t] = np.concatenate((t_array, z_obs[i][t][0].flatten()))
        z_corr[i][t] = np.concatenate((t_array, z_corr[i][t][0].flatten()))
    z_obs[i] = np.array(z_obs[i])
    z_corr[i] = np.array(z_corr[i])
    z_masks[i] = np.array(z_masks[i])
    z_obs[i] = z_obs[i].T
    z_corr[i] = z_corr[i].T






for pred, pred_mask in zip(predicts, predict_masks):
    state_filtered = state[:,pred_mask]
    err_abs = np.abs(state_filtered - pred[1:,:]) # ignore time row from pred
    euclidean = np.sqrt(err_abs[0,:] ** 2 + err_abs[1,:] ** 2)

for obs, obs_mask in zip(z_obs, z_masks):
    state_filtered_obs = state[:,obs_mask]
    err_obs = np.abs(state_filtered_obs - obs[1:,:]) # ignore time row from pred

for corr, corr_mask in zip(z_corr, z_masks):
    state_filtered_corr = state[:,corr_mask]
    err_corr = np.abs(state_filtered_corr - corr[1:,:]) # ignore time row from pred

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


err_obs_mean = np.array([   np.mean(err_obs[0,:]),
                            np.mean(err_obs[1,:]),
                            np.mean(err_obs[2,:]),
                            np.mean(err_obs[3,:]),
                            np.mean(err_obs[4,:])])

err_corr_mean = np.array([   np.mean(err_corr[0,:]),
                            np.mean(err_corr[1,:]),
                            np.mean(err_corr[2,:]),
                            np.mean(err_corr[3,:]),
                            np.mean(err_corr[4,:])])


print(np.shape(z_obs))
print(f"predicts last time={predicts[0][0,col_write-1]}")
print(f"gt last time={time[-1]}")
print(f"col write={col_write}")
print(f'finished simulation in {pytime.time() - t_start} seconds')
print(np.shape(predicts))
print(predict_masks[0], np.shape(predict_masks))


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

print(f"X---Absolute error obs: {err_obs_mean[0]}, Absolute error corr: {err_corr_mean[0]}")
print(f"Y---Absolute error obs: {err_obs_mean[1]}, Absolute error corr: {err_corr_mean[1]}")
print(f"THETA---Absolute error obs: {err_obs_mean[2]}, Absolute error corr: {err_corr_mean[2]}")
print(f"V---Absolute error obs: {err_obs_mean[3]}, Absolute error corr: {err_corr_mean[3]}")
print(f"W---Absolute error obs: {err_obs_mean[4]}, Absolute error corr: {err_corr_mean[4]}")

print("X-----OBS") if err_obs_mean[0] < err_corr_mean[0] else print("X-----CORR") 
print("Y-----OBS") if err_obs_mean[1] < err_corr_mean[1] else print("Y-----CORR") 
print("THETA-----OBS") if err_obs_mean[2] < err_corr_mean[2] else print("THETA-----CORR") 
print("V-----OBS") if err_obs_mean[3] < err_corr_mean[3] else print("V-----CORR") 
print("W-----OBS") if err_obs_mean[4] < err_corr_mean[4] else print("W-----CORR") 

# print("RMSE x:  ", rmse[0])
# print("RMSE y:  ", rmse[1])
# print("RMSE theta:  ", rmse[2])
# print("RMSE v:  ", rmse[3])
# print("RMSE w:  ", rmse[4])

print("Accuracy: ", np.mean(euclidean))
print("Precision: ", np.mean(dist_uavs))