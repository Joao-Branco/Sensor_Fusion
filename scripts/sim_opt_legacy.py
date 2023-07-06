import numpy as np
import math
from kalman_filter import KalmanFilter
import time as pytime
import matplotlib.pyplot as plt
import sklearn.metrics

# simulation parameters
f = 50
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

        self.corr_num = 0
        
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
        if(self.last_msgs.get(uav_i) != None):
            last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
            delay_est = t_now - t_z # delay present in the predict received
            z_corrected = z + (z - last_z) / (t_z - last_t_z) * delay_est # extrapolation of all the state vector to the actual time
        else:
            z_corrected = z


        self.last_z_share = z_corrected
        self.last_z_obs = z

        self.last_msgs[uav_i] = (z, t_z) # saving in memory the predict  

        return self.kf.update_fuse(z)


    def update_fuse_extrapolate_plus(self, z: np.ndarray, t_z, uav_i, t_now):
        if(self.last_msgs.get(uav_i) != None):
            last_z, last_t_z = self.last_msgs[uav_i] # last predict made by other UAV
            delay_est = t_now - t_z # delay present in the predict received

            z_corrected = z.copy() # copy the predict received

            v_x = z[3] * np.cos(z[2])
            v_y = z[3] * np.sin(z[2])

            z_corrected[0] = z[0] + v_x * delay_est #extrapolation of x, but using the v_x of the state
            z_corrected[1] = z[1] + v_y * delay_est #extrapolation of y, but using the v_y of the state

            #Suggestion

            #z_ext[2] = z[2] + z[4] * delay_est #extrapolation of theta, but using the w of the state

        else:
            z_corrected = z


        self.last_z_share = z_corrected
        self.last_z_obs = z

        self.last_msgs[uav_i] = (z, t_z) # saving in memory the predict  

        return self.kf.update_fuse(z)
    
    def update_fuse_ood(self, z, t_z, uav_i, t_now):
        raise NotImplementedError()


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

def target_dynamics_linear(t):
    w = np.zeros_like(t)
    theta = w * t
    v_x = 2 * np.ones_like(t)
    v_y =  2 * np.ones_like(t)
    v = np.sqrt(v_x ** 2 + v_y ** 2)
    x = v_x * t
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
            t_z_delay, t_z, z, i_z = z_
            if (t - t_z_delay) >= 0: # check if the message is not too recent
                # update
                kf.update_fuse(z, t_z, i_z, t)
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

                    q[uav_j].append((t + delay, t, x_i, uav_i))

                # update last share time for this uav
                q_ts[uav_i] = t + delay
                t_share[uav_i] = t
        col_write += 1
        t_predict= t
        continue

#Converting list with observations and extrapolations to an array



for i in range(n_uavs):
    n = 0
    t_col = 0
    for t, t_value in enumerate(z_obs[i]):

        # first observations dont extrapolate, due to dont have any observation in memory
        if np.all(z_obs[i][t][0] == z_corr[i][t][0]):
            n += 1
            continue
        else:
            t_array = np.array([z_obs[i][t][1]])
            z_obs[i][t_col] = np.concatenate((t_array, z_obs[i][t][0].flatten()))
            z_corr[i][t_col] = np.concatenate((t_array, z_corr[i][t][0].flatten()))
            t_col += 1

    # deleting the first observations that havent been extrapolated, in order to create a numpy array

    del z_obs[i][len(z_obs[i]) - n:]  
    del z_corr[i][len(z_corr[i]) - n:] 
    del z_masks[i][len(z_masks[i]) - n:] 

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

print(kfs[0].corr_num)
print(np.shape(z_obs))
print(z_obs[0][1,0:10])
print(z_obs[0][1,-10:-1])
print(z_corr[0][1,0:10])
print(z_corr[0][1,-10:-1])
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


for i in range(n_uavs):
    x_, y_ = predicts[i][1,:], predicts[i][2,:]
    plt.plot(x_[:col_write], y_[:col_write])
plt.plot(x,y, 'k')
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

for i, state in enumerate(['x', 'y', 'theta', 'v', 'w']):

    if err_obs_mean[i] < err_corr_mean[i]:
        print(f"{state}-----OBS")
    elif err_obs_mean[i] > err_corr_mean[i]:
        print(f"{state}-----CORR") 
    else:
        print(f"{state} state not interpolated")



# print("RMSE x:  ", rmse[0])
# print("RMSE y:  ", rmse[1])
# print("RMSE theta:  ", rmse[2])
# print("RMSE v:  ", rmse[3])
# print("RMSE w:  ", rmse[4])

print("Accuracy: ", np.mean(euclidean))
print("Precision: ", np.mean(dist_uavs))