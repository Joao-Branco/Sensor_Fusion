import target_dynamics
import sim_core
import sim_opt_plot

# simulation parameters
n_uavs = 3
f = 200
sim_time = 60
f_sample = 10
f_kf = 20
f_share = 5

SENSOR_STD = 2.0
SENSOR_MEAN = 0

# Operations parameters
# DELAY_MEAN = 0.5
# DELAY_STD = 0.01  # 0=guarantees no out of order, but removes randomness in delay
# SHARE_ON = True
# EKF = True
# DELAY = True
# OUT_OF_ORDER = True



#### FOR LATER DON'T USE YET
delay_lst = [True, False]
share_lst = [True, False]
ekf_lst = [True, False]
out_of_order_lst = [True, False]


dynamics_lst = [target_dynamics.circular_path,
                target_dynamics.linear_path,
                target_dynamics.sin_path]

delay_strategy_list = [None, 
                       "extrapolate",
                       "extrapolate_plus"]



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

for ekf in ekf_lst:
    for share in share_lst:
        for delay in delay_lst:
                for delay_strategy in delay_strategy_list:
                    if (delay_strategy == None):
                        OUT_OF_ORDER = False
                    else: 
                        OUT_OF_ORDER = True

                    for dynamics in dynamics_lst:
                        if (delay == True):
                            DELAY_MEAN = 0.5
                            DELAY_STD = 0.01  # 0=guarantees no out of order, but removes randomness in delay
                        else:
                            DELAY_MEAN = 0
                            DELAY_STD = 0
                        print("SHARE_ON   --- ", share)
                        print("EKF   --- ", ekf)
                        print("DELAY   --- ", delay)
                        print("DELAY STRATEGY   --- ", delay_strategy)
                        print("OUT_OF_ORDER   --- ", OUT_OF_ORDER)
                        state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y = sim_core.sim(DELAY_STRATEGY= delay_strategy, EKF=ekf, OUT_OF_ORDER= OUT_OF_ORDER, SHARE_ON= share, DELAY_MEAN= DELAY_MEAN, DELAY_STD= DELAY_STD,  SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD = SENSOR_STD,  sim_time = sim_time, n_uavs = n_uavs, f_sim = f, f_kf = f_kf, f_sample = f_sample, f_share = f_share, target_dynamics = dynamics) 
                        sim_opt_plot.sim_plot(state, predicts, predict_masks, n_uavs, col_write, x, y,  z_obs, z_corr, z_masks, OUT_OF_ORDER)

