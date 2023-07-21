import target_dynamics
import sim_core
import sim_opt_plot
from pathlib import Path
import time
import time as pytime
import pandas as pd

#path to create a folder with the results from the simulations

SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}")
sim_dir.mkdir()

# simulation parameters
n_uavs = 2
f = 200
sim_time = 10
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
delay_lst = [False, True]
share_lst = [False, True]
ekf_lst = [False, True]
out_of_order_lst = [False, True]


dynamics_lst = [target_dynamics.circular_path,
                target_dynamics.linear_path,
                target_dynamics.sin_path]

delay_strategy_list = [None, 
                       "extrapolate",
                       "extrapolate_plus",
                       "augmented_state"]


OUT_OF_ORDER = False

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

all_data = []

t_start = pytime.time()

for delay in delay_lst:
    dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}")
    dir.mkdir()
    dir_plots = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/plots")
    dir_plots.mkdir()
    dir_results = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/results")
    dir_results.mkdir()
    dir_data = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/data")
    dir_data.mkdir()

    for dynamics in dynamics_lst:
        dir_plots = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/plots/{str(dynamics.__name__)}")
        dir_plots.mkdir()
        dir_results = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/results/{str(dynamics.__name__)}")
        dir_results.mkdir()
        dir_data = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/data/{str(dynamics.__name__)}")
        dir_data.mkdir()

        for delay_strategy in delay_strategy_list:

            for share in share_lst:

                for ekf in ekf_lst:

                    if (delay == True):
                        DELAY_MEAN = 0.3
                        DELAY_STD = 0.01  # 0=guarantees no out of order, but removes randomness in delay
                    else:
                        DELAY_MEAN = 0
                        DELAY_STD = 0

                    if (delay == False and delay_strategy != None or share == False and delay == True):
                        continue
                    if (delay_strategy == "augmented_state" and ekf == False):
                        continue

                    if (delay_strategy == "augmented_state"):
                        aug_states = 10
                    else:
                        aug_states = 0


                    dir_data = Path(f"/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/data/{str(dynamics.__name__)}/EKF_{ekf}_Share_{share}_strategy_{delay_strategy}")
                    dir_data.mkdir()
                    print("SHARE_ON   --- ", share)
                    print("EKF   --- ", ekf)
                    print("DELAY   --- ", delay)
                    print("DELAY STRATEGY   --- ", delay_strategy)
                    print("OUT_OF_ORDER   --- ", OUT_OF_ORDER)
                    print("TARGET DYNAMIC   --- ", dynamics.__name__)
                    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y = sim_core.sim(DELAY_STRATEGY= delay_strategy, EKF=ekf, OUT_OF_ORDER= OUT_OF_ORDER, SHARE_ON= share, DELAY_MEAN= DELAY_MEAN, DELAY_STD= DELAY_STD,  SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD = SENSOR_STD,  sim_time = sim_time, n_uavs = n_uavs, f_sim = f, f_kf = f_kf, f_sample = f_sample, f_share = f_share, target_dynamics = dynamics, AUG = aug_states, dir = dir_data) 
                    accuracy, precision = sim_opt_plot.sim_plot(state, predicts, predict_masks, n_uavs, col_write, x, y,  z_obs, z_corr, z_masks, delay, delay_strategy, ekf, share, dynamics.__name__, str(dir_plots), str(dir_results))
                    all_data.append([share, delay, ekf, delay_strategy, str(dynamics.__name__), accuracy, precision])


column_values = ['Share', 'Delay', 'EKF', 'Delay strategy', 'Dynamics', 'accuracy', 'precision']
dataframe = pd.DataFrame(all_data, columns = column_values)
dataframe.to_csv(f'/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}/performance.csv')

# state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y = sim_core.sim(DELAY_STRATEGY= 'augmented_state', EKF=True, OUT_OF_ORDER= False, SHARE_ON= True, DELAY_MEAN= 0.5, DELAY_STD= 0.01,  SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD = SENSOR_STD,  sim_time = sim_time, n_uavs = n_uavs, f_sim = f, f_kf = f_kf, f_sample = f_sample, f_share = f_share, target_dynamics = target_dynamics.sin_path, AUG = 10) 
# accuracy, precision = sim_opt_plot.sim_plot(state, predicts, predict_masks, n_uavs, col_write, x, y,  z_obs, z_corr, z_masks, True, 'augmented_state', True, str(dir))



print(f'Finished all simulation in {pytime.time() - t_start} seconds')