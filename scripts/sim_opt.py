import target_dynamics
import sim_core
import sim_opt_plot
from pathlib import Path
import time
import time as pytime
import pandas as pd
import math

#path to create a folder with the results from the simulations

SIM_ID = int(time.time())
sim_dir = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}")
sim_dir.mkdir()

#/home/branco/catkin_ws/src/sensor_fusion/numerical_sims/sim_{SIM_ID}

# simulation parameters
n_uavs = 3
f = 200
sim_time = 90
f_sample = 10
f_kf = 20
f_share = 5

SENSOR_STD = 10
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
delay_mean_lst = [0, 0.1 , 0.5 , 1]
delay_std_lst = [0, 0.01 , 0.02 , 0.05]
delay_mean_lst = [0]
delay_std_lst = [0]


dynamics_lst = [target_dynamics.stoped_path,
                target_dynamics.linear_path,
                target_dynamics.sin_path,
                target_dynamics.circular_path,
                target_dynamics.stoped_xy_path,
                target_dynamics.linear_xy_path,
                target_dynamics.sin_xy_path,
                target_dynamics.circular_xy_path]

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

for delay, std_delay in zip(delay_mean_lst, delay_std_lst):
    dir = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}")
    dir.mkdir()
    dir_plots = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/plots")
    dir_plots.mkdir()
    dir_results = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/results")
    dir_results.mkdir()
    dir_data = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/data")
    dir_data.mkdir()

    for dynamics in dynamics_lst:
        dir_plots = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/plots/{str(dynamics.__name__)}")
        dir_plots.mkdir()
        dir_results = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/results/{str(dynamics.__name__)}")
        dir_results.mkdir()
        dir_data = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/data/{str(dynamics.__name__)}")
        dir_data.mkdir()

        for delay_strategy in delay_strategy_list:

            for share in share_lst:

                for ekf in ekf_lst:

                    if (delay != 0):
                            DELAY_MEAN = delay
                            DELAY_STD = std_delay  # 0=guarantees no out of order, but removes randomness in delay
                    else:
                        DELAY_MEAN = 0
                        DELAY_STD = 0

                    if (delay == 0 and delay_strategy != None or share == False and delay != 0):
                        continue
                    if (delay_strategy == "augmented_state" and ekf == False):
                        continue

                    if (delay_strategy == "augmented_state"):
                        aug_states = math.floor(DELAY_MEAN / (1 / f_kf)) + math.floor(DELAY_STD / (1 / f_kf)) + 1
                    else:
                        aug_states = 0



                    dir_data = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/delay_{delay}/data/{str(dynamics.__name__)}/EKF_{ekf}_Share_{share}_strategy_{delay_strategy}_N_{aug_states}")
                    dir_data.mkdir()
                    print("SHARE_ON   --- ", share)
                    print("EKF   --- ", ekf)
                    print("DELAY   --- ", delay)
                    print("DELAY STRATEGY   --- ", delay_strategy)
                    print("OUT_OF_ORDER   --- ", OUT_OF_ORDER)
                    print("TARGET DYNAMIC   --- ", dynamics.__name__)
                    print("DELAY MEAN   --- ", DELAY_MEAN)
                    print("DELAY STD   --- ", DELAY_STD)
                    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time = sim_core.sim(DELAY_STRATEGY= delay_strategy, EKF=ekf, OUT_OF_ORDER= OUT_OF_ORDER, SHARE_ON= share, DELAY_MEAN= DELAY_MEAN, DELAY_STD= DELAY_STD,  SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD = SENSOR_STD,  sim_time = sim_time, n_uavs = n_uavs, f_sim = f, f_kf = f_kf, f_sample = f_sample, f_share = f_share, target_dynamics = dynamics, AUG = aug_states, dir = dir_data) 
                    accuracy, precision = sim_opt_plot.sim_plot(state, predicts, predict_masks, n_uavs, col_write, x, y,  z_obs, z_corr, z_masks, delay, delay_strategy, ekf, share, dynamics.__name__, str(dir_plots), str(dir_results), sensors, time)
                    all_data.append([share, delay, DELAY_STD, ekf, delay_strategy, aug_states, str(dynamics.__name__), accuracy, precision, computer_cost])


column_values = ['Share', 'Delay', 'Delay_std', 'EKF', 'Delay strategy', 'N', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(all_data, columns = column_values) 
dataframe.to_csv(f'/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}/performance.csv')





print(f'Finished all simulation in {pytime.time() - t_start} seconds')