import target_dynamics_copy as target_dynamics
import sim_core
import sim_opt_plot
import sim_opt_compare
from pathlib import Path
import time
import time as pytime
import pandas as pd
import math
import numpy as np
import os

def check_target_ret(func, t):
    args = func(t)
    for i, a in enumerate(args):
        if not isinstance(a, np.ndarray):
            raise TypeError(f"ret value {i} is not numpy array")
    return args

def directories_create(dir):
    dir_plots = os.path.join(dir, "plots")
    os.mkdir(dir_plots)
    dir_results = os.path.join(dir, "results")
    os.mkdir(dir_results)
    dir_data = os.path.join(dir, "data")
    os.mkdir(dir_data)
    dir_compare = os.path.join(dir, "compare")
    os.mkdir(dir_compare)

    return dir_plots, dir_results, dir_data, dir_compare

def directories_create_dyn(dir, dynamics):
    dir = os.path.join(dir, f"{str(dynamics.__name__)}")
    os.mkdir(dir)

    return dir

#path to create a folder with the results from the simulations

SIM_ID = int(time.time())
sim_dir = Path(f"/Users/joao/Documents/Tese/git/Sensor_fusion/numerical_sims/sim_{SIM_ID}")
sim_dir.mkdir()


# simulation parameters
n_uavs = 3
f = 200
sim_time = 60
f_sample = 10
f_kf = 20
f_share = 5

SENSOR_STD = 10
SENSOR_MEAN = 0


#### FOR LATER DON'T USE YET
centr_lst = [True, False]
delay_lst = [False, True]
share_lst = [False, True]
ekf_lst = [False, True]
out_of_order_lst = [False, True]
delay_mean_lst = [0, 0.1 , 0.5 , 1]
delay_std_lst = [0, 0.01 , 0.02 , 0.05]
pi_lst = [0]
delay_mix = [0, 1, 2]

sim_lst = [0, 1, 2]


dynamics_lst = [target_dynamics.stoped_path,
                target_dynamics.linear_path,
                target_dynamics.sin_path,
                target_dynamics.circular_path]



delay_strategy_list = [None, 
                       "extrapolate",
                       "augmented_state"]


OUT_OF_ORDER = False

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


t_start = pytime.time()


##SINGLE#######
dir = os.path.join(sim_dir, 'single')
os.mkdir(dir)
dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)


for dynamics in dynamics_lst:

    performance = []
    time = np.arange(0, sim_time, 1/f)
    dyn = check_target_ret(dynamics, time)

    dir_plots = directories_create_dyn(dir_plots_main, dynamics)
    dir_results = directories_create_dyn(dir_results_main, dynamics)
    dir_data = directories_create_dyn(dir_data_main, dynamics)
    
    data = []
    
    for ekf in ekf_lst:

        n_uavs = 1
        print("EKF   --- ", ekf)
        print("TARGET DYNAMIC   --- ", dynamics.__name__)
        state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time = sim_core.sim(DELAY_STRATEGY= None, EKF=ekf, OUT_OF_ORDER= False, SHARE_ON= False, DELAY_MEAN= 0, DELAY_STD= 0,  SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD = SENSOR_STD,  sim_time = sim_time, n_uavs = n_uavs, f_sim = f, f_kf = f_kf, f_sample = f_sample, f_share = f_share, target_dynamics = dynamics, AUG = 0, dir = dir_data, state= dyn, PI = 0, CENTR = False) 
        accuracy, precision, euclidean = sim_opt_plot.sim_plot(state, predicts, predict_masks, n_uavs, col_write, x, y,  z_obs, z_corr, z_masks, 0, None, ekf, False, dynamics.__name__, str(dir_plots), str(dir_results), sensors, time, f_share)
        performance.append([ekf, str(dynamics.__name__), accuracy, precision, computer_cost])
        data.append([euclidean, predicts, f"EKF_{ekf}"])
    
    sim_opt_compare.compare_plots(dir_compare_main, dynamics.__name__, data)

column_values = [ 'EKF', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(performance, columns = column_values) 
dataframe.to_csv(str(dir) + '/performance.csv')

##CENTRALIZED#######

#######################
#####SHARE FREQ############
#######################
n_uavs = 3
dir_main = os.path.join(sim_dir, 'centralized')
os.mkdir(dir_main)
dir = os.path.join(dir_main, 'share_frequency')
os.mkdir(dir)

dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)

performance = []

dir_plots = directories_create_dyn(dir_plots_main, dynamics)
dir_results = directories_create_dyn(dir_results_main, dynamics)
dir_data = directories_create_dyn(dir_data_main, dynamics)

data = []

f_s_lst = [2, 5, 10]


for f_s in f_s_lst:

    print("FREQ SHARE   --- ", f_s)
    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time = sim_core.sim(DELAY_STRATEGY= None, EKF=True, OUT_OF_ORDER= False, SHARE_ON= True, DELAY_MEAN= 0, DELAY_STD= 0,  SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD = SENSOR_STD,  sim_time = sim_time, n_uavs = n_uavs, f_sim = f, f_kf = f_kf, f_sample = f_sample, f_share = f_s, target_dynamics = dynamics, AUG = 0, dir = dir_data, state= dyn, PI = 0, CENTR = True) 
    accuracy, precision, euclidean = sim_opt_plot.sim_plot(state, predicts, predict_masks, n_uavs, col_write, x, y,  z_obs, z_corr, z_masks, 0, None, ekf, False, dynamics.__name__, str(dir_plots), str(dir_results), sensors, time, f_s)
    performance.append([f_s, str(dynamics.__name__), accuracy, precision, computer_cost])
    data.append([euclidean, predicts, f"{f_s} Hz"])

sim_opt_compare.compare_plots(dir_compare_main, dynamics.__name__, data)

column_values = [ 'Freq. Share', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(performance, columns = column_values) 
dataframe.to_csv(str(dir) + '/performance.csv')



#######################
#####DELAY############
#######################


dir = os.path.join(dir_main, 'delay_ctc')
os.mkdir(dir)


dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)

dir_plots = directories_create_dyn(dir_plots_main, dynamics)
dir_results = directories_create_dyn(dir_results_main, dynamics)
dir_data = directories_create_dyn(dir_data_main, dynamics)
    
delay_mean_lst = [0.1 , 0.5 , 1]
delay_std_lst = [0.01 , 0.02 , 0.05]
    
for delay, std_delay in zip(delay_mean_lst, delay_std_lst):
    data = []
    for delay_strategy in delay_strategy_list:

        if (delay_strategy == "augmented_state"):
            aug_states = math.floor(delay / (1 / f_kf)) + math.floor(std_delay / (1 / f_kf))
        else:
            aug_states = 0

        print("DELAY MEAN   --- ", delay)
        print("DELAY STD   --- ", std_delay)
        print("DELAY STRATEGY   --- ", delay_strategy)
        state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time = sim_core.sim(DELAY_STRATEGY= delay_strategy, EKF=False, OUT_OF_ORDER= False, SHARE_ON= True, DELAY_MEAN= delay, DELAY_STD= std_delay,  SENSOR_MEAN = SENSOR_MEAN, SENSOR_STD = SENSOR_STD,  sim_time = sim_time, n_uavs = n_uavs, f_sim = f, f_kf = f_kf, f_sample = f_sample, f_share = f_share, target_dynamics = dynamics, AUG = aug_states, dir = dir_data, state= dyn, PI = 0, CENTR = True) 
        accuracy, precision, euclidean = sim_opt_plot.sim_plot(state, predicts, predict_masks, n_uavs, col_write, x, y,  z_obs, z_corr, z_masks, delay, delay_strategy, False, True, dynamics.__name__, str(dir_plots), str(dir_results), sensors, time, f_share)
        performance.append([delay_strategy, str(dynamics.__name__), accuracy, precision, computer_cost])
        data.append([euclidean, predicts, f"{delay_strategy}"])

    sim_opt_compare.compare_plots(dir_compare_main, dynamics.__name__, data, f"delay{delay}_std_{std_delay}")

    column_values = [ 'Delay_strategy', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
    dataframe = pd.DataFrame(performance, columns = column_values) 
    dataframe.to_csv(str(dir) +  f"/delay{delay}_std_{std_delay}_performance.csv")




##DISTRIBUTED#######

#TODO####
# n_uavs = 3
# dir = os.path.join(sim_dir, '/Distributed')
# dir.mkdir()
# time = np.arange(0, sim_time, 1/f)
# dyn = check_target_ret(target_dynamics.circular_path, time)
