import target_dynamics, sim_core, sim_opt_plot, sim_opt_compare, iter_simulations
import target_dynamics_2 as target_dynamics
from pathlib import Path
import time
import time as pytime
import matplotlib.font_manager as fm
import pandas as pd
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
# n_uavs = 3
# f = 200
# sim_time = 60
# f_sample = 10
# f_kf = 20
# f_share = 5

# SENSOR_STD = 10
# SENSOR_MEAN = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


t_start = pytime.time()

dynamics_lst = [target_dynamics.stoped_path,
                target_dynamics.linear_path,
                target_dynamics.circular_path, 
                target_dynamics.sin_path]

ekf_lst = [False, True]



##SINGLE#######
dir = os.path.join(sim_dir, 'single')
os.mkdir(dir)
dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)

single_inter = iter_simulations.simulations(dynamic= dynamics_lst, ekf= ekf_lst, n_uavs= [1], sim_time= [60])

data = []
performance = []
dict_plots = {}
dict_results = {}
dict_data = {}
dyn_rep = {}

for dina in dynamics_lst:
    dict_plots[dina.__name__] = directories_create_dyn(dir_plots_main, dina)
    dict_results[dina.__name__] = directories_create_dyn(dir_results_main, dina)
    dict_data[dina.__name__] = directories_create_dyn(dir_data_main, dina)
    time = np.arange(0, single_inter[0][1], 1/single_inter[0][3])
    dyn = check_target_ret(dina, time)
    dyn_rep[dina.__name__] = dyn


for iter in single_inter:
    print(tuple(iter))
    time = np.arange(0, iter[1], 1/iter[3])
    #dyn = check_target_ret(iter[9], time)
    dyn = dyn_rep.get(iter[9].__name__)
    last_dyn = iter[9]

    dir_plots = dict_plots[iter[9].__name__]
    dir_results = dict_results[iter[9].__name__]
    dir_data = dict_data[iter[9].__name__]
    
    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time, sensor_masks, x_obs, delays_uav = sim_core.sim(state= dyn, dir= dir_data, printt= iter[0], sim_time= iter[1], n_uavs= iter[2], f_sim= iter[3], f_kf= iter[4], f_sample= iter[5], f_share= iter[6], SENSOR_MEAN= iter[7][0], SENSOR_STD= iter[7][1], EKF= iter[8], SHARE_ON= iter[10], OUT_OF_ORDER= iter[11], DELAY_STRATEGY= iter[12], delay_d= iter[13], DELAY_MEAN= iter[14][0], DELAY_STD= iter[14][1], AUG= iter[15], PI= iter[16], CENTR= iter[17], Ring_on= iter[18]) 
    accuracy, precision, euclidean, desvio_medio = sim_opt_plot.sim_plot(state= state, predicts= predicts, predict_masks= predict_masks,
                                                            col_write= col_write, x= x, y= y, z_obs= z_obs, z_corr= z_corr,z_masks= z_masks,
                                                            dir_plot= str(dir_plots), dir_result= str(dir_results), sensors= sensors, time= time,
                                                            n_uavs= iter[2], f_s= iter[6],
                                                            ekf= iter[8], share= iter[10], delay_strategy= iter[12], delay= iter[14][0], sensor_masks= sensor_masks, pi= iter[16])
    
    performance.append([iter[8], str(iter[9].__name__), accuracy, precision, computer_cost])
    if iter[8] == True:
        label = "Estimador Rotacional"
    else:
        label = "Estimador Linear"
    data.append([euclidean, predicts, label, iter[9].__name__, state, time, x_obs])


for dina in dynamics_lst:   
    sim_opt_compare.compare_plots(dir_compare_main, dina.__name__, data)

column_values = [ 'EKF', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(performance, columns = column_values) 
dataframe.to_excel(str(dir) + '/performance.xlsx')

##CENTRALIZED#######
dir_main = os.path.join(sim_dir, 'centralized')
os.mkdir(dir_main)


#######################
#####SHARE FREQ############
#######################


f_s_lst = [7.5, 15]

last_dyn = target_dynamics.sin_path

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

dir = os.path.join(dir_main, 'share_frequency')
os.mkdir(dir)

dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)

dir_plots = directories_create_dyn(dir_plots_main, last_dyn)
dir_results = directories_create_dyn(dir_results_main, last_dyn)
dir_data = directories_create_dyn(dir_data_main, last_dyn)

data = []
performance = []

freq_s_inter = iter_simulations.simulations(dynamic= [last_dyn], ekf= [True], n_uavs= [3], f_share= f_s_lst)


uav_4 = iter_simulations.simulations(dynamic= [last_dyn], ekf= [True], n_uavs= [5], f_share= [15])

freq_s_inter.append(uav_4[0])

freq_s_inter = sorted(freq_s_inter, key=lambda x: x[6])


for iter in freq_s_inter:
    print(tuple(iter))
    time = np.arange(0, iter[1], 1/iter[3])
    dyn = check_target_ret(target_dynamics.sin_path, time)
    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time, sensor_masks, x_obs, delays_uav = sim_core.sim(state= dyn, dir= dir_data, printt= iter[0], sim_time= iter[1], n_uavs= iter[2], f_sim= iter[3], f_kf= iter[4], f_sample= iter[5], f_share= iter[6], SENSOR_MEAN= iter[7][0], SENSOR_STD= iter[7][1], EKF= iter[8], SHARE_ON= iter[10], OUT_OF_ORDER= iter[11], DELAY_STRATEGY= iter[12], delay_d= iter[13], DELAY_MEAN= iter[14][0], DELAY_STD= iter[14][1], AUG= iter[15], PI= iter[16], CENTR= iter[17], Ring_on= iter[18]) 
    accuracy, precision, euclidean, desvio_medio = sim_opt_plot.sim_plot(state= state, predicts= predicts, predict_masks= predict_masks,
                                                            col_write= col_write, x= x, y= y, z_obs= z_obs, z_corr= z_corr,z_masks= z_masks,
                                                            dir_plot= str(dir_plots), dir_result= str(dir_results), sensors= sensors, time= time,
                                                            n_uavs= iter[2], f_s= iter[6],
                                                            ekf= iter[8], share= iter[10], delay_strategy= iter[12], delay= iter[14][0], sensor_masks= sensor_masks, pi= iter[16])
    
    accuracy = np.mean(accuracy)
    performance.append([iter[6], accuracy, precision, computer_cost])
    data.append([euclidean, predicts, f"{iter[2]} UAVS {iter[6]} Hz", iter[9].__name__, state, time, x_obs, desvio_medio])

sim_opt_compare.compare_plots_multi(dir_compare_main, iter[9].__name__, data)

column_values = [ 'Freq. Share', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(performance, columns = column_values) 
dataframe.to_excel(str(dir) + '/performance.xlsx')



#######################
#####DELAY############
#######################


delay_lst = [(0.1, 0.01), (0.5, 0.05), (1, 0.1)]
delay_strategy_list = [None, 
                       "extrapolate",
                       "augmented_state"]

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


dir = os.path.join(dir_main, 'delay_ctc')
os.mkdir(dir)

#last_dyn = target_dynamics.sin_path
#if single and centralized commented

dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)

dir_plots = directories_create_dyn(dir_plots_main, last_dyn)
dir_results = directories_create_dyn(dir_results_main, last_dyn)
dir_data = directories_create_dyn(dir_data_main, last_dyn)

data = {}
performance = []

for (mean, std) in delay_lst:
    data[mean] = []



delay_inter = iter_simulations.simulations(dynamic= [target_dynamics.sin_path], ekf= [True], n_uavs= [3], f_share= [10], delay= delay_lst, delay_strategy= delay_strategy_list, share= [True])
    
for iter in delay_inter:
    time = np.arange(0, iter[1], 1/iter[3])
    dyn = check_target_ret(target_dynamics.sin_path, time)
    last_dyn = dyn 
    print(tuple(iter))
    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time, sensor_masks, x_obs, delay_matrix  = sim_core.sim(state= dyn, dir= dir_data, printt= iter[0], sim_time= iter[1], n_uavs= iter[2], f_sim= iter[3], f_kf= iter[4], f_sample= iter[5], f_share= iter[6], SENSOR_MEAN= iter[7][0], SENSOR_STD= iter[7][1], EKF= iter[8], SHARE_ON= iter[10], OUT_OF_ORDER= iter[11], DELAY_STRATEGY= iter[12], delay_d= iter[13], DELAY_MEAN= iter[14][0], DELAY_STD= iter[14][1], AUG= iter[15], PI= iter[16], CENTR= iter[17], Ring_on= iter[18]) 
    accuracy, precision, euclidean, desvio_medio = sim_opt_plot.sim_plot(state= state, predicts= predicts, predict_masks= predict_masks,
                                                            col_write= col_write, x= x, y= y, z_obs= z_obs, z_corr= z_corr,z_masks= z_masks,
                                                            dir_plot= str(dir_plots), dir_result= str(dir_results), sensors= sensors, time= time,
                                                            n_uavs= iter[2], f_s= iter[6],
                                                            ekf= iter[8], share= iter[10], delay_strategy= iter[12], delay= iter[14][0], sensor_masks= sensor_masks, pi= iter[16])
    
    performance.append([iter[12], iter[14][0],  str(iter[9].__name__), accuracy, precision, computer_cost])

    if iter[12] == None:
        label = "Sem correção"
    elif iter[12] ==  "extrapolate":
        label = "Extrapolação"
    else:   
        label = "Estado Aumentado"
    data[iter[14][0]].append([euclidean, predicts, label, iter[9].__name__, state, time, x_obs, desvio_medio, delay_matrix, iter[14]])

for ddd in data:
    sim_opt_compare.compare_plots_multi_delay(dir_compare_main, iter[9].__name__, data[ddd], f"delay{ddd}")

column_values = [ 'Delay_strategy', 'Delay_Mean', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(performance, columns = column_values) 
dataframe.to_csv(str(dir) +  f"/performance.csv")

#######################
#####DELAY_D############
#######################


delay_lst = [(0.001, 0.0001)  , (0.005, 0.0002) , (0.01, 0.005)]
delay_strategy_list = [None, 
                       "extrapolate",
                       "augmented_state"]

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

dir = os.path.join(dir_main, 'delay_d')
os.mkdir(dir)



dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)

dir_plots = directories_create_dyn(dir_plots_main, last_dyn)
dir_results = directories_create_dyn(dir_results_main, last_dyn)
dir_data = directories_create_dyn(dir_data_main, last_dyn)

data = {}
performance = []

for (mean, std) in delay_lst:
    data[mean] = []



delay_d_inter = iter_simulations.simulations(dynamic= [last_dyn], ekf= [True], n_uavs= [3], f_share= [10], delay= delay_lst, delay_strategy= delay_strategy_list, share= [True], delay_d=[True])
    
for iter in delay_d_inter:
    print(tuple(iter))
    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time, sensor_masks, x_obs, delay_matrix = sim_core.sim(state= dyn, dir= dir_data, printt= iter[0], sim_time= iter[1], n_uavs= iter[2], f_sim= iter[3], f_kf= iter[4], f_sample= iter[5], f_share= iter[6], SENSOR_MEAN= iter[7][0], SENSOR_STD= iter[7][1], EKF= iter[8], SHARE_ON= iter[10], OUT_OF_ORDER= iter[11], DELAY_STRATEGY= iter[12], delay_d= iter[13], DELAY_MEAN= iter[14][0], DELAY_STD= iter[14][1], AUG= iter[15], PI= iter[16], CENTR= iter[17], Ring_on= iter[18]) 
    accuracy, precision, euclidean, desvio_medio = sim_opt_plot.sim_plot(state= state, predicts= predicts, predict_masks= predict_masks,
                                                            col_write= col_write, x= x, y= y, z_obs= z_obs, z_corr= z_corr,z_masks= z_masks,
                                                            dir_plot= str(dir_plots), dir_result= str(dir_results), sensors= sensors, time= time,
                                                            n_uavs= iter[2], f_s= iter[6],
                                                            ekf= iter[8], share= iter[10], delay_strategy= iter[12], delay= iter[14][0], sensor_masks= sensor_masks, pi= iter[16])
    
    if iter[12] == None:
        label = "Normal"
    elif iter[12] ==  "extrapolate":
        label = "Extrapolação"
    else:   
        label = "Estado Aumentado"


    performance.append([iter[12], iter[14][0],  str(iter[9].__name__), accuracy, precision, computer_cost])
    data[iter[14][0]].append([euclidean, predicts, label, iter[9].__name__, state, time, x_obs, desvio_medio, delay_matrix, iter[14]])

for ddd in data:
    sim_opt_compare.compare_plots_delay(dir_compare_main, last_dyn.__name__, data[ddd], f"delay{ddd}")

column_values = [ 'Delay_strategy', 'Delay_Mean', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(performance, columns = column_values) 
dataframe.to_csv(str(dir) +  f"/performance.csv")



##DISTRIBUTED#######

dir_main = os.path.join(sim_dir, 'Distributed')
os.mkdir(dir_main)


#######################
#####DELAY_D############
#######################

delay_lst = [(0.001, 0.0001)  , (0.005, 0.0002) , (0.01, 0.005)]
pi_list = [0.8, 0.5, 0.4]
delay_strategy_list = ["augmented_state"]



# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

dir = os.path.join(dir_main, 'delay_d')
os.mkdir(dir)



dir_plots_main, dir_results_main, dir_data_main, dir_compare_main = directories_create(dir)

dir_plots = directories_create_dyn(dir_plots_main, last_dyn)
dir_results = directories_create_dyn(dir_results_main, last_dyn)
dir_data = directories_create_dyn(dir_data_main, last_dyn)

data = {}
performance = []

for (mean, std) in delay_lst:
    data[mean] = []



delay_inter = iter_simulations.simulations(dynamic= [last_dyn], ekf= [True], n_uavs= [6], f_share= [10], delay= delay_lst, delay_strategy= delay_strategy_list, share= [True], delay_d=[True], Centre= [False], PI= pi_list, Ring_on= [True])
    
for iter in delay_inter:
    print(tuple(iter))
    state, predicts, predict_masks, z_obs, z_corr, z_masks, col_write, x, y, computer_cost, sensors, time, sensor_masks = sim_core.sim(state= dyn, dir= dir_data, printt= iter[0], sim_time= iter[1], n_uavs= iter[2], f_sim= iter[3], f_kf= iter[4], f_sample= iter[5], f_share= iter[6], SENSOR_MEAN= iter[7][0], SENSOR_STD= iter[7][1], EKF= iter[8], SHARE_ON= iter[10], OUT_OF_ORDER= iter[11], DELAY_STRATEGY= iter[12], delay_d= iter[13], DELAY_MEAN= iter[14][0], DELAY_STD= iter[14][1], AUG= iter[15], PI= iter[16], CENTR= iter[17], Ring_on= iter[18]) 
    accuracy, precision, euclidean = sim_opt_plot.sim_plot(state= state, predicts= predicts, predict_masks= predict_masks,
                                                            col_write= col_write, x= x, y= y, z_obs= z_obs, z_corr= z_corr,z_masks= z_masks,
                                                            dir_plot= str(dir_plots), dir_result= str(dir_results), sensors= sensors, time= time,
                                                            n_uavs= iter[2], f_s= iter[6],
                                                            ekf= iter[8], share= iter[10], delay_strategy= iter[12], delay= iter[14][0], sensor_masks= sensor_masks, pi= iter[16])
    
    performance.append([iter[12], iter[14][0],  str(iter[9].__name__), accuracy, precision, computer_cost])
    data[iter[14][0]].append([euclidean, predicts, f"{iter[12]}"])

for ddd in data:
    sim_opt_compare.compare_plots(dir_compare_main, last_dyn.__name__, data[ddd], f"delay{ddd}")

column_values = [ 'Delay_strategy', 'Delay_Mean', 'Dynamics', 'accuracy', 'precision', 'Computer_Cost']
dataframe = pd.DataFrame(performance, columns = column_values) 
dataframe.to_csv(str(dir) +  f"/performance.csv")
