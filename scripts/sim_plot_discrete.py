import numpy as np
import matplotlib.pyplot as plt
import os


uav_path = '/Users/joao/Documents/Tese/git/Sensor_Fusion/numerical_sims/sim_1694081400/centralized/data/linear_path/EKF_Falseshare_True_freq_share_5_strategy_augmented_state_delay_0.1/uav_0'

dir_list = os.listdir(uav_path)
dir_list.sort()

state = []
kf = []


for t, step in enumerate(dir_list):
    step_path = os.path.join(uav_path, step)

    state_discrete = np.loadtxt(step_path + '/x_target.txt')
    x_discrete = np.loadtxt(step_path + '/x.txt')

    state.append(state_discrete)
    if (x_discrete.shape[0] == 4):
        var_list = ['x', 'y', 'vx', 'vy']
        kf.append(x_discrete[:4])
        axis = [ _ for _  in var_list]
        figure, ((axis[0], axis[1]), (axis[2], axis[3])) = plt.subplots(2, 2, figsize=(14, 6))
    elif(x_discrete.shape[0] == 5):
        var_list = ['x', 'y', 'w', 'vx', 'vy',]
        kf.append(x_discrete[:5])
        axis = [ _ for _  in var_list]
        figure, ((axis[0], axis[1], axis[2]), (axis[3], axis[4], z)) = plt.subplots(2, 3 , figsize=(14, 6))

    state_arr = np.array(state)
    kf_arr = np.array(kf)

    if(x_discrete.shape[0] == 5):
        z.plot(state_arr[:,0], state_arr[:,1], label = 'state')
        z.plot(kf_arr[:,0], kf_arr[:,1], 'x' , label = 'kf')
        z.grid()
        z.set_title('Position')

    for i, label in enumerate(var_list):
        axis[i].plot(state_arr[:,i], label = 'state')
        axis[i].plot(kf_arr[:,i], 'x' , label = 'kf')
        axis[i].grid()
        axis[i].set_title(label)


    figure.suptitle(str(step))
    figure.show()    

    input("Press Enter")
    plt.close()
 
        

    

    

    

    












