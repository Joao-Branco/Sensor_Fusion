import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib
matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})
import matplotlib.pyplot as plt
import numpy as np
from functools import reduce
from scipy import signal
import os.path
import sklearn.metrics
from matplotlib.animation import FuncAnimation, PillowWriter




def run_auto_plots(bag_fn, uav_total, single, delay, delay_estimation, fuse, folder_png=None, folder_pgf=None, folder_sim=None, name= None):
    if single == True:
        uav_total = 1
        
        
    
    
    topics = ['/target_position']

    topics_not_time = []
    


    for i in range(1, uav_total + 1):

        topics.append('/uav' + str(i) + '/target_position') # Noise
        if (fuse == True):
            topics.append('/uav' + str(i) + '/target_position_fuse') # After throotle or after buffer
        topics.append('/uav' + str(i) + '/target_position_estimation') # Kalman filter
        if (delay == True):
            topics.append('/uav' + str(i) + '/delay')
            topics_not_time.append('/uav' + str(i) + '/delay')
        if (delay_estimation == True):
            topics.append('/uav' + str(i) + '/delay_estimation')
            topics.append('/uav' + str(i) + '/interpolation')
            topics_not_time.append('/uav' + str(i) + '/delay_estimation')
            topics_not_time.append('/uav' + str(i) + '/interpolation')



    # Give filename of rosbag
    b = bagreader(bag_fn)

    # load messages to dataframes
    dataframes = {}
    minimos = []

    for topic in topics:
        data = b.message_by_topic(topic)
        print(data)
        df = pd.read_csv(data)
        if topic not in topics_not_time:
            df['Timestamp'] = df['timestamp.secs'] + df['timestamp.nsecs'] * 1e-9
            df.drop(['timestamp.secs','timestamp.nsecs'], axis='columns', inplace=True)
        dataframes[topic] = df
        minimos.append(dataframes[topic].iloc[0,0])

    minimo = min(minimos)   
    dataframes['/target_position'].iloc[:,0] = dataframes['/target_position'].iloc[:,0] - minimo

    for i in range(1, uav_total+1):
        dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] - minimo 
        if (fuse == True):
            dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] - minimo
        dataframes[f'/uav{str(i)}/target_position_estimation'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_estimation'].iloc[:,0] - minimo
        if (delay == True):
            dataframes[f'/uav{str(i)}/delay'].iloc[:,0] = dataframes[f'/uav{str(i)}/delay'].iloc[:,0] - minimo
        if (delay_estimation == True):
            dataframes[f'/uav{str(i)}/delay_estimation'].iloc[:,0] = dataframes[f'/uav{str(i)}/delay_estimation'].iloc[:,0] - minimo
            dataframes[f'/uav{str(i)}/interpolation'].iloc[:,0] = dataframes[f'/uav{str(i)}/interpolation'].iloc[:,0] - minimo
            



    target = dataframes['/target_position']
    target.columns = ['Time', 'x_target', 'y_target', 'theta_target', 'v_target', 'w_target', 'uav_id_target', 'Timestamp_target']
    #target.Time = pd.to_timedelta( target.Time, "sec")
    target_index = target.set_index('Time')
    data = {}
    data_error = {}
    delay_corr = {}
    precision = pd.DataFrame()
    precision_counter = 0
    data_all = pd.DataFrame()




    for i in range(1, uav_total+1):

        data[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/target_position_estimation']
        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].set_index('Time')
        real_index = data[f'uav{str(i)}'].index
        data_all = data_all.join(data[f'uav{str(i)}'][['x', 'y']].add_suffix(f'_{str(i)}'), how='outer')
        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].join(target_index, how='outer')

        data_error[f'uav{str(i)}'] = data[f'uav{str(i)}'].interpolate(limit_direction ='both').loc[real_index]

        error_x = abs(data_error[f'uav{str(i)}'].x_target - data_error[f'uav{str(i)}'].x)
        error_y = abs(data_error[f'uav{str(i)}'].y_target - data_error[f'uav{str(i)}'].y)
        error_theta = abs(data_error[f'uav{str(i)}'].theta_target - data_error[f'uav{str(i)}'].theta)
        error_v = abs(data_error[f'uav{str(i)}'].v_target - data_error[f'uav{str(i)}'].v)
        error_w = abs(data_error[f'uav{str(i)}'].w_target - data_error[f'uav{str(i)}'].w)

        
        euclidean = np.sqrt(error_x * error_x + error_y * error_y)
        

        error_Timestamp = abs(data_error[f'uav{str(i)}'].Timestamp_target - data_error[f'uav{str(i)}'].Timestamp)



        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_x", error_x)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_y", error_y)
        
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "euclidean", euclidean)

        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_theta", error_theta)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_v", error_v)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_w", error_w)

        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_Timestamp", error_Timestamp)
        
        data_error[f'uav{str(i)}'].reset_index(inplace=True)

        if (delay_estimation == True):
            delay_corr[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/interpolation']
            delay_corr[f'uav{str(i)}'] = delay_corr[f'uav{str(i)}'].tail(-1)
            real_index_delay = delay_corr[f'uav{str(i)}'].index
            delay_corr[f'uav{str(i)}'].columns = ['Time', 'obs_x', 'obs_y', 'obs_theta', 'obs_v', 'obs_w', 'obs_id', 'obs_timestamp_sec', 'obs_timestamp_nsec', 'int_x', 'int_y', 'int_theta', 'int_v', 'int_w', 'int_id', 'int_timestamp_sec', 'int_timestamp_nsec']
            delay_corr[f'uav{str(i)}'] = delay_corr[f'uav{str(i)}'].join(target_index, how='outer')
            delay_corr[f'uav{str(i)}'] = delay_corr[f'uav{str(i)}'].interpolate(limit_direction ='both').loc[real_index_delay]
            error_x_obs = abs(delay_corr[f'uav{str(i)}'].x_target - delay_corr[f'uav{str(i)}'].obs_x)
            error_y_obs = abs(delay_corr[f'uav{str(i)}'].y_target - delay_corr[f'uav{str(i)}'].obs_y)
            error_theta_obs = abs(delay_corr[f'uav{str(i)}'].theta_target - delay_corr[f'uav{str(i)}'].obs_theta)
            error_v_obs = abs(delay_corr[f'uav{str(i)}'].v_target - delay_corr[f'uav{str(i)}'].obs_v)
            error_w_obs = abs(delay_corr[f'uav{str(i)}'].w_target - delay_corr[f'uav{str(i)}'].obs_w)

            error_x_int = abs(delay_corr[f'uav{str(i)}'].x_target - delay_corr[f'uav{str(i)}'].int_x)
            error_y_int = abs(delay_corr[f'uav{str(i)}'].y_target - delay_corr[f'uav{str(i)}'].int_y)
            error_theta_int = abs(delay_corr[f'uav{str(i)}'].theta_target - delay_corr[f'uav{str(i)}'].int_theta)
            error_v_int = abs(delay_corr[f'uav{str(i)}'].v_target - delay_corr[f'uav{str(i)}'].int_v)
            error_w_int = abs(delay_corr[f'uav{str(i)}'].w_target - delay_corr[f'uav{str(i)}'].int_w)

            euclidean_obs = np.sqrt(error_x_obs ** 2 + error_y_obs ** 2)

            euclidean_int = np.sqrt(error_x_int ** 2 + error_y_int ** 2)

            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_x_obs", error_x_obs)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_y_obs", error_y_obs)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_theta_obs", error_theta_obs)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_v_obs", error_v_obs)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_w_obs", error_w_obs)
        
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "euclidean_obs", euclidean_obs)

            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_x_int", error_x_int)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_y_int", error_y_int)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_theta_int", error_theta_int)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_v_int", error_v_int)
            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "error_w_int", error_w_int)

            delay_corr[f'uav{str(i)}'].insert(len(delay_corr[f'uav{str(i)}'].columns), "euclidean_int", euclidean_int)

    data_all = data_all.interpolate(limit_direction ='both')

    data_all.to_csv( folder_sim + '/data_all_interpolate_' + name + '.csv')



    for i in range(1, uav_total):
        x_i = data_all.columns.get_loc('x_' + str(i))
        y_i = data_all.columns.get_loc('y_' + str(i))
        for j in range(1, uav_total + 1):
            x_j = data_all.columns.get_loc('x_' + str(j))
            y_j = data_all.columns.get_loc('y_' + str(j))
            if(j > i):
                error_x = abs(data_all.iloc[:,x_i] - data_all.iloc[:,x_j])
                error_y = abs(data_all.iloc[:,y_i] - data_all.iloc[:,y_j])
                distance_uavs = np.sqrt(error_x * error_x + error_y * error_y)
                precision.insert(precision_counter, f'uav{str(i)}_uav{str(j)}', distance_uavs)
                precision_counter = precision_counter + 1


    precision.insert(precision_counter, 'mean', precision.mean(axis=1))

    precision.to_csv( folder_sim + '/precision_' + name + '.csv')






    #Graphs of errors in estimation of uavs in x and y
    
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 2, 1)
    plt.title("Erro absoluto", fontsize=15)

    for i in range(1, uav_total+1):
        plt.plot(data_error[f'uav{str(i)}'].Time, data_error[f'uav{str(i)}'].error_x, 'x', label='UAV ' + str(i))

    plt.xlabel('Tempo (s)', fontsize=15)
    plt.ylabel('X (m)', fontsize=15)
    if (single == False):
        plt.legend(fontsize=10)
    plt.grid()

    plt.subplot(1, 2, 2)
    plt.title("Erro absoluto", fontsize=15)

    for i in range(1, uav_total+1):
        plt.plot(data_error[f'uav{str(i)}'].Time, data_error[f'uav{str(i)}'].error_y, 'x', label='UAV ' + str(i))

    plt.xlabel('Tempo (s)', fontsize=15)
    plt.ylabel('Y (m)', fontsize=15)
    if (single == False):
        plt.legend(fontsize=10)
    plt.grid()

    im_basename_png = 'Errors_' + name + '.png'
        
    im_basename_pgf = 'Errors_' + name + '.pgf'




    im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
    plt.savefig(im_fn_png)
    im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
    plt.savefig(im_fn_pgf)

    #Graphs of errors in delay estimation 
    if(delay_estimation == True):
    
        plt.figure(figsize=(15, 5))

        plt.subplot(1, 2, 1)
        plt.title("Erro absoluto", fontsize=15)

        for i in range(1, uav_total+1):
            plt.plot(delay_corr[f'uav{str(i)}'].Time, delay_corr[f'uav{str(i)}'].error_x_obs, 'x', label='UAV ' + str(i) + 'observado')
            plt.plot(delay_corr[f'uav{str(i)}'].Time, delay_corr[f'uav{str(i)}'].error_x_int, 'x', label='UAV ' + str(i) + 'interpolado')

        plt.xlabel('Tempo (s)', fontsize=15)
        plt.ylabel('X (m)', fontsize=15)
        if (single == False):
            plt.legend(fontsize=10)
        plt.grid()

        plt.subplot(1, 2, 2)
        plt.title("Erro absoluto", fontsize=15)

        for i in range(1, uav_total+1):
            plt.plot(delay_corr[f'uav{str(i)}'].Time, delay_corr[f'uav{str(i)}'].error_y_obs, 'x', label='UAV ' + str(i) + 'observado')
            plt.plot(delay_corr[f'uav{str(i)}'].Time, delay_corr[f'uav{str(i)}'].error_y_int, 'x', label='UAV ' + str(i)+ 'interpolado')

        plt.xlabel('Tempo (s)', fontsize=15)
        plt.ylabel('Y (m)', fontsize=15)
        if (single == False):
            plt.legend(fontsize=10)
        plt.grid()

        im_basename_png = 'Errors_delay' + name + '.png'
            
        im_basename_pgf = 'Errors_delay' + name + '.pgf'




        im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
        plt.savefig(im_fn_png)
        im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
        plt.savefig(im_fn_pgf)




    #Graphs of position target and estimation of uavs
    
    plt.figure(figsize=(6, 6))
    
    if (single == True):
        plt.plot(data_error['uav1'].x, data_error['uav1'].y, 'x', markersize=4, label='UAV 1')
        plt.plot(dataframes['/uav1/target_position'].x, dataframes['/uav1/target_position'].y, 'm.', markersize=2, label="Alvo com ruido")

            
        
    else:
        for i in range(1, uav_total+1):
            plt.plot(data_error[f'uav{str(i)}'].x, data_error[f'uav{str(i)}'].y, 'x', markersize=5, label='UAV ' + str(i))


    plt.plot(target.x_target, target.y_target, 'k',linewidth='3', label="Alvo")
    plt.plot(target.iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    plt.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    
    
    
    plt.title("Posição", fontsize=20)
    plt.xlabel('X (m)', fontsize=15)
    plt.ylabel('Y (m)', fontsize=15)
    plt.legend(fontsize=10)
    plt.grid()


    im_basename_png = 'Position_' + name + '.png'
        
    im_basename_pgf = 'Position_' + name + '.pgf'
        

    im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
    plt.savefig(im_fn_png)
    im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
    plt.savefig(im_fn_pgf)
    
    
    #GIFs of position of target and estimation of uavs
    
    
    
    # if (single == True):
    #     fig,ax = plt.subplots()
    #     def animate(i):
    #         ax.clear()
    #         line, = ax.plot(target.iloc[0:i,1], target.iloc[0:i,2], 'k',linewidth='3', label="Alvo")
    #         setpoints1, = ax.plot(data_error['uav1'].iloc[0:i,1], data_error['uav1'].iloc[0:i,2], 'x', markersize=4, label='UAV 1')
    #         setpoints2, = ax.plot(dataframes['/uav1/target_position'].iloc[0:i,1], dataframes['/uav1/target_position'].iloc[0:i,2], 'm.', markersize=2, label="Alvo com ruido")
    #         point1, = ax.plot(target.iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    #         point2, = ax.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    #         point3, = ax.plot(target.iloc[i,1], target.iloc[i,2], 'ko',  markersize=8)
    #         return line, setpoints1, setpoints2 , point1, point2, point3,
        

    #     im_basename_gif = "Position_single.gif"
    #     im_fn_gif = os.path.join(folder_png, im_basename_gif) if folder_png else im_basename_gif
    #     ani = FuncAnimation(fig, animate, interval=40, blit=True, repeat=True, frames = len(target.index))    
    #     ani.save(im_fn_gif, writer=PillowWriter(fps=20))
        
    # else:
    #     fig,ax = plt.subplots()
    #     def animate(i):
    #         ax.clear()
    #         setpoints = []
    #         line, = ax.plot(target.iloc[0:i,1], target.iloc[0:i,2], 'k',linewidth='3', label="Alvo")
    #         for j in range(1, uav_total+1):
    #             setpoints.append(ax.plot(data_error[f'uav{str(j)}'].iloc[0:i,1], data_error[f'uav{str(j)}'].iloc[0:i,2], 'x', markersize=5, label='UAV ' + str(i)))
    #         point1, = ax.plot(target.iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    #         point2, = ax.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    #         point3, = ax.plot(target.iloc[i,1], target.iloc[i,2], 'ko',  markersize=8)
    #         return line, setpoints, point1, point2, point3,
        
    #     im_basename_gif = "Position_multi.gif"
    #     im_fn_gif = os.path.join(folder_png, im_basename_gif) if folder_png else im_basename_gif
    #     ani = FuncAnimation(fig, animate, interval=40, blit=True, repeat=True, frames = len(target.index))    
    #     ani.save(im_fn_gif, writer=PillowWriter(fps=20))
        
    
    
    #Graphs of delays
    
    # if (delay == True):
    #     for i in range(1, uav_total+1):
    #         plt.figure(figsize=(15, 5))
    #         plt.plot(dataframes[f'/uav{str(i)}/delay'].Time, dataframes[f'/uav{str(i)}/delay'].data, 'x', markersize=5, label='true value')
    #         if (delay_estimation == True):
    #             plt.plot(dataframes[f'/uav{str(i)}/delay_estimation'].Time, dataframes[f'/uav{str(i)}/delay_estimation'].data, 'x', markersize=5, label='estimation')
    #         plt.title('UAV ' + str(i), fontsize=20)
    #         plt.xlabel('t (s)', fontsize=15)
    #         plt.ylabel('Delay (s)', fontsize=15)
    #         plt.legend(fontsize=10)
    #         plt.grid()
    #         im_basename_png = 'Delay_UAV' + str(i) + '.png'
    #         im_basename_pgf = 'Delay_UAV' + str(i) + '.pgf'
    #         im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
    #         plt.savefig(im_fn_png)
    #         im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
    #         plt.savefig(im_fn_pgf)

    #         fig,ax = plt.subplots()
    #         def animate(j):
    #             ax.clear()
    #             setpoints = []
    #             line, = ax.plot(target.iloc[0:j,1], target.iloc[0:j,2], 'k',linewidth='3', label="Alvo")
    #             point1, = ax.plot(dataframes[f'/uav{str(i)}/msg_correction'].iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    #             point2, = ax.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    #             point3, = ax.plot(target.iloc[i,1], target.iloc[i,2], 'ko',  markersize=8)
    #             point4, = ax.plot(dataframes[f'/uav{str(i)}/msg_correction'].iloc[j,1], dataframes[f'/uav{str(i)}/msg_correction'].iloc[j,2], 'px', markersize=8)
    #             point5, = ax.plot(dataframes[f'/uav{str(i)}/msg_true'].iloc[j,1], dataframes[f'/uav{str(i)}/msg_true'].iloc[j,2], 'kx', markersize=8)
    #             return line, point1, point2, point3, point4, point5,
        
    #         im_basename_gif = 'Corrections' + str(i) + '.gif'
    #         im_fn_gif = os.path.join(folder_png, im_basename_gif) if folder_png else im_basename_gif
    #         ani = FuncAnimation(fig, animate, interval=40, blit=True, repeat=True, frames = len(target.index))    
    #         ani.save(im_fn_gif, writer=PillowWriter(fps=20))
            
        
    # data of errors and measurments
    
    
    error_fusion = pd.DataFrame(columns=['UAV','error_x', 'error_y', 'RMSE_x', 'RMSE_y', 'euclidean', 'error_theta', 'error_v', 'error_w', 'RMSE_theta', 'RMSE_v', 'RMSE_w', 'error_Timestamp'])
    error_delay_all = pd.DataFrame(columns=['UAV','error_x_obs', 'error_y_obs', 'error_theta_obs', 'error_v_obs', 'error_w_obs', 'euclidean_obs','error_x_int', 'error_y_int', 'error_theta_int', 'error_v_int', 'error_w_int', 'euclidean_int'])
    
    

    for i in range(1, uav_total+1):
        data_error[f'uav{str(i)}'].to_csv( folder_sim + f'/uav{str(i)}.csv')
        error_fusion.loc[len(error_fusion)] = [f'uav{str(i)}' ,
                                                np.mean(data_error[f'uav{str(i)}'].error_x) ,
                                                np.mean(data_error[f'uav{str(i)}'].error_y),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].x_target, data_error[f'uav{str(i)}'].x)),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].y_target, data_error[f'uav{str(i)}'].y)),
                                                np.mean(data_error[f'uav{str(i)}'].euclidean),
                                                np.mean(data_error[f'uav{str(i)}'].error_theta),
                                                np.mean(data_error[f'uav{str(i)}'].error_v),
                                                np.mean(data_error[f'uav{str(i)}'].error_w),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].theta_target, data_error[f'uav{str(i)}'].theta)),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].v_target, data_error[f'uav{str(i)}'].v)),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].w_target, data_error[f'uav{str(i)}'].w)),
                                                np.mean(data_error[f'uav{str(i)}'].error_Timestamp)]
        
        if (delay_estimation == True):
            delay_corr[f'uav{str(i)}'].to_csv( folder_sim + f'/uav{str(i)}_interpolation.csv')
            error_delay_all.loc[len(error_delay_all)] = [f'uav{str(i)}' ,
                                                np.mean(delay_corr[f'uav{str(i)}'].error_x_obs) ,
                                                np.mean(delay_corr[f'uav{str(i)}'].error_y_obs),
                                                np.mean(delay_corr[f'uav{str(i)}'].error_theta_obs),
                                                np.mean(delay_corr[f'uav{str(i)}'].error_v_obs),
                                                np.mean(delay_corr[f'uav{str(i)}'].error_w_obs),
                                                np.mean(delay_corr[f'uav{str(i)}'].euclidean_obs),
                                                np.mean(delay_corr[f'uav{str(i)}'].error_x_int) ,
                                                np.mean(delay_corr[f'uav{str(i)}'].error_y_int),
                                                np.mean(delay_corr[f'uav{str(i)}'].error_theta_int),
                                                np.mean(delay_corr[f'uav{str(i)}'].error_v_int),
                                                np.mean(delay_corr[f'uav{str(i)}'].error_w_int),
                                                np.mean(delay_corr[f'uav{str(i)}'].euclidean_int),]

            error_delay_all.to_csv( folder_sim + '/error_fusion_delay_' + name + '.csv')

        
            
            
        error_fusion.to_csv( folder_sim + '/error_fusion_' + name + '.csv')


        total_precision = np.mean(precision)
        total_euclidean = np.mean(error_fusion.euclidean)
        total_error_x = np.mean(error_fusion.error_x)
        total_error_y = np.mean(error_fusion.error_y)
        total_error_theta = np.mean(error_fusion.error_theta)
        total_error_v = np.mean(error_fusion.error_v)
        total_error_w = np.mean(error_fusion.error_w)
        total_RMSEx = np.mean(error_fusion.RMSE_x)
        total_RMSEy = np.mean(error_fusion.RMSE_y)
        total_RMSEtheta = np.mean(error_fusion.RMSE_theta)
        total_RMSEv = np.mean(error_fusion.RMSE_v)
        total_RMSEw = np.mean(error_fusion.RMSE_w)
        performance = pd.DataFrame([[total_precision, total_euclidean, total_error_x, total_error_y, total_error_theta, total_error_v, total_error_w, total_RMSEx, total_RMSEy, total_RMSEtheta, total_RMSEv, total_RMSEw]], columns=['precision', 'accuracy', 'error_x','error_y', 'error_theta', 'error_v', 'error_w', 'RMSE_x', 'RMSE_y', 'RMSE_theta', 'RMSE_v', 'RMSE_w'])

        performance.to_csv( folder_sim + '/performance_' + name + '.csv')

        if (delay_estimation == True):
            total_error_x_obs = np.mean(error_delay_all.error_x_obs)
            total_error_y_obs = np.mean(error_delay_all.error_y_obs)
            total_error_theta_obs = np.mean(error_delay_all.error_theta_obs)
            total_error_v_obs = np.mean(error_delay_all.error_v_obs)
            total_error_w_obs = np.mean(error_delay_all.error_w_obs)
            total_euclidean_obs = np.mean(error_delay_all.euclidean_obs)

            total_error_x_int = np.mean(error_delay_all.error_x_int)
            total_error_y_int = np.mean(error_delay_all.error_y_int)
            total_error_theta_int = np.mean(error_delay_all.error_theta_int)
            total_error_v_int = np.mean(error_delay_all.error_v_int)
            total_error_w_int = np.mean(error_delay_all.error_w_int)
            total_euclidean_int = np.mean(error_delay_all.euclidean_int)

            performance_interpolation = pd.DataFrame([['obs',
                                                        total_error_x_obs,
                                                        total_error_y_obs, 
                                                        total_error_theta_obs, 
                                                        total_error_v_obs, 
                                                        total_error_w_obs, 
                                                        total_euclidean_obs],
                                                        ['int',
                                                        total_error_x_int,
                                                        total_error_y_int, 
                                                        total_error_theta_int, 
                                                        total_error_v_int, 
                                                        total_error_w_int, 
                                                        total_euclidean_int]], 
                                                        columns=['y','error_x','error_y', 'error_theta', 'error_v', 'error_w', 'euclidean'])
            
            performance_interpolation.to_csv( folder_sim + '/performance_interpolation_' + name + '.csv')







