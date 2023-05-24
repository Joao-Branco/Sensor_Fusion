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




def run_auto_plots(bag_fn, uav_total, single, delay, delay_estimation, folder_png=None, folder_pgf=None, folder_sim=None):
    if single == True:
        uav_total = 1
        
        
    
    
    topics = ['/target_position']
    


    for i in range(1, uav_total + 1):

        topics.append('/uav' + str(i) + '/target_position')
        topics.append('/uav' + str(i) + '/target_position_fuse')
        topics.append('/uav' + str(i) + '/target_position_estimation')
        if (delay == True):
            topics.append('/uav' + str(i) + '/delay')
        if (delay_estimation == True):
            topics.append('/uav' + str(i) + '/delay_estimation')



    # Give filename of rosbag
    b = bagreader(bag_fn)

    # load messages to dataframes
    dataframes = {}
    minimos = []

    for topic in topics:
        data = b.message_by_topic(topic)
        print(data)
        df = pd.read_csv(data)
        dataframes[topic] = df
        minimos.append(dataframes[topic].iloc[0,0])

    minimo = min(minimos)   
    dataframes['/target_position'].iloc[:,0] = dataframes['/target_position'].iloc[:,0] - minimo

    for i in range(1, uav_total+1):
        dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] - minimo 
        dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] - minimo
        dataframes[f'/uav{str(i)}/target_position_estimation'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_estimation'].iloc[:,0] - minimo
        if (delay == True):
            dataframes[f'/uav{str(i)}/delay'].iloc[:,0] = dataframes[f'/uav{str(i)}/delay'].iloc[:,0] - minimo
        if (delay_estimation == True):
            dataframes[f'/uav{str(i)}/delay_estimation'].iloc[:,0] = dataframes[f'/uav{str(i)}/delay_estimation'].iloc[:,0] - minimo
            



    target = dataframes['/target_position']
    target.columns = ['Time', 'x_target', 'y_target', 'v_x_target', 'v_y_target', 'Timestamp_sec', 'Timestamp_nsec']
    #target.Time = pd.to_timedelta( target.Time, "sec")
    target_index = target.set_index('Time')
    data = {}
    data_error = {}
    #data_noise = {}
    








    for i in range(1, uav_total+1):

        data[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/target_position_estimation']
        data[f'uav{str(i)}'].drop(data[f'uav{str(i)}'].tail(5).index, inplace = True)
        data[f'uav{str(i)}'].drop(data[f'uav{str(i)}'].index[:5], inplace = True)
        #data_noise[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/target_position']
        #data_noise[f'uav{str(i)}'].columns = ['Time', 'x_noise', 'y_noise']
        #data_noise_index = data_noise[f'uav{str(i)}'].set_index('Time')
        
        #data[f'uav{str(i)}']["Time"] = pd.to_timedelta( data[f'uav{str(i)}']["Time"], "sec")
        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].set_index('Time')
        real_index = data[f'uav{str(i)}'].index

        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].join(target_index, how='outer')

        data_error[f'uav{str(i)}'] = data[f'uav{str(i)}'].interpolate().loc[real_index]

        error_x = (data_error[f'uav{str(i)}'].x_target - data_error[f'uav{str(i)}'].x)
        error_y = (data_error[f'uav{str(i)}'].y_target - data_error[f'uav{str(i)}'].y)
        
        euclidean = np.sqrt(error_x * error_x + error_y * error_y)
        
        error_v_x = (data_error[f'uav{str(i)}'].v_x_target - data_error[f'uav{str(i)}'].v_x)
        error_v_y = (data_error[f'uav{str(i)}'].v_y_target - data_error[f'uav{str(i)}'].v_y)


        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_x", error_x)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_y", error_y)
        
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "euclidean", euclidean)
        
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_v_x", error_v_x)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_v_y", error_v_y)
        
        data_error[f'uav{str(i)}'].reset_index(inplace=True)


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

    if (single == True):
        im_basename_png = "Errors_single.png"
        
        im_basename_pgf = "Errors_single.pgf"

    else:
        im_basename_png = "Errors_multi.png"
        
        im_basename_pgf = "Errors_multi.pgf"


    im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
    plt.savefig(im_fn_png)
    im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
    plt.savefig(im_fn_pgf)


    plt.figure(figsize=(6, 6))

    
    if (single == True):
        plt.plot(data_error['uav1'].x, data_error['uav1'].y, 'x', markersize=4, label='UAV 1')
        plt.plot(dataframes['/uav1/target_position'].x, dataframes['/uav1/target_position'].y, 'm.', markersize=2, label="Alvo com ruido")
        
    else:
        for i in range(1, uav_total+1):
            plt.plot(data_error[f'uav{str(i)}'].x, data_error[f'uav{str(i)}'].y, 'x', markersize=5, label='UAV ' + str(i))
            #plt.plot(dataframes[f'/uav{str(i)}/target_position'].x, dataframes[f'/uav{str(i)}/target_position'].y, '.', markersize=2, label="Alvo com ruido")


    plt.plot(target.x_target, target.y_target, 'k',linewidth='3', label="Alvo")
    plt.plot(target.iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    plt.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    
    
    
    plt.title("Posição", fontsize=20)
    plt.xlabel('X (m)', fontsize=15)
    plt.ylabel('Y (m)', fontsize=15)
    plt.legend(fontsize=10)
    plt.grid()


    if (single == True):
        im_basename_png = "Position_single.png"
        im_basename_pgf = "Position_single.pgf"


    else:
        im_basename_png = "Position_multi.png"
        im_basename_pgf = "Position_multi.pgf"



    im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
    plt.savefig(im_fn_png)
    im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
    plt.savefig(im_fn_pgf)
    
    if (delay == True):
        for i in range(1, uav_total+1):
            plt.figure(figsize=(15, 5))
            plt.plot(dataframes[f'/uav{str(i)}/delay'].Time, dataframes[f'/uav{str(i)}/delay'].data, 'x', markersize=5, label='true value')
            if (delay_estimation == True):
                plt.plot(dataframes[f'/uav{str(i)}/delay_estimation'].Time, dataframes[f'/uav{str(i)}/delay_estimation'].data, 'x', markersize=5, label='estimation')
            plt.title('UAV ' + str(i), fontsize=20)
            plt.xlabel('t (s)', fontsize=15)
            plt.ylabel('Delay (s)', fontsize=15)
            plt.legend(fontsize=10)
            plt.grid()
            im_basename_png = 'Delay_UAV' + str(i) + '.png'
            im_basename_pgf = 'Delay_UAV' + str(i) + '.pgf'
            im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
            plt.savefig(im_fn_png)
            im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
            plt.savefig(im_fn_pgf)
            
        
    
    
    
    error_fusion = pd.DataFrame(columns=['UAV','error_x', 'error_y', 'RMSE_x', 'RMSE_y', 'euclidean', 'error_v_x', 'error_v_y', 'RMSE_v_x', 'RMSE_v_y'])
    

    if (single == True):
        data_error['uav1'].to_csv( folder_sim + '/error_uav_single.csv')
        error_fusion.loc[len(error_fusion)] = ['uav1' , np.mean(data_error['uav1'].error_x) , np.mean(data_error['uav1'].error_y), math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].x_target, data_error['uav1'].x)),  math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].y_target, data_error['uav1'].y)), np.mean(data_error['uav1'].euclidean), np.mean(data_error['uav1'].error_v_x), np.mean(data_error['uav1'].error_v_y), math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].v_x_target, data_error['uav1'].v_x)), math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].v_y_target, data_error['uav1'].v_y))]
        error_fusion.to_csv( folder_sim + f'/error_med_single.csv')
        
    else:    
        for i in range(1, uav_total+1):
            data_error[f'uav{str(i)}'].to_csv( folder_sim + f'/uav{str(i)}.csv')
            error_fusion.loc[len(error_fusion)] = [f'uav{str(i)}' , np.mean(data_error[f'uav{str(i)}'].error_x) , np.mean(data_error[f'uav{str(i)}'].error_y), math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].x_target, data_error[f'uav{str(i)}'].x)),  math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].y_target, data_error[f'uav{str(i)}'].y)), np.mean(data_error[f'uav{str(i)}'].euclidean), np.mean(data_error[f'uav{str(i)}'].error_v_x), np.mean(data_error[f'uav{str(i)}'].error_v_y), math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].v_x_target, data_error['uav1'].v_x)), math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].v_y_target, data_error['uav1'].v_y))]
            
            
        error_fusion.to_csv( folder_sim + f'/error_fusion.csv')





