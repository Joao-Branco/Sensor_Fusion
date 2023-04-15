import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
from functools import reduce
from scipy import signal
import os.path
import sklearn.metrics



def run_auto_plots(bag_fn, uav_total, single, folder=None):
    if single == True:
        uav_total = 1
        
        
    print(uav_total)
    print(single)
    print(bag_fn)
    
    
    topics = ['/target_position']


    for i in range(1, uav_total + 1):

        topics.append('/uav' + str(i) + '/target_position')
        topics.append('/uav' + str(i) + '/target_position_fuse')



    # Give filename of rosbag
    b = bagreader(bag_fn)

    # load messages to dataframes
    dataframes = {}
    minimos = []

    for topic in topics:
        data = b.message_by_topic(topic)
        df = pd.read_csv(data)
        dataframes[topic] = df
        minimos.append(dataframes[topic].iloc[0,0])

    minimo = min(minimos)   
    dataframes['/target_position'].iloc[:,0] = dataframes['/target_position'].iloc[:,0] - minimo

    for i in range(1, uav_total+1):
        dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] - minimo 
        dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] - minimo



    target = dataframes['/target_position']
    target.columns = ['Time', 'x_target', 'y_target']
    #target.Time = pd.to_timedelta( target.Time, "sec")
    target_index = target.set_index('Time')
    data = {}
    data_error = {}








    for i in range(1, uav_total+1):

        data[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/target_position_fuse']
        #data[f'uav{str(i)}']["Time"] = pd.to_timedelta( data[f'uav{str(i)}']["Time"], "sec")
        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].set_index('Time')
        real_index = data[f'uav{str(i)}'].index

        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].join(target_index, how='outer')

        data_error[f'uav{str(i)}'] = data[f'uav{str(i)}'].interpolate().loc[real_index]

        error_x = (data_error[f'uav{str(i)}'].x_target - data_error[f'uav{str(i)}'].x)
        error_y = (data_error[f'uav{str(i)}'].y_target - data_error[f'uav{str(i)}'].y)
        euclidean = np.sqrt(error_x * error_x + error_y * error_y)


        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_x", error_x)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_y", error_y)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "euclidean", euclidean)
        data_error[f'uav{str(i)}'].reset_index(inplace=True)


    plt.figure(figsize=(18, 8))

    plt.subplot(2, 1, 1)
    plt.title("Erro absoluto")

    for i in range(1, uav_total+1):
        plt.plot(data_error[f'uav{str(i)}'].Time, data_error[f'uav{str(i)}'].error_x, 'x', label='UAV ' + str(i))

    plt.xlabel('Tempo (s)')
    plt.ylabel('X (m)')
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)

    for i in range(1, uav_total+1):
        plt.plot(data_error[f'uav{str(i)}'].index, data_error[f'uav{str(i)}'].error_y, 'x', label='UAV ' + str(i))

    plt.xlabel('Tempo (s)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid()

    if (single == True):
        im_basename = "Errors_ single.png"

    else:
        im_basename = "Errors_ multi.png"

    im_fn = os.path.join(folder, im_basename) if folder else im_basename
    plt.savefig(im_fn)


    plt.figure(figsize=(18, 8))

    plt.plot(target.x_target, target.y_target, 'k', label="Alvo")

    for i in range(1, uav_total+1):
        plt.plot(data_error[f'uav{str(i)}'].x, data_error[f'uav{str(i)}'].y, 'x', label='UAV ' + str(i))

    plt.title("Posição")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid()


    if (single == True):
        im_basename = "Position_single.png"

    else:
        im_basename = "Position_ multi.png"


    im_fn = os.path.join(folder, im_basename) if folder else im_basename
    plt.savefig(im_fn)

    if (single == True):
        data_error['uav1'].to_csv('error_uav1.csv')
        
    else:    
        for i in range(1, uav_total+1):
            data_error[f'uav{str(i)}'].to_csv(f'uav{str(i)}.csv')





