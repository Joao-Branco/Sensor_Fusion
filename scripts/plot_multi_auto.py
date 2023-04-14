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

bag_fn = r'/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi1681381667/multi1681381667.bag'

#def run_multi_plots(bag_fn, folder=None):
uav_total = 2
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
    
min = min(minimos)   
dataframes['/target_position'].iloc[:,0] = dataframes['/target_position'].iloc[:,0] - min 

for i in range(1, uav_total+1):
    dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] - min 
    dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] - min



target = dataframes['/target_position']
target.columns = ['Time', 'x_target', 'y_target']
target["Time"] = pd.to_timedelta( target["Time"], "sec")
target_index = target.set_index('Time')
data = {}
data_error = {}








for i in range(1, uav_total+1):

    data[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/target_position_fuse']
    data[f'uav{str(i)}']["Time"] = pd.to_timedelta( data[f'uav{str(i)}']["Time"], "sec")
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
    
    
    print(data_error[f'uav{str(i)}'])
    


    
    


    
