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

#BAG_FN = r'/home/branco/catkin_ws/src/sensor_fusion/bags/single_2023-04-10-15-23-44.bag'


def run_single_plots(bag_fn, folder=None):

    topics = [
        '/target_position',
        '/uav1/target_position',
        '/uav1/target_position_fuse'
        ]


    # Give filename of rosbag
    b = bagreader(bag_fn)

    # load messages to dataframes
    dataframes = {}

    for topic in topics:
        data = b.message_by_topic(topic)
        print("File saved: {}".format(data))
        print(type(data))
        df = pd.read_csv(data)
        dataframes[topic] = df
        
    # Rename all position cols


    dataframes['/target_position'].rename(columns={
        'target_position.x': 'x',
        'target_position.y': 'y',
        'target_position.v_x': 'v_x',
        'target_position.v_y': 'v_y'
    }, inplace=True)

    dataframes['/uav1/target_position'].rename(columns={
        'target_position.x': 'x',
        'target_position.y': 'y'
    }, inplace=True)

    dataframes['/uav1/target_position_fuse'].rename(columns={
        'target_position.x': 'x',
        'target_position.y': 'y',
        'target_position.v_x': 'v_x',
        'target_position.v_y': 'v_y'
    }, inplace=True)

    # Init fig
    plt.figure(figsize=(18, 8))

    # PLOT 1 - X axis

    # Truth target position

    target_position = dataframes['/target_position'].to_numpy()
    uav1_target_position = dataframes['/uav1/target_position'].to_numpy()
    uav1_target_position_fuse = dataframes['/uav1/target_position_fuse'].to_numpy()

    minimo = target_position[0,0]
    maximo = max(target_position[0:,0].size, uav1_target_position[0:,0].size, uav1_target_position_fuse[0:,0].size)
    
    target_position[0:,0] = target_position[0:,0] - minimo
    uav1_target_position[0:,0] = uav1_target_position[0:,0] - minimo
    uav1_target_position_fuse[0:,0] = uav1_target_position_fuse[0:,0] - minimo

    t = uav1_target_position_fuse[0:,0]

    #w = 2.5

    #r = 1.5 * w * t
            
    target_x = 0 * t



    tgt_pos_resample_x = signal.resample(target_position[0:,1], maximo)

    tgt_pos_resample_y = signal.resample(target_position[0:,2], maximo)
    
    dtc = np.zeros(maximo)
    
    
    error_x = (tgt_pos_resample_x - uav1_target_position_fuse[0:,1]) 

    error_y = (tgt_pos_resample_y - uav1_target_position_fuse[0:,2]) 

    
    error_x_mse = error_x * error_x
    
    error_y_mse= error_y * error_y
    
    error_x_rmse = np.sqrt(error_x_mse)
    
    error_y_rmse = np.sqrt(error_y_mse)
    
    euclid = np.sqrt(error_x_mse * error_x_mse + error_y_mse * error_y_mse)
    
    

    print("Erro em x: ")
    print(np.mean(error_x))

    print("Erro em y: ")
    print(np.mean(error_y))
    
    print("RSME distancia euclidiana no UAV: ")
    
    print(math.sqrt(sklearn.metrics.mean_squared_error(euclid, dtc)))
    
    
    
    




    plt.subplot(2, 1, 1)



    # UAV1 target position

    plt.plot(dataframes['/uav1/target_position_fuse'].Time, error_x, 'r+')


    plt.title("Estimativa")
    plt.xlabel('Tempo (s)')
    plt.ylabel('X (m)')
    plt.grid()

    plt.subplot(2, 1, 2)




    # UAV1 target position

    plt.plot(dataframes['/uav1/target_position_fuse'].Time, error_y, 'r+')


    plt.xlabel('Tempo (s)')
    plt.ylabel('Y (m)')
    plt.grid()

    im_basename = "single_x_y.png"
    im_fn = os.path.join(folder, im_basename) if folder else im_basename
    plt.savefig(im_fn)



    plt.figure(figsize=(18, 8))


    plt.plot(dataframes['/target_position'].x, dataframes['/target_position'].y, 'k', label="Alvo")
    plt.plot(dataframes['/uav1/target_position'].x, dataframes['/uav1/target_position'].y, '+', label="Alvo do UAV 1")

    plt.plot(dataframes['/uav1/target_position_fuse'].x, dataframes['/uav1/target_position_fuse'].y, 'r+', label="Estimativa")


    plt.title("Posição")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid()
    
    im_basename = "single_position.png"
    im_fn = os.path.join(folder, im_basename) if folder else im_basename
    plt.savefig(im_fn)





    plt.figure(figsize=(18, 8))



    #plt.plot(t, target_x, 'k', label="Alvo")


    # UAV1 target position

    plt.plot(dataframes['/uav1/target_position_fuse'].Time, euclid, 'g')


    plt.suptitle("Distancia euclidiana")
    plt.xlabel('Tempo (s)')
    plt.ylabel('D (m)')
    plt.legend()
    plt.grid()

    im_basename = "teste.png"
    im_fn = os.path.join(folder, im_basename) if folder else im_basename
    plt.savefig(im_fn)


