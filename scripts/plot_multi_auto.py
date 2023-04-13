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

#bag_fn = r'/home/branco/catkin_ws/src/sensor_fusion/bags/multi_2023-04-10-15-23-07.bag'

def run_multi_plots(bag_fn, folder=None, uav_total):

    topics = ['/target_position']
    
    for i in range(uav_total + 1):
        if (i != 0):
            topics.append('/uav' + i + 'target_position')
            topics.append('/uav' + i + 'target_position_fuse')

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
    
    # Init fig
    plt.figure(figsize=(18, 8))

    # PLOT 1 - X axis

    # Truth target position

        
    target_position = dataframes['/target_position'].to_numpy()
    uav1_target_position = dataframes['/uav1/target_position'].to_numpy()
    uav1_target_position_fuse = dataframes['/uav1/target_position_fuse'].to_numpy()
    uav2_target_position = dataframes['/uav2/target_position'].to_numpy()
    uav2_target_position_fuse = dataframes['/uav2/target_position_fuse'].to_numpy()

    minimo = min(target_position[0,0], uav1_target_position[0,0], uav1_target_position_fuse[0,0], uav2_target_position[0,0], uav2_target_position_fuse[0,0])
    maximo_UAV1 = max(target_position[0:,0].size, uav1_target_position[0:,0].size, uav1_target_position_fuse[0:,0].size)
    maximo_UAV2 = max(target_position[0:,0].size, uav2_target_position[0:,0].size, uav2_target_position_fuse[0:,0].size)
    
    
    target_position[0:,0] = target_position[0:,0] - minimo

    uav1_target_position[0:,0] = uav1_target_position[0:,0] - minimo
    uav1_target_position_fuse[0:,0] = uav1_target_position_fuse[0:,0] - minimo

    uav2_target_position[0:,0] = uav2_target_position[0:,0] - minimo
    uav2_target_position_fuse[0:,0] = uav2_target_position_fuse[0:,0] - minimo



    tgt_pos_resample_x_UAV1 = signal.resample(target_position[0:,1], maximo_UAV1)
    
    tgt_pos_resample_y_UAV1 = signal.resample(target_position[0:,2], maximo_UAV1)

    tgt_pos_resample_x_UAV2 = signal.resample(target_position[0:,1], maximo_UAV2)
    
    tgt_pos_resample_y_UAV2 = signal.resample(target_position[0:,2], maximo_UAV2)

    dtc_UAV1 = np.zeros(maximo_UAV1)
    
    dtc_UAV2 = np.zeros(maximo_UAV2)
    
    
    error_x_UAV1 = (tgt_pos_resample_x_UAV1 - uav1_target_position_fuse[0:,1]) 

    error_y_UAV1 = (tgt_pos_resample_y_UAV1 - uav1_target_position_fuse[0:,2]) 
    
    error_x_UAV2 = (tgt_pos_resample_x_UAV2 - uav2_target_position_fuse[0:,1]) 

    error_y_UAV2 = (tgt_pos_resample_y_UAV2 - uav2_target_position_fuse[0:,2]) 

    
    error_x_mse_UAV1 = error_x_UAV1 * error_x_UAV1
    
    error_y_mse_UAV1= error_y_UAV1 * error_y_UAV1
    
    error_x_mse_UAV2 = error_x_UAV2 * error_x_UAV2
    
    error_y_mse_UAV2= error_y_UAV2 * error_y_UAV2
    
    euclid_UAV1 = np.sqrt(error_x_mse_UAV1 * error_x_mse_UAV1 + error_y_mse_UAV1 * error_y_mse_UAV1)
    
    euclid_UAV2 = np.sqrt(error_x_mse_UAV2 * error_x_mse_UAV2 + error_y_mse_UAV2 * error_y_mse_UAV2)


    print("Erro em x UAV 1: ")
    print(np.mean(error_x_UAV1))
    print("Erro em y UAV 1: ")
    print(np.mean(error_y_UAV1))
    print("RSME distancia euclidiana no UAV 1: ")
    print(math.sqrt(sklearn.metrics.mean_squared_error(euclid_UAV1, dtc_UAV1)))
    print("Erro em x UAV 2: ")
    print(np.mean(error_x_UAV2))
    print("Erro em y UAV 2: ")
    print(np.mean(error_y_UAV2))
    print("RSME distancia euclidiana no UAV 2: ")
    print(math.sqrt(sklearn.metrics.mean_squared_error(euclid_UAV2, dtc_UAV2)))


    


    plt.subplot(2, 1, 1)



    # UAV1 target position

    plt.plot(dataframes['/uav1/target_position_fuse'].Time, error_x_UAV1, 'r+', label="UAV 1")
    plt.plot(dataframes['/uav2/target_position_fuse'].Time, error_x_UAV2, 'bx', label="UAV 2")


    plt.title("Estimativa")
    plt.xlabel('Tempo (s)')
    plt.ylabel('X (m)')
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)




    # UAV1 target position

    plt.plot(dataframes['/uav1/target_position_fuse'].Time, error_y_UAV1, 'r+', label="UAV 1")
    plt.plot(dataframes['/uav2/target_position_fuse'].Time, error_y_UAV2, 'bx', label="UAV 2")


    plt.xlabel('Tempo (s)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid()



    im_basename = "multi_x_y.png"
    im_fn = os.path.join(folder, im_basename) if folder else im_basename
    plt.savefig(im_fn)
    

    plt.figure(figsize=(18, 8))


    plt.plot(dataframes['/target_position'].x, dataframes['/target_position'].y, 'k', label="Alvo")
    plt.plot(dataframes['/uav1/target_position'].x, dataframes['/uav1/target_position'].y, 'g+', label="Alvo com ruido UAV 1")
    plt.plot(dataframes['/uav2/target_position'].x, dataframes['/uav2/target_position'].y, 'x', label="Alvo com ruido UAV 2")

    plt.plot(dataframes['/uav1/target_position_fuse'].x, dataframes['/uav1/target_position_fuse'].y, 'r+', label="Estimativa UAV 1")
    plt.plot(dataframes['/uav2/target_position_fuse'].x, dataframes['/uav2/target_position_fuse'].y, 'bx', label="Estimativa UAV 2")
    


    plt.title("Posição")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid()


    im_basename = "multi_position.png"
    im_fn = os.path.join(folder, im_basename) if folder else im_basename
    plt.savefig(im_fn)