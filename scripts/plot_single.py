import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
from functools import reduce

bag_fn = r'/home/branco/catkin_ws/src/sensor_fusion/bags/single_2023-04-05-16-09-33.bag'


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

minimo = min(target_position[0,0], uav1_target_position[0,0], uav1_target_position_fuse[0,0])
target_position[0:,0] = target_position[0:,0] - minimo
uav1_target_position[0:,0] = uav1_target_position[0:,0] - minimo
uav1_target_position_fuse[0:,0] = uav1_target_position_fuse[0:,0] - minimo





plt.subplot(2, 1, 1)


plt.plot(dataframes['/target_position'].Time, dataframes['/target_position'].x, 'k', label="Alvo")


# UAV1 target position

plt.plot(dataframes['/uav1/target_position_fuse'].Time, dataframes['/uav1/target_position_fuse'].x, 'g', label="Estimativa")


plt.suptitle("Posição")
plt.xlabel('Tempo (s)')
plt.ylabel('X (m)')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)


plt.plot(dataframes['/target_position'].Time, dataframes['/target_position'].y, 'k', label="Alvo")


# UAV1 target position

plt.plot(dataframes['/uav1/target_position_fuse'].Time, dataframes['/uav1/target_position_fuse'].y, 'g', label="Estimativa")


plt.xlabel('Tempo (s)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid()



plt.savefig('/home/branco/catkin_ws/src/sensor_fusion/plots/single_x_y.png')


plt.figure(figsize=(18, 8))

plt.subplot(1, 2, 1)

plt.plot(dataframes['/target_position'].x, dataframes['/target_position'].y, 'k', label="Alvo")


plt.plot(dataframes['/uav1/target_position'].x, dataframes['/uav1/target_position'].y, 'g', label="Estimativa")

plt.title("Posição")
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid()

erro_x = abs(dataframes['/uav1/target_position'].x - dataframes['/target_position'].x) / dataframes['/target_position'].x * 100

erro_y = abs(dataframes['/uav1/target_position'].y - dataframes['/target_position'].y) / dataframes['/target_position'].y * 100

plt.plot(dataframes['/uav1/target_position_fuse'].Time, erro_x, 'r', label="X")

plt.plot(dataframes['/uav1/target_position_fuse'].Time, erro_y, 'b', label="Y")



plt.title("Erro relativo")
plt.xlabel('Tempo (s)')
plt.ylabel('%')
plt.legend()
plt.grid()

plt.savefig('/home/branco/catkin_ws/src/sensor_fusion/plots/single_position.png')
