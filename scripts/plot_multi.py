import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
from functools import reduce

bag_fn = r'/home/branco/catkin_ws/src/sensor_fusion/bags/multi_2023-04-05-15-40-19.bag'


topics = [
    '/target_position',
    '/uav1/target_position',
    '/uav1/target_position_fuse',
    '/uav2/target_position',
    '/uav2/target_position_fuse'
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

dataframes['/uav2/target_position'].rename(columns={
    'target_position.x': 'x',
    'target_position.y': 'y'
}, inplace=True)

dataframes['/uav2/target_position_fuse'].rename(columns={
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
uav2_target_position = dataframes['/uav2/target_position'].to_numpy()
uav2_target_position_fuse = dataframes['/uav2/target_position_fuse'].to_numpy()

minimo = min(target_position[0,0], uav1_target_position[0,0], uav1_target_position_fuse[0,0], uav2_target_position[0,0], uav2_target_position_fuse[0,0])

target_position[0:,0] = target_position[0:,0] - minimo

uav1_target_position[0:,0] = uav1_target_position[0:,0] - minimo
uav1_target_position_fuse[0:,0] = uav1_target_position_fuse[0:,0] - minimo

uav2_target_position[0:,0] = uav2_target_position[0:,0] - minimo
uav2_target_position_fuse[0:,0] = uav2_target_position_fuse[0:,0] - minimo



plt.subplot(2, 1, 1)


plt.plot(dataframes['/target_position'].Time, dataframes['/target_position'].x, 'k', label="Alvo")


# UAV1 target position


plt.plot(dataframes['/uav1/target_position_fuse'].Time, dataframes['/uav1/target_position_fuse'].x, 'g', label="Estimativa 1")


plt.plot(dataframes['/uav2/target_position_fuse'].Time, dataframes['/uav2/target_position_fuse'].x, 'm', label="Estimativa 2")


plt.suptitle("Posição")
plt.xlabel('Tempo (s)')
plt.ylabel('X (m)')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)


plt.plot(dataframes['/target_position'].Time, dataframes['/target_position'].y, 'k', label="Alvo")


# UAV1 target position

plt.plot(dataframes['/uav1/target_position_fuse'].Time, dataframes['/uav1/target_position_fuse'].y, 'g', label="Estimativa 1")

plt.plot(dataframes['/uav2/target_position_fuse'].Time, dataframes['/uav2/target_position_fuse'].y, 'm', label="Estimativa 2")


plt.xlabel('Tempo (s)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid()



plt.savefig('/home/branco/catkin_ws/src/sensor_fusion/plots/multi_x_y.png')


plt.figure(figsize=(8, 8))

plt.plot(dataframes['/target_position'].x, dataframes['/target_position'].y, 'k', label="Alvo")

plt.plot(dataframes['/uav1/target_position'].x, dataframes['/uav1/target_position'].y, 'g', label="Estimativa 1")

plt.plot(dataframes['/uav2/target_position'].x, dataframes['/uav2/target_position'].y, 'm', label="Estimativa 2")

plt.title("Posição")
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid()

plt.savefig('/home/branco/catkin_ws/src/sensor_fusion/plots/multi_position.png')
