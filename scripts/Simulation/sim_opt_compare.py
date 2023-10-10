import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics
import pandas as pd
import os.path
from pathlib import Path

def compare_plots(dir, dynamics_name, data, delay = None):

    plt.rcParams['axes.prop_cycle'] = plt.cycler('color', plt.cm.tab10.colors)

    plt.figure(figsize=(9, 5))
    for i in data:
        if (i[3] == dynamics_name):
            plt.plot(i[1][0][0, :], i[0], label=i[2])
    plt.xlabel('Tempo (s)', fontsize=15)
    plt.ylabel('Erro (m)', fontsize=15)
    plt.legend(fontsize=10)
    plt.grid()


    plot_jpg = f'{dynamics_name}_{delay}' +  '.png'
    
    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()

    indices_by_name = {}

    for target_name in  ['stoped_path', 'linear_path', 'sin_path', 'circular_path']:
        indices = [index for index, inner_list in enumerate(data) if inner_list[3] == target_name]
        indices_by_name[target_name] = indices


    label_st = [['X', 'X (m)'], ['Y', 'Y (m)'], ['Vx', 'Vx (m/s)'], ['Vy', 'Vy (m/s)'], ['W', 'W (rad/s)']]
    for x, y in indices_by_name.items():
        for st in range(4):
            plt.figure(figsize=(9, 5))
            for index in y:
                plt.plot(data[index][1][0][0, :], data[index][1][0][st+1, :], label=data[index][2])
            plt.plot(data[index][5], data[index][4][st], 'k', label=label_st[st][0])
            plt.xlabel('Tempo (s)', fontsize=15)
            plt.ylabel(label_st[st][1], fontsize=15)
            plt.legend(fontsize=10)
            plt.grid()
            plot_jpg = f'Estado{label_st[st][0]}_{x}_{delay}' +  '.png'
        
            plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
            plt.savefig(plot_jpg)

            plt.close()


    # plt.figure(figsize=(9, 5))
    # for st in range(4):
    #     for dynamic in ['stoped_path', 'linear_path', 'sin_path', 'circular_path']:
    #         if (i[3] == dynamics_name):
    #             plt.figure(figsize=(9, 5))
    #         label_st = [['X', 'X (m)'], ['Y', 'Y (m)'], ['Vx', 'Vx (m/s)'], ['Vy', 'Vy (m/s)'], ['W', 'W (rad/s)']]
    #         plt.plot(i[1][0][0, :], i[0], label=i[2])
    #         plt.plot(i[5], i[4][st], 'k', label=label_st[st][0])
    #         plt.xlabel('Tempo (s)', fontsize=15)
    #         plt.ylabel(label_st[st][1], fontsize=15)
    #         plt.legend(fontsize=10)
    #         plt.grid()
    #         plot_jpg = f'Estado{label_st[st][0]}_{dynamics_name}_{delay}' +  '.png'
            
    #         plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    #         plt.savefig(plot_jpg)

    #         plt.close()



    