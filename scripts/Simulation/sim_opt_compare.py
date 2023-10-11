import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics
import pandas as pd
import os.path
from pathlib import Path
import matplotlib as mpl

def compare_plots(dir, dynamics_name, data, delay = None):

    plt.rcParams['axes.prop_cycle'] = plt.cycler('color', plt.cm.tab10.colors)

    plt.figure(figsize=(9, 5))
    for i in data:
        if (i[3] == dynamics_name):
            plt.plot(i[1][0][0, :], i[0], label=i[2])
    plt.xlabel('Tempo (s)', fontsize=20, fontweight='bold')
    plt.ylabel('Erro de posição (m)', fontsize=20, fontweight='bold')
    plt.legend(fontsize=15)
    plt.grid()


    plot_jpg = f'{dynamics_name}_{delay}' +  '.png'
    
    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()

    indices_by_name = {}

    for target_name in  ['stoped_path', 'linear_path', 'sin_path', 'circular_path']:
        indices = [index for index, inner_list in enumerate(data) if inner_list[3] == target_name]
        indices_by_name[target_name] = indices

    mpl.rcParams['text.usetex'] = True
    tab10_colors = mpl.cm.tab10.colors
    label_st = [['X', "$\dot{x}$ (m/s)"], ['Y', "$\dot{y}$ (m/s)"], ['W', "$\dot{\psi}$ (rad/s)"]]
    for x, y in indices_by_name.items():
        fig, axs = plt.subplots(3, figsize=(12, 8))
        for st in range(3):
            for i, index in enumerate(y):
                if not (data[index][2] == "Estimador Linear" and st == 2):
                    color = tab10_colors[i % len(tab10_colors)]  # Cycle through "tab20" colors
                    axs[st].plot(data[index][1][0][0, :], data[index][1][0][st+3, :], label=data[index][2], color=color)
            axs[st].plot(data[index][5], data[index][4][st+2], 'k:', label='Alvo')
            axs[st].set_ylabel(label_st[st][1], fontsize=25, fontweight='bold')
            #axs[st].set_title(label_st[st][0], fontsize=25)
            axs[st].grid()
        #plot_jpg = f'Estado{label_st[st][0]}_{x}_{delay}' +  '.png'
        plot_jpg = f'{x}_{delay}_states' +  '.png'
        axs[2].set_xlabel('Tempo (s)', fontsize=25 , fontweight='bold')
        handles, labels = axs[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right', fontsize=20)
    
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



    