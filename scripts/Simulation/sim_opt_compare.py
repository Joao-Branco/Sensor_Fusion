import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics
import pandas as pd
import os.path
from pathlib import Path
import matplotlib as mpl

def compare_plots(dir, dynamics_name, data, delay = None):

    plt.rcParams['axes.prop_cycle'] = plt.cycler('color', plt.cm.tab10.colors)

    indices_by_name = {}

    for target_name in  ['stoped_path', 'linear_path', 'sin_path', 'circular_path']:
        indices = [index for index, inner_list in enumerate(data) if inner_list[3] == target_name]
        indices_by_name[target_name] = indices


    mpl.rcParams['text.usetex'] = True
    label_st = [['X', "$\dot{x}$ (m/s)"], ['Y', "$\dot{y}$ (m/s)"], ['W', "$\dot{\psi}$ (rad/s)"]]
    for x, y in indices_by_name.items():
        fig, axs = plt.subplots(3, figsize=(12, 8))
        for st in range(3):
            tab10_index = 0 
            for i, index in enumerate(y):
                if not (data[index][2] == "Estimador Linear" and st == 2):
                    color = plt.cm.tab10(tab10_index)
                    axs[st].plot(data[index][1][0][0, :], data[index][1][0][st+3, :], label=data[index][2], color=color)
                    tab10_index += 1
                else:
                    # Skip one color in Tab10 colormap
                    tab10_index = (tab10_index + 1) % 10  # Skip the next color
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




    list_performance = []
    fig, axs = plt.subplots(2, figsize=(12, 8))
    for i in data:
        obs = np.transpose(np.array(i[6][0]))
        interpolated_predicted_values_x = np.interp(obs[0,:], i[1][0][0, :], i[1][0][1,:])
        interpolated_predicted_values_y = np.interp(obs[0,:], i[1][0][0, :], i[1][0][2,:])

        position_residual = np.sqrt((obs[1,:] - interpolated_predicted_values_x) ** 2 + (obs[2,:] - interpolated_predicted_values_y) ** 2)
    
        # Calculate the residuals
        if (i[3] == dynamics_name):
            axs[0].plot(obs[0,:], position_residual, label=i[2])
            axs[1].plot(i[1][0][0, :], i[0], label=i[2])
            list_performance.append([i[2], np.mean(position_residual), np.std(position_residual), np.max(position_residual), np.min(position_residual)])
            
    axs[1].set_xlabel('Tempo (s)', fontsize=20, fontweight='bold')
    axs[1].set_ylabel('Erro de Posição (m)', fontsize=20, fontweight='bold')
    axs[1].grid()
    axs[0].set_ylabel('Valor Residual de Posição (m)', fontsize=20, fontweight='bold')
    axs[0].grid()
    handles, labels = axs[0].get_legend_handles_labels()

    fig.legend(handles, labels, loc='upper right', fontsize=20)

    plot_jpg = f'{dynamics_name}_{delay}_errors_' +   '.png'

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()
    performance = pd.DataFrame(list_performance, columns= ['Estimador', 'Mean', 'Std', 'Max', 'Min'])
    csv_residual = f'{dynamics_name}_{delay}_residual_.xlsx'
    csv_residual = os.path.join(dir, csv_residual) if dir else csv_residual
    performance.to_excel(csv_residual)







    