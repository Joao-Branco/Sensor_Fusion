import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics
import pandas as pd
import os.path
from pathlib import Path
import matplotlib as mpl
import math
import matplotlib
matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})

def compare_plots(dir, dynamics_name, data, delay = None):

    plt.rcParams['axes.prop_cycle'] = plt.cycler('color', plt.cm.tab10.colors)

    indices_by_name = {}

    for target_name in  ['linear_path', 'sin_path']:
        indices = [index for index, inner_list in enumerate(data) if inner_list[3] == target_name]
        indices_by_name[target_name] = indices


    mpl.rcParams['text.usetex'] = True
    label_st = [['X', "$\dot{x}$ (m/s)"], ['Y', "$\dot{y}$ (m/s)"], ['W', "$\dot{\psi}$ (rad/s)"]]
    for x, y in indices_by_name.items():
        fig, axs = plt.subplots(3, figsize=(13, 9))
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
            axs[st].plot(data[index][5], data[index][4][st+2], 'k:', linewidth='4', label='Alvo')
            axs[st].set_ylabel(label_st[st][1], fontsize=35, fontweight='bold')
            #axs[st].set_title(label_st[st][0], fontsize=25)
            axs[st].grid()
            axs[st].tick_params(axis='both', labelsize=20)
                           

        #plot_jpg = f'Estado{label_st[st][0]}_{x}_{delay}' +  '.png'
        plot_jpg = f'{x}_{delay}_states' +  '.pgf'
        axs[2].set_xlabel('Tempo (s)', fontsize=30 , fontweight='bold')
        handles, labels = axs[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right', fontsize=25)
    
        plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
        plt.savefig(plot_jpg)

        plt.close()




    list_performance = []
    fig, axs = plt.subplots(2, figsize=(13, 9))
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
            
    axs[1].set_xlabel('Tempo (s)', fontsize=30, fontweight='bold')
    axs[1].set_ylabel('Erro de Posição (m)', fontsize=20, fontweight='bold')
    axs[1].grid()
    axs[0].set_ylabel('Valor Residual de Posição (m)', fontsize=20, fontweight='bold')
    axs[0].grid()
    axs[0].tick_params(axis='both', labelsize=20)
    axs[1].tick_params(axis='both', labelsize=20)
    
    handles, labels = axs[0].get_legend_handles_labels()
    

    fig.legend(handles, labels, loc='upper right', fontsize=25)

    plot_jpg = f'{dynamics_name}_{delay}_errors_' +   '.pgf'

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)
    plot_jpg = f'{dynamics_name}_{delay}_errors_' +   '.png'

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()
    performance = pd.DataFrame(list_performance, columns= ['Estimador', 'Mean', 'Std', 'Max', 'Min'])
    csv_residual = f'{dynamics_name}_{delay}_residual_.xlsx'
    csv_residual = os.path.join(dir, csv_residual) if dir else csv_residual
    performance.to_excel(csv_residual)


def compare_plots_multi(dir, dynamics_name, data, delay = None):

    plt.rcParams['axes.prop_cycle'] = plt.cycler('color', plt.cm.tab10.colors)
    label_st = [['X', "$\dot{x}$ (m/s)"], ['Y', "$\dot{y}$ (m/s)"], ['W', "$\dot{\psi}$ (rad/s)"]]

    fig, axs = plt.subplots(3, figsize=(13, 9))
    for st in range(3):
        tab10_index = 0 
        for i in data:
            color = plt.cm.tab10(tab10_index)
            estimative_mean = []
            for j in i[1]:
                estimative_mean.append(j[st+3, :])
            axs[st].plot(i[1][0][0, :], np.mean(estimative_mean, axis=0), label=i[2], color=color)
            tab10_index += 1
        axs[st].plot(i[5], i[4][st+2], 'k:', linewidth='4', label='Alvo')
        axs[st].set_ylabel(label_st[st][1], fontsize=35, fontweight='bold')
        #axs[st].set_title(label_st[st][0], fontsize=25)
        axs[st].grid()
        axs[st].tick_params(axis='both', labelsize=20)


    #plot_jpg = f'Estado{label_st[st][0]}_{x}_{delay}' +  '.png'
    plot_jpg = f'{dynamics_name}_{delay}_states' +  '.pgf'
    axs[2].set_xlabel('Tempo (s)', fontsize=30 , fontweight='bold')
    handles, labels = axs[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', fontsize=25)

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()


    mpl.rcParams['text.usetex'] = True
    mpl.rcParams['text.latex.preamble'] = r'\usepackage{lmodern}'




    list_performance = []
    fig, axs = plt.subplots(3, figsize=(13, 9))
    for i in data:
        position_residual = []
        for j, obs in enumerate(i[6]):
            obs = np.transpose(np.array(obs))
            interpolated_predicted_values_x = np.interp(obs[0,:], i[1][j][0, :], i[1][j][1,:])
            interpolated_predicted_values_y = np.interp(obs[0,:], i[1][j][0, :], i[1][j][2,:])

            position_residual.append(np.sqrt((obs[1,:] - interpolated_predicted_values_x) ** 2 + (obs[2,:] - interpolated_predicted_values_y) ** 2))
        position_residual = np.array(position_residual)
        mean_residual = np.mean(position_residual, axis=0)

        # Calculate the residuals
        if (i[3] == dynamics_name):
            axs[0].plot(obs[0,:], mean_residual, label=i[2])
            axs[1].plot(i[1][0][0, :], i[0], label=i[2])
            axs[2].plot(i[1][0][0, :], i[7], label=i[2])
            list_performance.append([i[2], np.mean(mean_residual), np.std(mean_residual), np.max(mean_residual), np.min(mean_residual), np.mean(i[7]), np.std(i[7]), np.max(i[7]), np.min(i[7])])

    axs[2].set_xlabel('Tempo (s)', fontsize=30, fontweight='bold')
    axs[2].set_ylabel('DMP (m)', fontsize=30, fontweight='bold')
    axs[2].grid()       
    axs[1].set_ylabel('EPM (m)', fontsize=30, fontweight='bold')
    axs[1].grid()
    axs[0].set_ylabel('VRMP (m)', fontsize=30, fontweight='bold')
    axs[0].grid()
    axs[0].tick_params(axis='both', labelsize=20)
    axs[1].tick_params(axis='both', labelsize=20)
    axs[2].tick_params(axis='both', labelsize=20)
    
    
    handles, labels = axs[0].get_legend_handles_labels()

    fig.legend(handles, labels, loc='upper right', fontsize=25)

    plot_jpg = f'{dynamics_name}_{delay}_errors_' +   '.pgf'

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plot_jpg = f'{dynamics_name}_{delay}_errors_' +   '.png'

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()
    performance = pd.DataFrame(list_performance, columns= ['Estimador', 'Mean_res', 'Std_res', 'Max_res', 'Min_res', 'Mean_des', 'Std_des', 'Max_des', 'Min_des'])
    csv_residual = f'{dynamics_name}_{delay}_residual_.xlsx'
    csv_residual = os.path.join(dir, csv_residual) if dir else csv_residual
    performance.to_excel(csv_residual)



def compare_plots_multi_delay(dir, dynamics_name, data, delay = None):

    plt.rcParams['axes.prop_cycle'] = plt.cycler('color', plt.cm.tab10.colors)
    label_st = [['X', "$\dot{x}$ (m/s)"], ['Y', "$\dot{y}$ (m/s)"], ['W', "$\dot{\psi}$ (rad/s)"]]

    fig, axs = plt.subplots(3, figsize=(13, 9))
    for st in range(3):
        tab10_index = 0 
        for i in data:
            color = plt.cm.tab10(tab10_index)
            estimative_mean = []
            for j in i[1]:
                estimative_mean.append(j[st+3, :])
            axs[st].plot(i[1][0][0, :], np.mean(estimative_mean, axis=0), label=i[2], color=color)
            tab10_index += 1
            axs[st].plot(i[5], i[4][st+2], 'k:', label='Alvo')
        axs[st].set_ylabel(label_st[st][1], fontsize=30, fontweight='bold')
        #axs[st].set_title(label_st[st][0], fontsize=25)
        axs[st].grid()
        axs[st].tick_params(axis='both', labelsize=20)

    #plot_jpg = f'Estado{label_st[st][0]}_{x}_{delay}' +  '.png'
    plot_jpg = f'{dynamics_name}_{delay}_states' +  '.pgf'
    axs[2].set_xlabel('Tempo (s)', fontsize=30 , fontweight='bold')
    handles, labels = axs[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', fontsize=25)

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()


    mpl.rcParams['text.usetex'] = True





    for i in data:
        plt.figure(figsize=(9, 7))
        n_bins = 40
        n, bins, patches = plt.hist(i[8], bins= n_bins, density=True, label= ['UAV 1', 'UAV 2', 'UAV 3'])
        mu = i[9][0]
        sigma = i[9][1]
        y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
        np.exp(-0.5 * (1 / sigma * (bins - mu))**2))
        plt.plot(bins, y, 'k--')
        if (i[12] == "augmented_state"):
            plt.axvline(i[10], color='red', label = 'Valor Limite')
        plt.xlabel('Atrasos (s)', fontsize=25)
        plt.ylabel('Densidade de probabilidade', fontsize=20)
        plt.legend(fontsize=15)
        plt.grid()
        plt.tick_params(axis='both', labelsize=20)

        plot_jpg = f'{dynamics_name}_{delay}_{i[2]}_densidade_' +   '.pgf'

        plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
       #plt.savefig(plot_jpg)

        plot_jpg = f'{dynamics_name}_{delay}_{i[2]}_densidade_' +   '.png'

        plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
        #plt.savefig(plot_jpg)

        plt.close()


        if (i[14] == "augmented_state"):

            aug = i[10]
            counter_list = []
            for uav_dela , dela in enumerate(i[8]):
                counter_list.append([f'UAV {uav_dela}', 0, 0])
                for d in dela:
                    if d > aug:
                        counter_list[uav_dela][1] += 1 
                    
                    counter_list[uav_dela][2] += 1 
            
            

            performance = pd.DataFrame(counter_list, columns= ['UAV' ,'Elementos desprezados', 'Total de elementos'])
            csv_residual = f'{dynamics_name}_{delay}_{i[2]}_aug_desprezados_.xlsx'
            csv_residual = os.path.join(dir, csv_residual) if dir else csv_residual
            #performance.to_excel(csv_residual)

        if (i[14] == "extrapolate"):

            state = i[4]
            counter_corr = []
            uav_corr = 0
            for z_obs, z_corr, z_masks in zip(i[11], i[12], i[13]):

                counter_corr.append([f'UAV {uav_corr}', 0, z_masks.shape[0]])
                for enu, tt in enumerate(z_masks):
                    d_obs = np.sqrt((state[0][tt] - z_obs[0][enu]) ** 2 + (state[1][tt] - z_obs[1][enu]) ** 2 )
                    d_corr = np.sqrt((state[0][tt] - z_corr[0][enu]) ** 2 + (state[1][tt] - z_corr[1][enu]) ** 2)

                    if(d_corr < d_obs):
                        counter_corr[uav_corr][1] += 1 
                uav_corr += 1

            performance = pd.DataFrame(counter_corr, columns= ['UAV' ,'Elementos corrigidos', 'total'])
            csv_residual = f'{dynamics_name}_{delay}_{i[2]}_extra_corrigidos_.xlsx'
            csv_residual = os.path.join(dir, csv_residual) if dir else csv_residual
            #performance.to_excel(csv_residual)
            
                

    list_performance = []
    fig, axs = plt.subplots(2, figsize=(13, 9))  
    for i in data:
        if i[3] == dynamics_name:
            axs[0].plot(i[1][0][0, :], i[0], label=i[2])
            axs[1].plot(i[1][0][0, :], i[7], label=i[2]) 
            list_performance.append([i[2], np.mean(i[7]), np.std(i[7]), np.max(i[7]), np.min(i[7])])

    axs[1].set_xlabel('Tempo (s)', fontsize=30, fontweight='bold')
    axs[1].set_ylabel('DMP (m)', fontsize=30, fontweight='bold')
    axs[1].grid()
    axs[0].set_ylabel('EPM (m)', fontsize=30, fontweight='bold')
    axs[0].grid()
    axs[0].tick_params(axis='both', labelsize=20)
    axs[1].tick_params(axis='both', labelsize=20)

    handles, labels = axs[0].get_legend_handles_labels()

    fig.legend(handles, labels, loc='upper right', fontsize=25)

    plot_jpg = f'{dynamics_name}_{delay}_errors_' + '.pgf'

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)
    plot_jpg = f'{dynamics_name}_{delay}_errors_' + '.png'

    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()
    performance = pd.DataFrame(list_performance, columns=['Estimador', 'Mean_des', 'Std_des', 'Max_des', 'Min_des'])
    csv_residual = f'{dynamics_name}_{delay}_performance.xlsx'
    csv_residual = os.path.join(dir, csv_residual) if dir else csv_residual
    performance.to_excel(csv_residual)

    




    