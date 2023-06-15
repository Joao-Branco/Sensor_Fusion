import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib
matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})
import matplotlib.pyplot as plt
import numpy as np
from functools import reduce
from scipy import signal
import os.path
import sklearn.metrics
from matplotlib.animation import FuncAnimation, PillowWriter




def run_auto_plots(bag_fn, uav_total, single, delay, delay_estimation, fuse, folder_png=None, folder_pgf=None, folder_sim=None):
    if single == True:
        uav_total = 1
        
        
    
    
    topics = ['/target_position']
    


    for i in range(1, uav_total + 1):

        topics.append('/uav' + str(i) + '/target_position') # Noise
        if (fuse == True):
            topics.append('/uav' + str(i) + '/target_position_fuse') # After throotle or after buffer
        topics.append('/uav' + str(i) + '/target_position_estimation') # Kalman filter
        if (delay == True):
            topics.append('/uav' + str(i) + '/delay')
        if (delay_estimation == True):
            topics.append('/uav' + str(i) + '/delay_estimation')
            topics.append('/uav' + str(i) + '/msg_true')
            topics.append('/uav' + str(i) + '/msg_correction')



    # Give filename of rosbag
    b = bagreader(bag_fn)

    # load messages to dataframes
    dataframes = {}
    minimos = []

    for topic in topics:
        data = b.message_by_topic(topic)
        print(data)
        df = pd.read_csv(data)
        df['Timestamp'] = df['timestamp.secs'] + df['timestamp.nsecs'] * 1e-9
        df.drop(['timestamp.secs','timestamp.nsecs'], axis='columns', inplace=True)
        dataframes[topic] = df
        minimos.append(dataframes[topic].iloc[0,0])

    minimo = min(minimos)   
    dataframes['/target_position'].iloc[:,0] = dataframes['/target_position'].iloc[:,0] - minimo

    for i in range(1, uav_total+1):
        dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position'].iloc[:,0] - minimo 
        if (fuse == True):
            dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_fuse'].iloc[:,0] - minimo
        dataframes[f'/uav{str(i)}/target_position_estimation'].iloc[:,0] = dataframes[f'/uav{str(i)}/target_position_estimation'].iloc[:,0] - minimo
        if (delay == True):
            dataframes[f'/uav{str(i)}/delay'].iloc[:,0] = dataframes[f'/uav{str(i)}/delay'].iloc[:,0] - minimo
        if (delay_estimation == True):
            dataframes[f'/uav{str(i)}/delay_estimation'].iloc[:,0] = dataframes[f'/uav{str(i)}/delay_estimation'].iloc[:,0] - minimo
            dataframes[f'/uav{str(i)}/msg_correction'].iloc[:,0] = dataframes[f'/uav{str(i)}/msg_correction'].iloc[:,0] - minimo
            dataframes[f'/uav{str(i)}/msg_true'].iloc[:,0] = dataframes[f'/uav{str(i)}/msg_true'].iloc[:,0] - minimo
            



    target = dataframes['/target_position']
    target.columns = ['Time', 'x_target', 'y_target', 'v_x_target', 'v_y_target', 'Timestamp_target']
    #target.Time = pd.to_timedelta( target.Time, "sec")
    target_index = target.set_index('Time')
    data = {}
    data_error = {}
    precision = pd.DataFrame()
    precision_counter = 0
    data_all = pd.DataFrame()




    for i in range(1, uav_total+1):

        data[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/target_position_estimation']
        #data[f'uav{str(i)}'].drop(data[f'uav{str(i)}'].tail(5).index, inplace = True)
        #data[f'uav{str(i)}'].drop(data[f'uav{str(i)}'].index[:5], inplace = True)
        #data_noise[f'uav{str(i)}'] = dataframes[f'/uav{str(i)}/target_position']
        #data_noise[f'uav{str(i)}'].columns = ['Time', 'x_noise', 'y_noise']
        #data_noise_index = data_noise[f'uav{str(i)}'].set_index('Time')
        
        #data[f'uav{str(i)}']["Time"] = pd.to_timedelta( data[f'uav{str(i)}']["Time"], "sec")
        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].set_index('Time')
        real_index = data[f'uav{str(i)}'].index
        data_all = data_all.join(data[f'uav{str(i)}'][['x', 'y']].add_suffix(f'_{str(i)}'), how='outer')
        data[f'uav{str(i)}'] = data[f'uav{str(i)}'].join(target_index, how='outer')

        data_error[f'uav{str(i)}'] = data[f'uav{str(i)}'].interpolate(limit_direction ='both').loc[real_index]

        error_x = abs(data_error[f'uav{str(i)}'].x_target - data_error[f'uav{str(i)}'].x)
        error_y = abs(data_error[f'uav{str(i)}'].y_target - data_error[f'uav{str(i)}'].y)
        
        euclidean = np.sqrt(error_x * error_x + error_y * error_y)
        
        v_x = data_error[f'uav{str(i)}'].v * np.cos(data_error[f'uav{str(i)}'].theta)
        v_y = data_error[f'uav{str(i)}'].v * np.sin(data_error[f'uav{str(i)}'].theta)
        #v_x = data_error[f'uav{str(i)}'].v_x
        #v_y = data_error[f'uav{str(i)}'].v_y

        error_v_x = abs(data_error[f'uav{str(i)}'].v_x_target - v_x)
        error_v_y = abs(data_error[f'uav{str(i)}'].v_y_target - v_y)

        error_Timestamp = abs(data_error[f'uav{str(i)}'].Timestamp_target - data_error[f'uav{str(i)}'].Timestamp)


        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "v_x", v_x)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "v_y", v_y)

        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_x", error_x)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_y", error_y)
        
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "euclidean", euclidean)

        #data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "euclidean_precision", euclidean)
        
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_v_x", error_v_x)
        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_v_y", error_v_y)

        data_error[f'uav{str(i)}'].insert(len(data_error[f'uav{str(i)}'].columns), "error_Timestamp", error_Timestamp)
        
        data_error[f'uav{str(i)}'].reset_index(inplace=True)

    data_all = data_all.interpolate(limit_direction ='both')

    data_all.to_csv( folder_sim + '/data_all_interpolate.csv')



    for i in range(1, uav_total):
        x_i = data_all.columns.get_loc('x_' + str(i))
        y_i = data_all.columns.get_loc('y_' + str(i))
        for j in range(1, uav_total + 1):
            x_j = data_all.columns.get_loc('x_' + str(j))
            y_j = data_all.columns.get_loc('y_' + str(j))
            if(j > i):
                error_x = abs(data_all.iloc[:,x_i] - data_all.iloc[:,x_j])
                error_y = abs(data_all.iloc[:,y_i] - data_all.iloc[:,y_j])
                distance_uavs = np.sqrt(error_x * error_x + error_y * error_y)
                precision.insert(precision_counter, f'uav{str(i)}_uav{str(j)}', distance_uavs)
                precision_counter = precision_counter + 1


    precision.insert(precision_counter, 'mean', precision.mean(axis=1))

    precision.to_csv( folder_sim + '/precision.csv')






    #Graphs of errors in estimation of uavs in x and y
    
    plt.figure(figsize=(15, 5))

    plt.subplot(1, 2, 1)
    plt.title("Erro absoluto", fontsize=15)

    for i in range(1, uav_total+1):
        plt.plot(data_error[f'uav{str(i)}'].Time, data_error[f'uav{str(i)}'].error_x, 'x', label='UAV ' + str(i))

    plt.xlabel('Tempo (s)', fontsize=15)
    plt.ylabel('X (m)', fontsize=15)
    if (single == False):
        plt.legend(fontsize=10)
    plt.grid()

    plt.subplot(1, 2, 2)
    plt.title("Erro absoluto", fontsize=15)

    for i in range(1, uav_total+1):
        plt.plot(data_error[f'uav{str(i)}'].Time, data_error[f'uav{str(i)}'].error_y, 'x', label='UAV ' + str(i))

    plt.xlabel('Tempo (s)', fontsize=15)
    plt.ylabel('Y (m)', fontsize=15)
    if (single == False):
        plt.legend(fontsize=10)
    plt.grid()

    if (single == True):
        im_basename_png = "Errors_single.png"
        
        im_basename_pgf = "Errors_single.pgf"

    else:
        im_basename_png = "Errors_multi.png"
        
        im_basename_pgf = "Errors_multi.pgf"


    im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
    plt.savefig(im_fn_png)
    im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
    plt.savefig(im_fn_pgf)


    #Graphs of position target and estimation of uavs
    
    plt.figure(figsize=(6, 6))
    
    if (single == True):
        plt.plot(data_error['uav1'].x, data_error['uav1'].y, 'x', markersize=4, label='UAV 1')
        plt.plot(dataframes['/uav1/target_position'].x, dataframes['/uav1/target_position'].y, 'm.', markersize=2, label="Alvo com ruido")

            
        
    else:
        for i in range(1, uav_total+1):
            plt.plot(data_error[f'uav{str(i)}'].x, data_error[f'uav{str(i)}'].y, 'x', markersize=5, label='UAV ' + str(i))


    plt.plot(target.x_target, target.y_target, 'k',linewidth='3', label="Alvo")
    plt.plot(target.iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    plt.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    
    
    
    plt.title("Posição", fontsize=20)
    plt.xlabel('X (m)', fontsize=15)
    plt.ylabel('Y (m)', fontsize=15)
    plt.legend(fontsize=10)
    plt.grid()


    if (single == True):
        im_basename_png = "Position_single.png"
        im_basename_pgf = "Position_single.pgf"


    else:
        im_basename_png = "Position_multi.png"
        im_basename_pgf = "Position_multi.pgf"
        

    im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
    plt.savefig(im_fn_png)
    im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
    plt.savefig(im_fn_pgf)
    
    
    #GIFs of position of target and estimation of uavs
    
    
    
    # if (single == True):
    #     fig,ax = plt.subplots()
    #     def animate(i):
    #         ax.clear()
    #         line, = ax.plot(target.iloc[0:i,1], target.iloc[0:i,2], 'k',linewidth='3', label="Alvo")
    #         setpoints1, = ax.plot(data_error['uav1'].iloc[0:i,1], data_error['uav1'].iloc[0:i,2], 'x', markersize=4, label='UAV 1')
    #         setpoints2, = ax.plot(dataframes['/uav1/target_position'].iloc[0:i,1], dataframes['/uav1/target_position'].iloc[0:i,2], 'm.', markersize=2, label="Alvo com ruido")
    #         point1, = ax.plot(target.iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    #         point2, = ax.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    #         point3, = ax.plot(target.iloc[i,1], target.iloc[i,2], 'ko',  markersize=8)
    #         return line, setpoints1, setpoints2 , point1, point2, point3,
        

    #     im_basename_gif = "Position_single.gif"
    #     im_fn_gif = os.path.join(folder_png, im_basename_gif) if folder_png else im_basename_gif
    #     ani = FuncAnimation(fig, animate, interval=40, blit=True, repeat=True, frames = len(target.index))    
    #     ani.save(im_fn_gif, writer=PillowWriter(fps=20))
        
    # else:
    #     fig,ax = plt.subplots()
    #     def animate(i):
    #         ax.clear()
    #         setpoints = []
    #         line, = ax.plot(target.iloc[0:i,1], target.iloc[0:i,2], 'k',linewidth='3', label="Alvo")
    #         for j in range(1, uav_total+1):
    #             setpoints.append(ax.plot(data_error[f'uav{str(j)}'].iloc[0:i,1], data_error[f'uav{str(j)}'].iloc[0:i,2], 'x', markersize=5, label='UAV ' + str(i)))
    #         point1, = ax.plot(target.iloc[0,1], target.iloc[0,2], 'go', markersize=8)
    #         point2, = ax.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
    #         point3, = ax.plot(target.iloc[i,1], target.iloc[i,2], 'ko',  markersize=8)
    #         return line, setpoints, point1, point2, point3,
        
    #     im_basename_gif = "Position_multi.gif"
    #     im_fn_gif = os.path.join(folder_png, im_basename_gif) if folder_png else im_basename_gif
    #     ani = FuncAnimation(fig, animate, interval=40, blit=True, repeat=True, frames = len(target.index))    
    #     ani.save(im_fn_gif, writer=PillowWriter(fps=20))
        
    
    
    #Graphs of delays
    
    if (delay == True):
        for i in range(1, uav_total+1):
            plt.figure(figsize=(15, 5))
            plt.plot(dataframes[f'/uav{str(i)}/delay'].Time, dataframes[f'/uav{str(i)}/delay'].data, 'x', markersize=5, label='true value')
            if (delay_estimation == True):
                plt.plot(dataframes[f'/uav{str(i)}/delay_estimation'].Time, dataframes[f'/uav{str(i)}/delay_estimation'].data, 'x', markersize=5, label='estimation')
            plt.title('UAV ' + str(i), fontsize=20)
            plt.xlabel('t (s)', fontsize=15)
            plt.ylabel('Delay (s)', fontsize=15)
            plt.legend(fontsize=10)
            plt.grid()
            im_basename_png = 'Delay_UAV' + str(i) + '.png'
            im_basename_pgf = 'Delay_UAV' + str(i) + '.pgf'
            im_fn_png = os.path.join(folder_png, im_basename_png) if folder_png else im_basename_png
            plt.savefig(im_fn_png)
            im_fn_pgf = os.path.join(folder_pgf, im_basename_pgf) if folder_pgf else im_basename_pgf
            plt.savefig(im_fn_pgf)

            fig,ax = plt.subplots()
            def animate(j):
                ax.clear()
                setpoints = []
                line, = ax.plot(target.iloc[0:j,1], target.iloc[0:j,2], 'k',linewidth='3', label="Alvo")
                point1, = ax.plot(dataframes[f'/uav{str(i)}/msg_correction'].iloc[0,1], target.iloc[0,2], 'go', markersize=8)
                point2, = ax.plot(target.iloc[-1,1], target.iloc[-1,2], 'ro',  markersize=8)
                point3, = ax.plot(target.iloc[i,1], target.iloc[i,2], 'ko',  markersize=8)
                point4, = ax.plot(dataframes[f'/uav{str(i)}/msg_correction'].iloc[j,1], dataframes[f'/uav{str(i)}/msg_correction'].iloc[j,2], 'px', markersize=8)
                point5, = ax.plot(dataframes[f'/uav{str(i)}/msg_true'].iloc[j,1], dataframes[f'/uav{str(i)}/msg_true'].iloc[j,2], 'kx', markersize=8)
                return line, point1, point2, point3, point4, point5,
        
            im_basename_gif = 'Corrections' + str(i) + '.gif'
            im_fn_gif = os.path.join(folder_png, im_basename_gif) if folder_png else im_basename_gif
            ani = FuncAnimation(fig, animate, interval=40, blit=True, repeat=True, frames = len(target.index))    
            ani.save(im_fn_gif, writer=PillowWriter(fps=20))
            
        
    # data of errors and measurments
    
    
    error_fusion = pd.DataFrame(columns=['UAV','error_x', 'error_y', 'RMSE_x', 'RMSE_y', 'euclidean', 'error_v_x', 'error_v_y', 'RMSE_v_x', 'RMSE_v_y', 'error_Timestamp'])
    

    if (single == True):
        data_error['uav1'].to_csv( folder_sim + '/error_uav_single.csv')
        error_fusion.loc[len(error_fusion)] = ['uav1' ,
                                                np.mean(data_error['uav1'].error_x) ,
                                                np.mean(data_error['uav1'].error_y),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].x_target, data_error['uav1'].x)),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].y_target, data_error['uav1'].y)),
                                                np.mean(data_error['uav1'].euclidean), np.mean(data_error['uav1'].error_v_x), np.mean(data_error['uav1'].error_v_y),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].v_x_target, data_error['uav1'].v_x)),
                                                math.sqrt(sklearn.metrics.mean_squared_error(data_error['uav1'].v_y_target, data_error['uav1'].v_y)),
                                                np.mean(data_error['uav1'].error_Timestamp)]
        error_fusion.to_csv( folder_sim + '/error_med_single.csv')
        
    else:    
        for i in range(1, uav_total+1):
            data_error[f'uav{str(i)}'].to_csv( folder_sim + f'/uav{str(i)}.csv')
            error_fusion.loc[len(error_fusion)] = [f'uav{str(i)}' ,
                                                    np.mean(data_error[f'uav{str(i)}'].error_x) ,
                                                    np.mean(data_error[f'uav{str(i)}'].error_y),
                                                    math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].x_target, data_error[f'uav{str(i)}'].x)),
                                                    math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].y_target, data_error[f'uav{str(i)}'].y)),
                                                    np.mean(data_error[f'uav{str(i)}'].euclidean),
                                                    np.mean(data_error[f'uav{str(i)}'].error_v_x),
                                                    np.mean(data_error[f'uav{str(i)}'].error_v_y),
                                                    math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].v_x_target, data_error[f'uav{str(i)}'].v_x)),
                                                    math.sqrt(sklearn.metrics.mean_squared_error(data_error[f'uav{str(i)}'].v_y_target, data_error[f'uav{str(i)}'].v_y)),
                                                    np.mean(data_error[f'uav{str(i)}'].error_Timestamp)]
            
            
        error_fusion.to_csv( folder_sim + '/error_fusion.csv')


        total_precision = np.mean(precision)
        total_euclidean = np.mean(error_fusion.euclidean)
        total_RMSEx = np.mean(error_fusion.RMSE_x)
        total_RMSEy = np.mean(error_fusion.RMSE_y)
        performance = pd.DataFrame([[total_precision, total_euclidean, total_RMSEx, total_RMSEy]], columns=['precision', 'accuracy', 'RMSE_x', 'RMSE_y'])

        performance.to_csv( folder_sim + '/performance.csv')






