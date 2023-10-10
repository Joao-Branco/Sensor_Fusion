import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics
import pandas as pd
import os.path
from pathlib import Path

def circular_difference(angle1, angle2):
    signed_difference = angle2 - angle1
    normalized_difference = (signed_difference + np.pi) % (2 * np.pi) - np.pi
    return normalized_difference


def sim_plot(sensor_masks, state : np, predicts : list, predict_masks : list, n_uavs : int, col_write, x, y,  z_obs, z_corr, z_masks, delay, delay_strategy, ekf, share, dir_plot, dir_result, sensors, time, f_s, pi = 1):


    exp = '/_share_'+ str(share) + '_ekf_' + str(ekf) + '_strategy_' + str(delay_strategy) + '_mean_' + str(delay) + 'freq_share_' + str(f_s) + 'pi_' + str(pi)+ 'n_uavs_' + str(n_uavs)
    dir_plots = Path(dir_plot + exp)
    dir_plots.mkdir()
    dir_results = Path(dir_result + exp)
    dir_results.mkdir()

    for pred, pred_mask in zip(predicts, predict_masks):
        if(ekf == True):
            state_filtered = state[:5,pred_mask]
            state_lst = ['X', 'Y', 'Vx', 'Vy', 'W']
            err_abs = np.abs(state_filtered - pred[1:,:]) # ignore time row from pred
        else:
            state_filtered = state[:4,pred_mask]
            state_lst = ['X', 'Y', 'Vx', 'Vy']
            err_abs = np.abs(state_filtered - pred[1:,:]) # ignore time row from pred
            

        euclidean = np.sqrt(err_abs[0,:] ** 2 + err_abs[1,:] ** 2)

    x_noise = [_ for _ in range(n_uavs)]
    y_noise = [_ for _ in range(n_uavs)]
    t_noise = [_ for _ in range(n_uavs)]

    for i, (sen, sen_mask) in enumerate(zip(sensors, sensor_masks)):
        x_noise[i] = sen[0][sen_mask]
        y_noise[i] = sen[1][sen_mask]
        t_noise[i] = time[sen_mask]


    for i, st in enumerate(state_lst):
        plt.figure(figsize=(9, 5))
        plt.plot(time, state[i], 'k', markersize=5, label= st)
        for u in range(n_uavs):
            plt.plot(predicts[u][0, :], predicts[u][i + 1, :], 'x', markersize=2, label= 'UAV ' + str(u + 1))
        plt.xlabel('t (s)', fontsize=15)
        plt.ylabel(st, fontsize=15)
        plt.legend(fontsize=10)
        plt.grid()

        plot_jpg = 'Error___' + st + '.png'
    
        plot_jpg = os.path.join(dir_plots, plot_jpg) if dir_plots else plot_jpg
        plt.savefig(plot_jpg)
        plt.close()

    for i, st in enumerate(state_lst):
        plt.figure(figsize=(8, 4))
        plt.plot(time, state[i], markersize=5, label= st)
        plt.xlabel('t (s)', fontsize=10)
        plt.ylabel(st, fontsize=10)
        plt.legend(fontsize=10)
        plt.grid()

        plot_jpg = 'State___' + st + '.png'
    
        plot_jpg = os.path.join(dir_plots, plot_jpg) if dir_plots else plot_jpg
        plt.savefig(plot_jpg)
        plt.close()
    



    dist_uavs = []

    for i_uav in range(n_uavs -1):
        for j_uav in range(n_uavs):
            if j_uav > i_uav :
                x_e = predicts[i_uav][1,:] - predicts[j_uav][1,:]
                y_e = predicts[i_uav][2,:] - predicts[j_uav][2,:]
                dist_uavs.append(np.sqrt(x_e ** 2 + y_e ** 2)) 

    dist_uavs = np.array(dist_uavs)   

    plt.figure(figsize=(6, 6))

    plt.rcParams['axes.prop_cycle'] = plt.cycler('color', plt.cm.tab20.colors)
    
    for i in range(n_uavs):
        sensor_x, sensor_y = x_noise[i], y_noise[i]
        x_, y_ = predicts[i][1,:], predicts[i][2,:]
        if (n_uavs == 1):
            plt.plot(x_[:col_write], y_[:col_write], '+', markersize=3, label='Estimativa')
            plt.plot(sensor_x, sensor_y, 'o', markersize=0.8, label='Observações')
        else:
            plt.plot(x_[:col_write], y_[:col_write], '+', markersize=3, label='UAV ' + str(i + 1))
            plt.plot(sensor_x, sensor_y, 'o', markersize=0.8, label='Observações' + str(i + 1))
    plt.plot(x,y, 'k',linewidth='3', label="Alvo")
    if (x[0] != x[-1]):
        plt.plot(x[0],y[0], 'og', markersize=8)
        plt.plot(x[-1],y[-1], 'or', markersize=8)
    else:
        plt.plot(x[0],y[0], 'ok', markersize=5)

    plt.xlabel('X (m)', fontsize=10)
    plt.ylabel('Y (m)', fontsize=10)
    plt.legend(fontsize=10)
    plt.grid()


    plot_jpg = 'Dinamica_alvo_share_'+ str(share) + '_ekf_' + str(ekf) + '_strategy_' + str(delay_strategy) + '_mean_' + str(delay) +  '.png'
    
    plot_jpg = os.path.join(dir_plots, plot_jpg) if dir_plots else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()


    plt.figure(figsize=(6, 6))
    
    for i in range(n_uavs):
        plt.plot(t_noise[i], x_noise[i], 'o', markersize=0.5, label='UAV_obs' + str(i + 1))
    plt.title("Posição", fontsize=20)
    plt.xlabel('t (s)', fontsize=15)
    plt.ylabel('X (m)', fontsize=15)
    plt.legend(fontsize=10)
    plt.grid()


    plot_jpg = 'Observations_x_alvo_share_'+ str(share) + '_ekf_' + str(ekf) + '_strategy_' + str(delay_strategy) + '_mean_' + str(delay) +  '.png'
    
    plot_jpg = os.path.join(dir_plots, plot_jpg) if dir_plots else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()

    plt.figure(figsize=(6, 6))
    
    for i in range(n_uavs):
        y_noise[i]
        plt.plot(t_noise[i], y_noise[i], 'o', markersize=0.5, label='UAV_obs' + str(i + 1))
    plt.title("Posição", fontsize=20)
    plt.xlabel('t (s)', fontsize=15)
    plt.ylabel('Y (m)', fontsize=15)
    plt.legend(fontsize=10)
    plt.grid()


    plot_jpg = 'Observations_y_alvo_share_'+ str(share) + '_ekf_' + str(ekf) + '_strategy_' + str(delay_strategy) + '_mean_' + str(delay) +  '.png'
    
    plot_jpg = os.path.join(dir_plots, plot_jpg) if dir_plots else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()




    
    if (delay != 0 and delay_strategy != None and delay_strategy != 'augmented_state'):

        for i in range(n_uavs):
            n = 0
            t_col = 0
            for t, t_value in enumerate(z_obs[i]):

                # first observations dont extrapolate, due to dont have any observation in memory
                if np.all(z_obs[i][t][0] == z_corr[i][t][0]):
                    n += 1
                    continue
                else:
                    t_array = np.array([z_obs[i][t][1]])
                    z_obs[i][t_col] = np.concatenate((t_array, z_obs[i][t][0].flatten()))
                    z_corr[i][t_col] = np.concatenate((t_array, z_corr[i][t][0].flatten()))
                    t_col += 1

        # deleting the first observations that havent been extrapolated, in order to create a numpy array

            del z_obs[i][len(z_obs[i]) - n:]  
            del z_corr[i][len(z_corr[i]) - n:] 
            del z_masks[i][len(z_masks[i]) - n:] 

            z_obs[i] = np.array(z_obs[i])
            z_corr[i] = np.array(z_corr[i])
            z_masks[i] = np.array(z_masks[i], dtype=np.uint32)
            z_obs[i] = z_obs[i].T
            z_corr[i] = z_corr[i].T

        for obs, obs_mask in zip(z_obs, z_masks):
            state_filtered_obs = state[:(obs.shape[0] - 1),obs_mask]
            err_obs = np.abs(state_filtered_obs - obs[1:,:]) # ignore time row from pred



        for corr, corr_mask in zip(z_corr, z_masks):
            state_filtered_corr = state[:(corr.shape[0] - 1),corr_mask]
            err_corr = np.abs(state_filtered_corr - corr[1:,:]) # ignore time row from pred


        

    if(ekf == True):


        err_abs_mean = np.array([   np.mean(err_abs[0,:]),
                                    np.mean(err_abs[1,:]),
                                    np.mean(err_abs[2,:]),
                                    np.mean(err_abs[3,:]),
                                    np.mean(err_abs[4,:])])

        rmse = np.array([   np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[0,:], pred[0,:])),
                            np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[1,:], pred[1,:])),
                            np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[2,:], pred[2,:])),
                            np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[3,:], pred[3,:])),
                            np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[4,:], pred[4,:]))])
        
        print("Absolute error x:  ", err_abs_mean[0])
        print("Absolute error y:  ", err_abs_mean[1])
        print("Absolute error v_x:  ", err_abs_mean[2])
        print("Absolute error v_y:  ", err_abs_mean[3])
        print("Absolute error w:  ", err_abs_mean[4])

        column_values = ['x', 'y', 'v_x', 'v_y', 'w']
        index_values = ['Error_abs', 'RMSE']
        dataframe = pd.DataFrame([err_abs_mean, rmse], index = index_values, columns = column_values)

        if (delay != 0 and delay_strategy == "extrapolate"):

            err_obs_mean = np.array([   np.mean(err_obs[0,:]),
                                        np.mean(err_obs[1,:])])

            err_corr_mean = np.array([   np.mean(err_corr[0,:]),
                                        np.mean(err_corr[1,:])])

            print(f"X---Absolute error obs: {err_obs_mean[0]}, Absolute error corr: {err_corr_mean[0]}")
            print(f"Y---Absolute error obs: {err_obs_mean[1]}, Absolute error corr: {err_corr_mean[1]}")

            index_values = ['Error_obs', 'Error_corr']
            column_values = ['x', 'y']
            pdd = pd.DataFrame([err_obs_mean, err_corr_mean], index = index_values, columns = column_values)

            dataframe = pd.concat([dataframe, pdd])

            for i, state in enumerate(column_values):

                if err_obs_mean[i] < err_corr_mean[i]:
                    print(f"{state}-----OBS")
                elif err_obs_mean[i] > err_corr_mean[i]:
                    print(f"{state}-----CORR") 
                else:
                    print(f"{state} state not interpolated")

        





        print("RMSE x:  ", rmse[0])
        print("RMSE y:  ", rmse[1])
        print("RMSE v_x:  ", rmse[2])
        print("RMSE v_y:  ", rmse[3])
        print("RMSE w:  ", rmse[4])

    else:

        err_abs_mean = np.array([   np.mean(err_abs[0,:]),
                                    np.mean(err_abs[1,:]),
                                    np.mean(err_abs[2,:]),
                                    np.mean(err_abs[3,:])])

        rmse = np.array([   np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[0,:], pred[0,:])),
                            np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[1,:], pred[1,:])),
                            np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[2,:], pred[2,:])),
                            np.sqrt(sklearn.metrics.mean_squared_error(state_filtered[3,:], pred[3,:]))])


        column_values = ['x', 'y', 'v_x', 'v_y']
        index_values = ['Error_abs', 'RMSE']
        dataframe = pd.DataFrame([err_abs_mean, rmse], index = index_values, columns = column_values)
        
        if (delay != 0 and delay_strategy == "extrapolate"):

            err_obs_mean = np.array([   np.mean(err_obs[0,:]),
                                        np.mean(err_obs[1,:])])

            err_corr_mean = np.array([   np.mean(err_corr[0,:]),
                                        np.mean(err_corr[1,:])])

            print(f"X---Absolute error obs: {err_obs_mean[0]}, Absolute error corr: {err_corr_mean[0]}")
            print(f"Y---Absolute error obs: {err_obs_mean[1]}, Absolute error corr: {err_corr_mean[1]}")

            index_values = ['Error_obs', 'Error_corr']
            column_values = ['x', 'y']
            pdd = pd.DataFrame([err_obs_mean, err_corr_mean], index = index_values, columns = column_values)

            dataframe = pd.concat([dataframe, pdd])

            for i, state in enumerate(column_values):

                if err_obs_mean[i] < err_corr_mean[i]:
                    print(f"{state}-----OBS")
                elif err_obs_mean[i] > err_corr_mean[i]:
                    print(f"{state}-----CORR") 
                else:
                    print(f"{state} state not interpolated")

    



        print("RMSE x:  ", rmse[0])
        print("RMSE y:  ", rmse[1])
        print("RMSE v_x:  ", rmse[2])
        print("RMSE v_y:  ", rmse[3])



    dataframe.to_csv( str(dir_results) + '/data_errors_share_' + str(share) + '_ekf_' + str(ekf) + '_strategy_' + str(delay_strategy) +  '_mean_' + str(delay) +   '.csv')


    print("Accuracy: ", np.mean(euclidean))
    print("Precision: ", np.mean(dist_uavs))

    performance = pd.DataFrame([np.mean(euclidean), np.mean(dist_uavs)], index = ['Accuracy', 'Precision'])

    performance.to_csv( str(dir_results) + '/performance_share_'+ str(share) + '_ekf_' + str(ekf)+ '_strategy_' + str(delay_strategy) + '_mean_' + str(delay) + '.csv')

    return np.mean(euclidean), np.mean(dist_uavs), euclidean