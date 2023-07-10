import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics
import pandas as pd


def sim_plot(state, predicts, predict_masks, n_uavs : int, col_write, x, y,  z_obs, z_corr, z_masks, delay, delay_strategy, ekf):

    for pred, pred_mask in zip(predicts, predict_masks):
        state_filtered = state[:,pred_mask]
        err_abs = np.abs(state_filtered - pred[1:,:]) # ignore time row from pred
        euclidean = np.sqrt(err_abs[0,:] ** 2 + err_abs[1,:] ** 2)

    dist_uavs = []

    for i_uav in range(n_uavs -1):
        for j_uav in range(n_uavs):
            if j_uav > i_uav :
                x_e = predicts[i_uav][1,:] - predicts[j_uav][1,:]
                y_e = predicts[i_uav][2,:] - predicts[j_uav][2,:]
                dist_uavs.append(np.sqrt(x_e ** 2 + y_e ** 2)) 

    dist_uavs = np.array(dist_uavs)   

    for i in range(n_uavs):
        x_, y_ = predicts[i][1,:], predicts[i][2,:]
        plt.plot(x_[:col_write], y_[:col_write], 'x')
    plt.plot(x,y, 'k')
    plt.grid()
    plt.show()


    
    if (delay == True and delay_strategy != None):

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
            state_filtered_obs = state[:,obs_mask]
            err_obs = np.abs(state_filtered_obs - obs[1:,:]) # ignore time row from pred

        for corr, corr_mask in zip(z_corr, z_masks):
            state_filtered_corr = state[:,corr_mask]
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
        print("Absolute error theta:  ", err_abs_mean[2])
        print("Absolute error v:  ", err_abs_mean[3])
        print("Absolute error w:  ", err_abs_mean[4])

        if (delay == True and delay_strategy != None):

            err_obs_mean = np.array([   np.mean(err_obs[0,:]),
                                        np.mean(err_obs[1,:]),
                                        np.mean(err_obs[2,:]),
                                        np.mean(err_obs[3,:]),
                                        np.mean(err_obs[4,:])])

            err_corr_mean = np.array([   np.mean(err_corr[0,:]),
                                        np.mean(err_corr[1,:]),
                                        np.mean(err_corr[2,:]),
                                        np.mean(err_corr[3,:]),
                                        np.mean(err_corr[4,:])])

            print(f"X---Absolute error obs: {err_obs_mean[0]}, Absolute error corr: {err_corr_mean[0]}")
            print(f"Y---Absolute error obs: {err_obs_mean[1]}, Absolute error corr: {err_corr_mean[1]}")
            print(f"THETA---Absolute error obs: {err_obs_mean[2]}, Absolute error corr: {err_corr_mean[2]}")
            print(f"V---Absolute error obs: {err_obs_mean[3]}, Absolute error corr: {err_corr_mean[3]}")
            print(f"W---Absolute error obs: {err_obs_mean[4]}, Absolute error corr: {err_corr_mean[4]}")


            for i, state in enumerate(['x', 'y', 'theta', 'v', 'w']):

                if err_obs_mean[i] < err_corr_mean[i]:
                    print(f"{state}-----OBS")
                elif err_obs_mean[i] > err_corr_mean[i]:
                    print(f"{state}-----CORR") 
                else:
                    print(f"{state} state not interpolated")



        print("RMSE x:  ", rmse[0])
        print("RMSE y:  ", rmse[1])
        print("RMSE theta:  ", rmse[2])
        print("RMSE v:  ", rmse[3])
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
        
        if (delay == True and delay_strategy != None):
        
            err_obs_mean = np.array([   np.mean(err_obs[0,:]),
                                        np.mean(err_obs[1,:]),
                                        np.mean(err_obs[2,:]),
                                        np.mean(err_obs[3,:])])

            err_corr_mean = np.array([   np.mean(err_corr[0,:]),
                                        np.mean(err_corr[1,:]),
                                        np.mean(err_corr[2,:]),
                                        np.mean(err_corr[3,:])])

            print(f"X---Absolute error obs: {err_obs_mean[0]}, Absolute error corr: {err_corr_mean[0]}")
            print(f"Y---Absolute error obs: {err_obs_mean[1]}, Absolute error corr: {err_corr_mean[1]}")
            print(f"V_X---Absolute error obs: {err_obs_mean[2]}, Absolute error corr: {err_corr_mean[2]}")
            print(f"V_Y---Absolute error obs: {err_obs_mean[3]}, Absolute error corr: {err_corr_mean[3]}")



            for i, state in enumerate(['x', 'y', 'v_x', 'v_y']):

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


    print("Accuracy: ", np.mean(euclidean))
    print("Precision: ", np.mean(dist_uavs))