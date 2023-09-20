import numpy as np
from pathlib import Path
import os


def discrete_printing(list_uav : list, dir : Path, w):
    
    t = list_uav[0]
    mode = list_uav[1]

    dir_i = os.path.join(str(dir), f'{w : 07d}___{mode}____{t}_')
    os.mkdir(dir_i)
    w = w + 1
    
    if (mode == 'predict'):

        vars_list = ['x_previus','x_target', 'x', 'P', 'J']

    elif (mode == 'update'):
        vars_list = ['x_previus', 'x_target', 'x', 'P', 'H', 'z', 'K', 'y']
        
    else:
        vars_list = ['x_previus', 'x_target', 'x', 'P', 'H', 'z', 'N', 'delay']



    for i, var  in enumerate(vars_list):

        file_name =  var + '.txt'
    
        file_name = os.path.join(dir_i, file_name) if dir_i else file_name
        if (var == 'H'):
            fmt_e = '% d'
        if(var in ['x', 'x_target', 'x_previus']):
            fmt_e='%1.9f'
        else:
            fmt_e='%1.3f'

        np.savetxt(file_name, list_uav[i+2], fmt=fmt_e)

