import numpy as np
from pathlib import Path
import os.path


def discrete_printing(discrete_kf : list, uav_i : int, dir : Path):
    w = 0
    for list_uav in discrete_kf[uav_i]:
        t = list_uav[0]
        mode = list_uav[1]
    
        dir_i = Path(str(dir) + f'/{w}___{mode}____{t}_')
        dir_i.mkdir()
        w = w + 1
        
        if (mode == 'predict'):
    
            vars_list = ['x_previus','x_target', 'x', 'P', 'J']
    
        elif (mode == 'update'):
            vars_list = ['x_previus', 'x_target', 'x', 'P', 'H', 'z', 'K', 'y']
            
        else:
            vars_list = ['x_previus', 'x_target', 'x', 'P', 'H', 'z', 'K', 'y', 'N', 'delay']

    
    
        for i, var  in enumerate(vars_list):
    
            file_name = mode + '______matrix____' + var + '.txt'
        
            file_name = os.path.join(dir_i, file_name) if dir_i else file_name
            if (var == 'H'):
                fmt_e = '% d'
            if(var in ['x', 'x_target', 'x_previus']):
                fmt_e='%1.9f'
            else:
                fmt_e='%1.3f'

            np.savetxt(file_name, list_uav[i+2], fmt=fmt_e)