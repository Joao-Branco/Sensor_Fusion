import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics
import pandas as pd
import os.path
from pathlib import Path

def compare_plots(dir, dynamics_name, data, delay = None):

    for i in data:
        if (i[3] == dynamics_name):
            plt.plot(i[1][0][0, :], i[0], label=i[2])
    plt.title("Precisão", fontsize=20)
    plt.xlabel('Tempo (s)', fontsize=15)
    plt.ylabel('Erro (m)', fontsize=15)
    plt.legend(fontsize=10)
    plt.grid()


    plot_jpg = f'{dynamics_name}_{delay}' +  '.png'
    
    plot_jpg = os.path.join(dir, plot_jpg) if dir else plot_jpg
    plt.savefig(plot_jpg)

    plt.close()

    