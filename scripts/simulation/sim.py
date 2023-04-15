import subprocess as sp
import time
import datetime
import os
import signal
from pathlib import Path

from plot_auto import run_auto_plots

uav_total = 5

bag_fn = r'/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi1681381667/multi1681381667.bag'
SIM_ID = int(time.time())

sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}")
sim_dir.mkdir()


single = True
run_single_plots(str(bag_fn), uav_total, single, folder=str(sim_dir))


print("Now MULTI")

SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}")
sim_dir.mkdir()

single = False
run_multi_plots(str(bag_fn), folder=str(sim_dir), uav_total, single)
