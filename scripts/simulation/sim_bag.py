import subprocess as sp
import time
import datetime
import os
import signal
from pathlib import Path

from plot_single import run_single_plots
from plot_multi import run_multi_plots

SIMTIME = 50
SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims")
sim_dir.mkdir()

bag_fn_single = sim_dir.joinpath(f"target_noise{SIM_ID}.bag")

cmd_launch_single = "roslaunch sensor_fusion bag_target_noise.launch"
cmd_bag_single = f"rosbag record -a -O {str(bag_fn_single)}"

p_launch_single = sp.Popen(cmd_launch_single.split())

p_bag_single = sp.Popen(cmd_bag_single.split())

t_0 = time.time()

while time.time() - t_0 < SIMTIME :
    continue


os.kill(p_launch_single.pid, signal.SIGINT)

os.kill(p_bag_single.pid, signal.SIGINT)
