import subprocess as sp
import time
import datetime
import os
import signal
from pathlib import Path

from plot_auto import run_auto_plots

uav_total = 5

SIMTIME = 10
SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}")
sim_dir.mkdir()

bag_fn_single = sim_dir.joinpath(f"single{SIM_ID}.bag")

cmd_launch_single = "roslaunch sensor_fusion single_filters.launch"
cmd_bag_single = f"rosbag record -a -O {str(bag_fn_single)}"

p_launch_single = sp.Popen(cmd_launch_single.split())

p_bag_single = sp.Popen(cmd_bag_single.split())

t_0 = time.time()

while time.time() - t_0 < SIMTIME :
    continue


os.kill(p_launch_single.pid, signal.SIGINT)

os.kill(p_bag_single.pid, signal.SIGINT)

time.sleep(2)

single = True

run_auto_plots(str(bag_fn_single), uav_total, single, folder=str(sim_dir))


print("Now MULTI")

SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}")
sim_dir.mkdir()

bag_fn_multi = sim_dir.joinpath(f"multi{SIM_ID}.bag")

cmd_launch_multi = "roslaunch sensor_fusion multi_filters.launch"
cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"

p_launch_multi = sp.Popen(cmd_launch_multi.split())

p_bag_multi = sp.Popen(cmd_bag_multi.split())

t_0 = time.time()

while time.time() - t_0 < SIMTIME :
    continue


os.kill(p_launch_multi.pid, signal.SIGINT)

os.kill(p_bag_multi.pid, signal.SIGINT)

time.sleep(2)


single = False

run_auto_plots(str(bag_fn_multi), uav_total, single, folder=str(sim_dir))
