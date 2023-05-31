import subprocess as sp
import time
import datetime
import os
import signal
from pathlib import Path

from plot_auto import run_auto_plots

uav_total = 3

SIMTIME = 65
SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}")
sim_dir.mkdir()

png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}/png")
png_dir.mkdir()

pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}/pgf")
pgf_dir.mkdir()

bag_fn_single = sim_dir.joinpath(f"single{SIM_ID}.bag")

cmd_launch_single = "roslaunch sensor_fusion single_filters_sim.launch"
cmd_bag_single = f"rosbag record -a -O {str(bag_fn_single)}"


p_bag_single = sp.Popen(cmd_bag_single.split())


p_launch_single = sp.Popen(cmd_launch_single.split())


t_0 = time.time()

while time.time() - t_0 < SIMTIME :
    continue


os.kill(p_launch_single.pid, signal.SIGINT)

os.kill(p_bag_single.pid, signal.SIGINT)

time.sleep(15)

single = True

delay = False

delay_estimation = False

run_auto_plots(str(bag_fn_single), uav_total, single, delay, delay_estimation, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))


print("Now MULTI")

SIMTIME = 65
SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}")
sim_dir.mkdir()

png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}/png")
png_dir.mkdir()

pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}/pgf")
pgf_dir.mkdir()

bag_fn_multi = sim_dir.joinpath(f"multi{SIM_ID}.bag")

cmd_launch_multi = "roslaunch sensor_fusion multi_filters_sim.launch"
cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"
cmd_rosclean = "rosclean"


p_bag_multi = sp.Popen(cmd_bag_multi.split())


p_launch_multi = sp.Popen(cmd_launch_multi.split())


t_0 = time.time()

while time.time() - t_0 < SIMTIME :
    continue


os.kill(p_launch_multi.pid, signal.SIGINT)

os.kill(p_bag_multi.pid, signal.SIGINT)

time.sleep(15)


single = False

run_auto_plots(str(bag_fn_multi), uav_total, single, delay, delay_estimation, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))

print("Now MULTI Delay  not estimation")

SIMTIME = 65
SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay_not{SIM_ID}")
sim_dir.mkdir()

png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay_not{SIM_ID}/png")
png_dir.mkdir()

pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay_not{SIM_ID}/pgf")
pgf_dir.mkdir()

bag_fn_multi = sim_dir.joinpath(f"multi_delay{SIM_ID}.bag")

cmd_launch_multi = "roslaunch sensor_fusion multi_filters_sim_delay_not.launch"
cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"


p_bag_multi = sp.Popen(cmd_bag_multi.split())


p_launch_multi = sp.Popen(cmd_launch_multi.split())


t_0 = time.time()

while time.time() - t_0 < SIMTIME :
    continue


os.kill(p_launch_multi.pid, signal.SIGINT)

os.kill(p_bag_multi.pid, signal.SIGINT)

time.sleep(20)


single = False

delay = True

delay_estimation = False

run_auto_plots(str(bag_fn_multi), uav_total, single, delay, delay_estimation, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))


print("Now MULTI Delay estimation")

SIMTIME = 65
SIM_ID = int(time.time())
sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay{SIM_ID}")
sim_dir.mkdir()

png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay{SIM_ID}/png")
png_dir.mkdir()

pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay{SIM_ID}/pgf")
pgf_dir.mkdir()

bag_fn_multi = sim_dir.joinpath(f"multi_delay{SIM_ID}.bag")

cmd_launch_multi = "roslaunch sensor_fusion multi_filters_sim_delay.launch"
cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"


p_bag_multi = sp.Popen(cmd_bag_multi.split())


p_launch_multi = sp.Popen(cmd_launch_multi.split())


t_0 = time.time()

while time.time() - t_0 < SIMTIME :
    continue


os.kill(p_launch_multi.pid, signal.SIGINT)

os.kill(p_bag_multi.pid, signal.SIGINT)

time.sleep(20)


single = False

delay_estimation = True

run_auto_plots(str(bag_fn_multi), uav_total, single, delay, delay_estimation, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))