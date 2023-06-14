import subprocess as sp
import time
import datetime
import os
import signal
from pathlib import Path

from plot_auto import run_auto_plots



def single(SIMTIME):

    print("Now SINGLE")

    uav_total = 1

    SIM_ID = int(time.time())
    sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}")
    sim_dir.mkdir()

    png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}/png")
    png_dir.mkdir()

    pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-single{SIM_ID}/pgf")
    pgf_dir.mkdir()

    bag_fn_single = sim_dir.joinpath(f"single{SIM_ID}.bag")

    cmd_roscore = "roscore"
    cmd_launch = "roslaunch sensor_fusion single_filters_sim.launch"
    #cmd_bag = f"rosbag record -a -O {str(bag_fn_single)}"

    record_topics = '''//rosout
        /rosout_agg
        /target_position
        /uav1/target_position
        /uav1/target_position_estimation
        /uav1/target_position_fuse
        '''

    record_topics = record_topics.splitlines()
    record_topics = [t.strip() for t in record_topics if t.strip() != '']

    cmd_bag = f"rosbag record -O {str(bag_fn_single)} {' '.join(record_topics)}"

    p_roscore = sp.Popen(cmd_roscore.split())

    time.sleep(1)

    p_bag = sp.Popen(cmd_bag.split())

    time.sleep(5)

    p_launch = sp.Popen(cmd_launch.split())


    t_0 = time.time()

    while time.time() - t_0 < SIMTIME :
        continue


    os.kill(p_bag.pid, signal.SIGINT)

    time.sleep(10)

    os.kill(p_launch.pid, signal.SIGINT)

    time.sleep(10)

    os.kill(p_roscore.pid, signal.SIGINT)


    single = True

    delay = False

    delay_estimation = False

    fuse = False

    run_auto_plots(str(bag_fn_single), uav_total, single, delay, delay_estimation, fuse, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))

def multi_fuse(SIMTIME):

    print("Now MULTI FUSE")

    uav_total = 3
    SIM_ID = int(time.time())
    sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi-fuse{SIM_ID}")
    sim_dir.mkdir()

    png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi-fuse{SIM_ID}/png")
    png_dir.mkdir()

    pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi-fuse{SIM_ID}/pgf")
    pgf_dir.mkdir()

    bag_fn_multi = sim_dir.joinpath(f"multi{SIM_ID}.bag")

    cmd_roscore = "roscore"
    cmd_launch_multi = "roslaunch sensor_fusion multi_filters_sim.launch"
    #cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"


    record_topics = '''/rosout
        /rosout_agg
        /target_position
        /uav1/target_position
        /uav1/target_position_estimation
        /uav1/target_position_fuse
        /uav2/target_position
        /uav2/target_position_estimation
        /uav2/target_position_fuse
        /uav3/target_position
        /uav3/target_position_estimation
        /uav3/target_position_fuse
        '''
    
    record_topics = record_topics.splitlines()
    record_topics = [t.strip() for t in record_topics if t.strip() != '']

    cmd_bag_multi = f"rosbag record -O {str(bag_fn_multi)} {' '.join(record_topics)}"


    p_roscore = sp.Popen(cmd_roscore.split())

    time.sleep(10)

    p_bag_multi = sp.Popen(cmd_bag_multi.split())

    time.sleep(10)

    p_launch_multi = sp.Popen(cmd_launch_multi.split())






    t_0 = time.time()

    while time.time() - t_0 < SIMTIME :
        continue


    os.kill(p_bag_multi.pid, signal.SIGINT)

    time.sleep(5)

    os.kill(p_launch_multi.pid, signal.SIGINT)

    time.sleep(1)

    os.kill(p_roscore.pid, signal.SIGINT)

    time.sleep(4)



    single = False

    delay = False

    delay_estimation = False

    fuse = True

    run_auto_plots(str(bag_fn_multi), uav_total, single, delay, delay_estimation, fuse, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))



def multi(SIMTIME):

    print("Now MULTI")

    uav_total = 3
    SIM_ID = int(time.time())
    sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}")
    sim_dir.mkdir()

    png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}/png")
    png_dir.mkdir()

    pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi{SIM_ID}/pgf")
    pgf_dir.mkdir()

    bag_fn_multi = sim_dir.joinpath(f"multi{SIM_ID}.bag")

    cmd_roscore = "roscore"
    cmd_launch_multi = "roslaunch sensor_fusion multi_filters_sim_not_fuse.launch"
    #cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"


    record_topics = '''/rosout
        /rosout_agg
        /target_position
        /uav1/target_position
        /uav1/target_position_estimation
        /uav1/target_position_fuse
        /uav2/target_position
        /uav2/target_position_estimation
        /uav2/target_position_fuse
        /uav3/target_position
        /uav3/target_position_estimation
        /uav3/target_position_fuse
        '''
    
    record_topics = record_topics.splitlines()
    record_topics = [t.strip() for t in record_topics if t.strip() != '']

    cmd_bag_multi = f"rosbag record -O {str(bag_fn_multi)} {' '.join(record_topics)}"


    p_roscore = sp.Popen(cmd_roscore.split())

    time.sleep(10)

    p_bag_multi = sp.Popen(cmd_bag_multi.split())

    time.sleep(10)

    p_launch_multi = sp.Popen(cmd_launch_multi.split())


    t_0 = time.time()

    while time.time() - t_0 < SIMTIME :
        continue



    os.kill(p_bag_multi.pid, signal.SIGINT)

    time.sleep(5)

    os.kill(p_launch_multi.pid, signal.SIGINT)

    time.sleep(1)

    os.kill(p_roscore.pid, signal.SIGINT)

    time.sleep(4)


    single = False

    delay = False

    delay_estimation = False

    fuse = False

    run_auto_plots(str(bag_fn_multi), uav_total, single, delay, delay_estimation, fuse, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))

def multi_fuse_delay_not(SIMTIME):

    print("Now MULTI Delay  not estimation")

    uav_total = 3
    SIM_ID = int(time.time())
    sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay_not{SIM_ID}")
    sim_dir.mkdir()

    png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay_not{SIM_ID}/png")
    png_dir.mkdir()

    pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay_not{SIM_ID}/pgf")
    pgf_dir.mkdir()

    bag_fn_multi = sim_dir.joinpath(f"multi_delay{SIM_ID}.bag")

    cmd_roscore = "roscore"
    cmd_launch_multi = "roslaunch sensor_fusion multi_filters_sim_delay_not.launch"
    cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"


    record_topics = '''/rosout
        /rosout_agg
        /target_position
        /uav1/delay
        /uav1/delay_estimation
        /uav1/msg_correction
        /uav1/msg_true
        /uav1/samples_delay
        /uav1/target_position
        /uav1/target_position_delay
        /uav1/target_position_estimation
        /uav1/target_position_fuse
        /uav2/delay
        /uav2/delay_estimation
        /uav2/msg_correction
        /uav2/msg_true
        /uav2/samples_delay
        /uav2/target_position
        /uav2/target_position_delay
        /uav2/target_position_estimation
        /uav2/target_position_fuse
        /uav3/delay
        /uav3/delay_estimation
        /uav3/msg_correction
        /uav3/msg_true
        /uav3/samples_delay
        /uav3/target_position
        /uav3/target_position_delay
        /uav3/target_position_estimation
        /uav3/target_position_fuse
        '''
    
    record_topics = record_topics.splitlines()
    record_topics = [t.strip() for t in record_topics if t.strip() != '']

    #cmd_bag_multi = f"rosbag record -O {str(bag_fn_multi)} {' '.join(record_topics)}"


    p_roscore = sp.Popen(cmd_roscore.split())

    time.sleep(10)

    p_bag_multi = sp.Popen(cmd_bag_multi.split())

    time.sleep(10)

    p_launch_multi = sp.Popen(cmd_launch_multi.split())


    t_0 = time.time()

    while time.time() - t_0 < SIMTIME :
        continue


    os.kill(p_bag_multi.pid, signal.SIGINT)

    time.sleep(5)

    os.kill(p_launch_multi.pid, signal.SIGINT)

    time.sleep(1)

    os.kill(p_roscore.pid, signal.SIGINT)

    time.sleep(4)


    single = False

    delay = True

    delay_estimation = False

    fuse = True

    run_auto_plots(str(bag_fn_multi), uav_total, single, delay, delay_estimation, fuse, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))

def multi_fuse_delay(SIMTIME):
    print("Now MULTI Delay estimation")

    uav_total = 3
    SIM_ID = int(time.time())
    sim_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay{SIM_ID}")
    sim_dir.mkdir()

    png_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay{SIM_ID}/png")
    png_dir.mkdir()

    pgf_dir = Path(f"/home/branco/catkin_ws/src/sensor_fusion/sims/sim-multi_delay{SIM_ID}/pgf")
    pgf_dir.mkdir()

    bag_fn_multi = sim_dir.joinpath(f"multi_delay{SIM_ID}.bag")

    cmd_roscore = "roscore"
    cmd_launch_multi = "roslaunch sensor_fusion multi_filters_sim_delay.launch"
    cmd_bag_multi = f"rosbag record -a -O {str(bag_fn_multi)}"


    record_topics = '''/rosout
        /rosout_agg
        /target_position
        /uav1/delay
        /uav1/delay_estimation
        /uav1/msg_correction
        /uav1/msg_true
        /uav1/samples_delay
        /uav1/target_position
        /uav1/target_position_delay
        /uav1/target_position_estimation
        /uav1/target_position_fuse
        /uav2/delay
        /uav2/delay_estimation
        /uav2/msg_correction
        /uav2/msg_true
        /uav2/samples_delay
        /uav2/target_position
        /uav2/target_position_delay
        /uav2/target_position_estimation
        /uav2/target_position_fuse
        /uav3/delay
        /uav3/delay_estimation
        /uav3/msg_correction
        /uav3/msg_true
        /uav3/samples_delay
        /uav3/target_position
        /uav3/target_position_delay
        /uav3/target_position_estimation
        /uav3/target_position_fuse
        '''
    
    record_topics = record_topics.splitlines()
    record_topics = [t.strip() for t in record_topics if t.strip() != '']

    #cmd_bag_multi = f"rosbag record -O {str(bag_fn_multi)} {' '.join(record_topics)}"


    p_roscore = sp.Popen(cmd_roscore.split())

    time.sleep(3)

    p_bag_multi = sp.Popen(cmd_bag_multi.split())

    time.sleep(10)

    p_launch_multi = sp.Popen(cmd_launch_multi.split())


    t_0 = time.time()

    while time.time() - t_0 < SIMTIME :
        continue


    os.kill(p_bag_multi.pid, signal.SIGINT)

    time.sleep(5)

    os.kill(p_launch_multi.pid, signal.SIGINT)

    time.sleep(1)

    os.kill(p_roscore.pid, signal.SIGINT)

    time.sleep(4)


    single = False

    delay = True

    delay_estimation = True

    fuse = True

    run_auto_plots(str(bag_fn_multi), uav_total, single, delay, delay_estimation, fuse, folder_png=str(png_dir), folder_pgf=str(pgf_dir), folder_sim=str(sim_dir))