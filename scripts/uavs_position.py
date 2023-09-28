#! /usr/bin/env python

import rospy
import numpy as np
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position


def calculate_uav_positions(num_uavs, radius, speed, time):
    if num_uavs <= 0 or radius <= 0 or speed <= 0 or time < 0:
        return []

    positions = []
    angular_speed = speed / radius  # Calculate angular speed in radians per second

    for i in range(num_uavs):
        angle = (2 * np.pi * i) / num_uavs  # Calculate the angle for each UAV
        x = radius * np.cos(angular_speed * time + angle)
        y = radius * np.sin(angular_speed * time + angle)
        positions.append((x, y))

    return positions


if __name__ == "__main__":
    rospy.init_node("uavs_positions_py")
    uav_total = rospy.get_param("/total_uav")
    rospy.loginfo("Node UAV positions has started")


    pub = []

    for i in range(1, uav_total + 1):
        pub.append(rospy.Publisher('/uav' + str(i) + '/position', target_position, queue_size=10))
    
    f = 30
    rate = rospy.Rate(f)

    t = 0

    while not rospy.is_shutdown(): 
        t = t + 1 / f
        position_uavs = calculate_uav_positions(uav_total, 50, 6, t)     
        for i in range(uav_total):
            pub[i].publish(position_uavs[i][0], position_uavs[i][1], rospy.Time.now())
        
        #rospy.loginfo("Positions published")

        rate.sleep()
    
