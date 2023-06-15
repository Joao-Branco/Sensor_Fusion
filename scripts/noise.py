#! /usr/bin/env python

import rospy
import math
import numpy as np
from sensor_fusion.msg import target_position

#target_x, target_y = 0,0

def target_callback(msg):
    # definition of parameters of noise
    # std - standard deviation     mean
    #target_x, target_y = msg.x, msg.y
    
    std = 2
    mean = 0
    
    noise_uav_x = np.random.normal(mean,std)
    noise_uav_y = np.random.normal(mean,std)        
    
    
    pub_uav.publish(msg.x + noise_uav_x, msg.y + noise_uav_y, msg.v_x, msg.v_y, msg.timestamp)
    #rospy.loginfo("Noise has been published in UAV" + str(uav_id))

if __name__ == "__main__":
    rospy.init_node("Noise_py")
    rospy.loginfo("Node Noise has started")
    uav_id = rospy.get_param("~uav_id")

    rospy.Subscriber('/target_position', target_position, target_callback, queue_size=1)
    
    
    pub_uav = rospy.Publisher('target_position', target_position, queue_size=10)
    
    f = 5

    

    rate = rospy.Rate(f)
    

    while not rospy.is_shutdown():
        rate.sleep()