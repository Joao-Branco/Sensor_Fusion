#! /usr/bin/env python

import rospy
import math
import numpy as np
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position
    
def target_callback(msg):
    # definition of parameters of noise
    # std - standard deviation     mean
    
    std = 2
    mean = 0
    
    noise_uav_x = np.random.normal(mean,std)
    noise_uav_y = np.random.normal(mean,std)        
    
    
    pub_uav.publish(msg.x + noise_uav_x, msg.y + noise_uav_y)
    rospy.loginfo("Noise has been published")

if __name__ == "__main__":
    rospy.init_node("Noise_py")
    rospy.loginfo("Node Noise has started")
    uav_id = rospy.get_param("~uav_id")


    rospy.Subscriber('/target_position', target_position, target_callback)
    
    
    pub_uav = rospy.Publisher('/uav' + str(uav_id) + '/target_position', target_position, queue_size=10)
    
    

    while not rospy.is_shutdown():
        rospy.spin()