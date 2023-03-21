#! /usr/bin/env python

import rospy
import math
import numpy as np
from sensor_fusion.msg import target_position_fuse


def target(self, x, y, v_x, v_y):
    self.x = x
    self.y = y
    self.v_x = v_x
    self.v_y = v_y

if __name__ == "__main__":
    rospy.init_node("target_py")
    rospy.loginfo("Node Target has started")


    pub = rospy.Publisher("/target_position", target_position_fuse, queue_size=10)
    
    pub_uav1 = rospy.Publisher("/uav1/target_position", target_position_fuse, queue_size=10)
    pub_uav2 = rospy.Publisher("/uav2/target_position", target_position_fuse, queue_size=10)
    pub_uav3 = rospy.Publisher("/uav3/target_position", target_position_fuse, queue_size=10)
    
    
    

    
    target.x = 0.0 
    target.y = 0.0
    target.v_x = 0.0
    target.v_y = 0.0

    v = 1
    f = 5
    dt = 1/f
    t = 0
    std = 0.5
    mean = 0
    i = 0
    
    noise_uav1_x = np.random.normal(mean,std,1000)
    noise_uav1_y = np.random.normal(mean,std,1000)        
    
    noise_uav2_x = np.random.normal(mean,std,1000)
    noise_uav2_y = np.random.normal(mean,std,1000)

    noise_uav3_x = np.random.normal(mean,std,1000)
    noise_uav3_y = np.random.normal(mean,std,1000)
    



    rate = rospy.Rate(f)


    while not rospy.is_shutdown():
        rospy.loginfo("X: %f, Y: %f", target.x, target.y) 
        target.x = 10 * math.sin(2*t)
        t = t + dt
        pub.publish(target.x, target.y, 0 ,0)
        if (i == 999):
            i = 0
            
            
        pub_uav1.publish(target.x + noise_uav1_x[i], target.y + noise_uav1_y[i], target.v_x, target.v_y)
        pub_uav2.publish(target.x + noise_uav2_x[i], target.y + noise_uav2_y[i], target.v_x, target.v_y)
        pub_uav3.publish(target.x + noise_uav3_x[i], target.y + noise_uav3_y[i], target.v_x, target.v_y)
        
        i = i + 1
        
        rospy.loginfo("Noise has been published")
        


        rate.sleep()
    
