#! /usr/bin/env python

import rospy
import math
import numpy as np
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position




if __name__ == "__main__":
    rospy.init_node("target_py")
    rospy.loginfo("Node Target has started")


    pub = rospy.Publisher("/target_position", target_position_fuse, queue_size=10)
    
    
    # definition of parameters of target
    # x,y - starting position (m)   v - starting velocity (m/s)

    
    x = 0.0 
    y = 0.0
    v_x = 0.0
    v_y = 0.0
    v = 0
    w = 0


    # frenquency of publication of position
    f = 10

    #10 - 25 Hz (Computer Vision)

    

    rate = rospy.Rate(f)
    
    t_0 = rospy.Time.now()
    
    #t=0

    while not rospy.is_shutdown():
        
        
        
        t = rospy.Time.now()
        
        t = (t-t_0).to_sec()
        
        t = t + 1/f
        
        
        
        #Target not moving
        #target.x = 0 * t
        #target.y = 0 * t
        
        #Target Moving with constant speed
        # w = 0
        # theta = w * t
        # v_x =  2
        # v_y =  2 
        # v = np.sqrt(v_x ** 2 + v_y ** 2)
        # x =  v_x * t
        # y = v_y * t
        
        #Target Moving with aceleration not constant

        # w = 0.09
        # theta = w * t
        # v_x = - 0.45 * np.sin(theta)
        # v_y =  0.45 * np.cos(theta) 
        # v = np.sqrt(v_x ** 2 + v_y ** 2)
        # x =  5 * np.cos(theta)
        # y = 5 * np.sin(theta)

        #Target moving sinusoidal 
        
        w = 0.09
        theta = w * t
        v_x = - 0.45 * np.sin(theta)
        v_y = 2
        v = np.sqrt(v_x ** 2 + v_y ** 2)
        x =  5  * np.cos(theta)
        y = v_y * t
    
        
        rospy.loginfo("X: %f, Y: %f", x, y) 
        
        pub.publish(x, y, theta, v, w, 0, rospy.Time.now())

        
        rospy.loginfo("Target has been publish")
        


        rate.sleep()
    
