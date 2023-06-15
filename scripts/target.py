#! /usr/bin/env python

import rospy
import math
import numpy as np
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position


def target(self, x, y, v_x, v_y):
    self.x = x
    self.y = y
    self.v_x = v_x
    self.v_y = v_y

if __name__ == "__main__":
    rospy.init_node("target_py")
    rospy.loginfo("Node Target has started")


    pub = rospy.Publisher("/target_position", target_position, queue_size=10)
    
    
    # definition of parameters of target
    # x,y - starting position (m)   v_x,v_y - starting velocity (m/s)

    
    target.x = 0.0 
    target.y = 0.0
    target.v_x = 0.0
    target.v_y = 0.0

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
        
        
        rospy.loginfo("X: %f, Y: %f", target.x, target.y) 
        
        #Target not moving
        #target.x = 0 * t
        #target.y = 0 * t
        
        #Target Moving with constant speed
        #target.x = 2 * t 
        #target.v_x = 2

        #target.y = 2 * t 
        #target.v_y = 2
        
        #Target Moving with aceleration not constant
        
        #target.x = 5 * math.cos( 0.09 * t) 
        #target.v_x =  -0.45 * math.sin(0.09 * t)
        
        #target.y = 5 * math.sin( 0.09 * t)
        #target.v_y = 0.45 * math.cos(0.09 * t)

        #Target moving sinusoidal 
        
        target.x = 5 * math.cos( 0.2 * t) 
        target.v_x =  -0.1 * math.sin(0.2 * t)

        target.y = 1 * t
        target.v_y = 1
        
        
        pub.publish(target.x, target.y, target.v_x, target.v_y, rospy.Time.now())

        
        rospy.loginfo("Target has been publish")
        


        rate.sleep()
    
