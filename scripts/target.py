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
    f = 100

    

    rate = rospy.Rate(f)
    
    t_0 = rospy.Time.now()
    

    while not rospy.is_shutdown():
        
        # = 0
        t = rospy.Time.now()
        
        t = (t-t_0).to_sec()
        
        
        rospy.loginfo("X: %f, Y: %f", target.x, target.y) 
        
        #Target not moving
        #target.x = 0 * t
        #target.y = 0 * t
        
        #Trget Moving with constant speed
        target.x = 2 * t
        target.y = 2 * t
        
        pub.publish(target.x, target.y)  #, rospy.Time.now())

        
        rospy.loginfo("Target has been publish")
        


        rate.sleep()
    
