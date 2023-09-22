#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point




if __name__ == "__main__":
    rospy.init_node("target_py")
    rospy.loginfo("Node Target has started")


    pub = rospy.Publisher("/target_position", Point, queue_size=10)
    
    
    # definition of parameters of target
    # x,y - starting position (m)   v - starting velocity (m/s)

    
    x = 0.0 
    y = 0.0


    # frenquency of publication of position
    f = 30

    #10 - 25 Hz (Computer Vision)

    

    rate = rospy.Rate(f)
    
    t_0 = rospy.Time.now()
    
    #t=0

    while not rospy.is_shutdown():
        
        
        
        t = rospy.Time.now()
        
        t = (t-t_0).to_sec()
        
        t = t + 1/f
        
        
        
        #Target not moving
        target.x = 0 * t
        target.y = 0 * t
        target.z = 0
        

    
       
        
        pub.publish(target)

        
        rospy.loginfo("Target has been publish")
        


        rate.sleep()
    

