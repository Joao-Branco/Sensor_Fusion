#! /usr/bin/env python

import rospy
from sensor_fusion.msg import target_position_fuse

def target(self, x, y, z):
    self.x = x
    self.y = y

if __name__ == "__main__":
    rospy.init_node("target_py")
    rospy.loginfo("Node Target has started")


    target.x = 0.0 
    target.y = 0.0
    target.z = 0.0

    v = 1
    dt = 0.4

    pub = rospy.Publisher("/target_position", target_position_fuse, queue_size=10)



    rate = rospy.Rate(5)


    while not rospy.is_shutdown():
        rospy.loginfo("X: %f, Y: %f", target.x, target.y) 
        #target.x = target.x + v * dt
        pub.publish(target.x, target.y, 0 ,0)
        rate.sleep()
    
