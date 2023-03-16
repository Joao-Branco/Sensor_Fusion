#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3 

def target(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

if __name__ == "__main__":
    rospy.init_node("target_py")
    rospy.loginfo("Node Target has started")


    target.x = 0.0 
    target.y = 0.0
    target.z = 0.0

    v = 1
    dt = 0.4

    pub = rospy.Publisher("/target_position", Vector3, queue_size=10)



    rate = rospy.Rate(5)


    while not rospy.is_shutdown():
        rospy.loginfo("X: %f, Y: %f, Z: %f", target.x, target.y, target.z) 
        #target.x = target.x + v * dt
        pub.publish(target.x, target.y, target.z)
        rate.sleep()
    
