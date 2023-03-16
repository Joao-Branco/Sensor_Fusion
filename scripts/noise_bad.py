#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3 


class Mean:

    def __init__(self):
        self.mean = 0
        self.values = []
        pos_sub = rospy.Subscriber("/target_position", Vector3, self.target_callback)



    def target_callback(self, msg):
        noise_uav_x = np.random.normal(0,1,3)
        noise_uav_y = np.random.normal(0,1,3)
        noise_uav_z = np.random.normal(0,1,3)

        pub_uav1.publish(msg.x + noise_uav_x[0], msg.y + noise_uav_y[0], msg.z + noise_uav_z[0])
        pub_uav2.publish(msg.x + noise_uav_x[1], msg.y + noise_uav_y[1], msg.z + noise_uav_z[1])
        #pub_uav3.publish(msg.x + noise_uav_x[2], msg.y + noise_uav_y[2], msg.z + noise_uav_z[2])
        #pub_uav2.publish(msg.x, msg.y, msg.z)
        pub_uav3.publish(msg.x, msg.y, msg.z)

        self.values.append(msg.x + noise_uav_x[0])
        self.mean = sum(self.values)/ len(self.values)



        rospy.loginfo("Noise has been published")
        rospy.loginfo("Mean: %f--------Sum: %f--------i: %f", self.mean, sum(self.values), len(self.values))

    


if __name__ == "__main__":
    rospy.init_node("add_noise_init_py")

    pub_uav1 = rospy.Publisher("/uav1/target_position", Vector3, queue_size=10)
    pub_uav2 = rospy.Publisher("/uav2/target_position", Vector3, queue_size=10)
    pub_uav3 = rospy.Publisher("/uav3/target_position", Vector3, queue_size=10)

    rospy.loginfo("Node Noise has started")



    Mean()
    rospy.spin()