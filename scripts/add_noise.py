#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3 

class Noise:



    def __init__(self):

        self.i = 0
        self.std = 1.3
        self.noise_uav1_x = np.random.normal(0,self.std,1000)
        self.noise_uav1_y = np.random.normal(0,self.std,1000)
        self.noise_uav1_z = np.random.normal(0,self.std,1000)
        

        self.noise_uav2_x = np.random.normal(0,self.std,1000)
        self.noise_uav2_y = np.random.normal(0,self.std,1000)
        self.noise_uav2_z = np.random.normal(0,self.std,1000)

        self.noise_uav3_x = np.random.normal(0,self.std,1000)
        self.noise_uav3_y = np.random.normal(0,self.std,1000)
        self.noise_uav3_z = np.random.normal(0,self.std,1000)

        self.noise_uav4_x = np.random.normal(0,self.std,1000)
        self.noise_uav4_y = np.random.normal(0,self.std,1000)
        self.noise_uav4_z = np.random.normal(0,self.std,1000)

        self.values = []
        self.mean = 0
        self.std = 0


        pos_sub = rospy.Subscriber("/target_position", Vector3, self.addnoise_callback)



    def addnoise_callback(self, msg):

        self.values.append(msg.x + self.noise_uav1_x[self.i])
        pub_uav1.publish(msg.x + self.noise_uav1_x[self.i], msg.y + self.noise_uav1_y[self.i], msg.z + self.noise_uav1_z[self.i])
        pub_uav2.publish(msg.x + self.noise_uav2_x[self.i], msg.y + self.noise_uav2_y[self.i], msg.z + self.noise_uav2_z[self.i])
        pub_uav3.publish(msg.x + self.noise_uav3_x[self.i], msg.y + self.noise_uav3_y[self.i], msg.z + self.noise_uav3_y[self.i])
        pub_uav4.publish(msg.x + self.noise_uav4_x[self.i], msg.y + self.noise_uav4_y[self.i], msg.z + self.noise_uav4_y[self.i])
        #pub_uav2.publish(msg.x, msg.y, msg.z)
        #pub_uav3.publish(msg.x, msg.y, msg.z)


        self.mean = sum(self.values)/ len(self.values)
        self.std = np.std(self.values)
        self.i = self.i +1

        rospy.loginfo("Noise has been published")
        rospy.loginfo("Mean: %f--------Deviation: %f--------i: %f", self.mean, self.std, len(self.values))

        if self.i == 999:
            self.noise_uav1_x = np.random.normal(0,self.std,1000)
            self.noise_uav1_y = np.random.normal(0,self.std,1000)
            self.noise_uav1_z = np.random.normal(0,self.std,1000)

            self.noise_uav2_x = np.random.normal(0,self.std,1000)
            self.noise_uav2_y = np.random.normal(0,self.std,1000)
            self.noise_uav2_z = np.random.normal(0,self.std,1000)

            self.noise_uav3_x = np.random.normal(0,self.std,1000)
            self.noise_uav3_y = np.random.normal(0,self.std,1000)
            self.noise_uav3_z = np.random.normal(0,self.std,1000)

            self.noise_uav4_x = np.random.normal(0,self.std,1000)
            self.noise_uav4_y = np.random.normal(0,self.std,1000)
            self.noise_uav4_z = np.random.normal(0,self.std,1000)
            self.i = 0
            





if __name__ == "__main__":
    rospy.init_node("add_noise_py")

    pub_uav1 = rospy.Publisher("/uav1/target_position", Vector3, queue_size=10)
    pub_uav2 = rospy.Publisher("/uav2/target_position", Vector3, queue_size=10)
    pub_uav3 = rospy.Publisher("/uav3/target_position", Vector3, queue_size=10)
    pub_uav4 = rospy.Publisher("/uav4/target_position", Vector3, queue_size=10)


    rospy.loginfo("Node Noise has started")

    Noise()

    rospy.spin()