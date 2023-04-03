#! /usr/bin/env python

import rospy
import numpy as np
from sensor_fusion.msg import target_position_fuse

class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z, R, H):
        y = z - np.dot(H, self.x)
        S = R + np.dot(H, np.dot(self.P, H.T))
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, H), self.P), 
        	(I - np.dot(K, H)).T) + np.dot(np.dot(K, R), K.T)

class Fusion:
    def __init__(self, f):

        dt = 1 / f
        

        #self.x = np.array([0, 0, 0, 0, 0, 0])

        # x ----Initial value for the state

        self.F = np.array([     [1, 0, dt, 0],
                                [0, 1, 0, dt],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1],])

        # F ----State trasition(A) (Dynamic Model)

        self.H = np.array([     [1, 0, 0, 0],
                                [0, 1, 0, 0]])
        

        self.H_fuse = np.array([    [1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])

        # H ----Measurement function (y) 

        #self.kf.P *= 1

        # P ----Covariance matrix
        
        self.Q = np.array([     [0.1, 0, 0, 0],
                                [0, 0.1, 0, 0],
                                [0, 0, 0.1, 0],
                                [0, 0, 0, 0.1]])

        # Q ----Process Noise

        self.R = np.array([     [0.001, 0],
                                [0, 0.001]])
        
        self.R_fuse = np.array([    [0.001, 0, 0, 0],
                                    [0, 0.001, 0, 0],
                                    [0, 0, 0.1, 0],
                                    [0, 0, 0, 0.1]])

        # R ----Measurement Noise

        sub_UAV1 = rospy.Subscriber("/uav1/target_position", target_position_fuse, self.target_callback_uav1)

        sub_UAV2_fuse = rospy.Subscriber("/uav2/target_position_fuse", target_position_fuse, self.target_callback_uav2_fuse)

        self.kf = KalmanFilter(F = self.F, H = self.H , Q = self.Q , R = self.R)

        
    def predict(self):
        self.kf.predict()
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.v_x = self.kf.x[2]
        state.v_y = self.kf.x[3]
        pub_UAV1.publish(state)
        rospy.loginfo("Kalman 1 ---------Prediction was made")

        
    def target_callback_uav1(self, msg):
        measurment = np.array([[msg.x], [msg.y]])

        self.kf.update(measurment, self.R, self.H)
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.v_x = self.kf.x[2]
        state.v_y = self.kf.x[3]
        pub_UAV1.publish(state)
        rospy.loginfo("Kalman 1 ---------Update was made 1")


    def target_callback_uav2_fuse(self, msg):
        measurment = np.array([[msg.x], [msg.y], [msg.v_x], [msg.v_y]])
        


        self.kf.update(measurment, self.R_fuse, self.H_fuse)
        rospy.loginfo("Kalman 1 ---------Update fuse was made 2")



if __name__ == "__main__":


    rospy.init_node("kalman_filter_1")
    rospy.loginfo("Fusion Kalman filter 1 start")

    pub_UAV1 = rospy.Publisher("/uav1/target_position_fuse", target_position_fuse, queue_size=10)
    
    
    f = 40
    
    #frequency of the Kalman filter
    ss = Fusion(f)

    rate = rospy.Rate(f) 

    while not rospy.is_shutdown(): 
        ss.predict()
        rate.sleep()