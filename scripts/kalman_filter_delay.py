#! /usr/bin/env python

import rospy
import numpy as np
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position
from std_msgs.msg import Float64, Int64


class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, H_fuse = None, Q = None, R = None, R_fuse = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.H_fuse = H_fuse
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.R_fuse = np.eye(self.n) if R_fuse is None else R_fuse
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
             
    def update_fuse(self, z):
        y = z - np.dot(self.H_fuse, self.x)
        S = self.R_fuse + np.dot(self.H_fuse, np.dot(self.P, self.H_fuse.T))
        self.K = np.dot(np.dot(self.P, self.H_fuse.T), np.linalg.inv(S))
        self.x = self.x + np.dot(self.K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(self.K, self.H_fuse), self.P), 
        	(I - np.dot(self.K, self.H_fuse)).T) + np.dot(np.dot(self.K, self.R_fuse), self.K.T)
        

class Fusion:
    def __init__(self, f, uav_id, uav_total):

        dt = 1 / f
        self.timestamp_ant = 0
        
        self.uav_id = uav_id
        self.uav_total = uav_total

        #x0 = np.array([[0, 0, 2, 2]])

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
        
        self.Q = np.array([     [0.001, 0, 0, 0],
                                [0, 0.001, 0, 0],
                                [0, 0, 0.001, 0],
                                [0, 0, 0, 0.001]])

        # Q ----Process Noise

        self.R = np.array([     [2, 0],
                                [0, 2]])
        
        self.R_fuse = np.array([    [0.5, 0, 0, 0],
                                    [0, 0.5, 0, 0],
                                    [0, 0, 0.1, 0],
                                    [0, 0, 0, 0.1]])

        # R ----Measurement Noise


        rospy.Subscriber('/uav' + str(uav_id) + '/target_position', target_position, self.target_callback)
        
            
        for i in range(uav_total + 1):
            if (i != uav_id and i != 0):
                rospy.Subscriber('/uav' + str(i) + '/target_position_fuse', target_position_fuse, self.target_callback_fuse)
                

            


        self.kf = KalmanFilter(F = self.F, H = self.H, H_fuse = self.H_fuse , Q = self.Q , R = self.R, R_fuse = self.R_fuse)

        
    def predict(self):
        
        self.kf.predict()
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.v_x = self.kf.x[2]
        state.v_y = self.kf.x[3]
        state.timestamp = rospy.Time.now()
        predicts_mem.append([self.kf.x , state.timestamp.to_nsec() * 1e-9])

        #rospy.loginfo('Kalman ' + str(self.uav_id) + '---------Prediction was made')
        return state

        
    def target_callback(self, msg):
        
        measurment = np.array([[msg.x], [msg.y]])

        self.kf.update(measurment)
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.v_x = self.kf.x[2]
        state.v_y = self.kf.x[3]
        state.timestamp = rospy.Time.now()
        #rospy.loginfo('Kalman ' + str(self.uav_id) + '---------Update was made')


    def target_callback_fuse(self, msg):
        
        timestamp = msg.timestamp.to_nsec() * 1e-9
        time_now = rospy.Time.now().to_nsec() * 1e-9
        delay = time_now - timestamp
        
        #rospy.loginfo('Timestamp %f, T_now %f, Delay %f, timestamp_ant %f', timestamp, time_now, delay, self.timestamp_ant)
        
        pub_delay.publish(delay)
        
        N = round(delay * f)
        
        #rospy.loginfo('MSG  %f, %f, %f, %f', msg.x, msg.y, msg.v_x, msg.v_y)
        
        #pub_msg_true.publish(msg)
                        
        #msg.x = self.kf.x[0] + (msg.x - self.kf.x[0]) / (time_now - self.kf.time) * (timestamp - self.kf.time)
        #msg.y = self.kf.x[1] + (msg.y - self.kf.x[1]) / (time_now - self.kf.time) * (timestamp - self.kf.time)
        #msg.v_x = self.kf.x[2] + (msg.v_x - self.kf.x[2]) / (time_now - self.kf.time) * (timestamp - self.kf.time)
        #msg.v_y = self.kf.x[3] + (msg.v_x - self.kf.x[3]) / (time_now - self.kf.time) * (timestamp - self.kf.time)
        
        
        
        if (self.timestamp_ant > 0):
            msg.x = self.msg_ant.x + (msg.x - self.msg_ant.x) / (time_now - self.timestamp_ant) * (timestamp - self.timestamp_ant)
            msg.y = self.msg_ant.y + (msg.y - self.msg_ant.y) / (time_now - self.timestamp_ant) * (timestamp - self.timestamp_ant)
            msg.v_x = self.msg_ant.v_x + (msg.v_x - self.msg_ant.v_x) / (time_now - self.timestamp_ant) * (timestamp - self.timestamp_ant)
            msg.v_y = self.msg_ant.v_y + (msg.v_y - self.msg_ant.v_y) / (time_now - self.timestamp_ant) * (timestamp - self.timestamp_ant)
        
        self.timestamp_ant = timestamp
        self.msg_ant = msg
        
        #pub_msg_correction.publish(msg)
        
        #rospy.loginfo('MSG correction %f, %f, %f, %f', msg.x, msg.y, msg.v_x, msg.v_y) 
        
        measurment = np.array([[msg.x], [msg.y], [msg.v_x], [msg.v_y]])

        #rospy.loginfo('N %f', N)
        
        pub_samples.publish(N)
        
        self.kf.update_fuse(measurment)
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.v_x = self.kf.x[2]
        state.v_y = self.kf.x[3]
        state.timestamp = rospy.Time.now()
        #rospy.loginfo('Kalman %d ---------Update estimation was made', self.uav_id)
        





if __name__ == "__main__":


    rospy.init_node("kalman_filter")
    uav_id = rospy.get_param("~uav_id")
    uav_total = rospy.get_param("/total_uav")
    rospy.loginfo('Fusion Kalman filter %d start', uav_id)
        
    predicts_mem = []
    k_mem = []

    
    pub_estimation = rospy.Publisher('target_position_estimation', target_position_fuse, queue_size=1)
    
    pub_delay = rospy.Publisher('delay_estimation', Float64, queue_size=1)
    
    pub_samples = rospy.Publisher('samples_delay', Int64, queue_size=1)
    
    pub_msg_true = rospy.Publisher('msg_true', target_position_fuse, queue_size=1)
    
    pub_msg_correction = rospy.Publisher('msg_correction', target_position_fuse, queue_size=1)
    
    
    
    f = 20
    
    #frequency of the Kalman filter
    ss = Fusion(f, uav_id, uav_total)

    rate = rospy.Rate(f) 

    while not rospy.is_shutdown(): 
        state = ss.predict()
        pub_estimation.publish(state)
        rate.sleep()
        
