#! /usr/bin/env python

import rospy
import numpy as np
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position
from std_msgs.msg import Float64, Int64
from kalman_filter import KalmanFilter

class Fusion:
    def __init__(self, f, uav_id, uav_total):

        self.timestamp_ant = 0
        
        self.dt = 1 / f

        self.uav_id = uav_id
        self.uav_total = uav_total

        self.x0 = np.array([    [0],
                                 [0],
                                 [0],
                                 [0],
                                 [0]])

        # x ----Initial value for the state

        self.F = np.array([     [1, 0, self.dt, 0],
                                [0, 1, 0, self.dt],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1],])

        # F ----State trasition(A) (Dynamic Model)

        self.H = np.array([     [1, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0]])
        

        self.H_fuse = np.array([    [1, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0],
                                    [0, 0, 1, 0, 0],
                                    [0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 1]])

        # H ----Measurement function (y) 

        #self.kf.P *= 1

        # P ----Covariance matrix
        
        self.Q = np.array([     [0.001, 0, 0, 0, 0],
                                [0, 0.001, 0, 0, 0],
                                [0, 0, 0.001, 0, 0],
                                [0, 0, 0, 0.001, 0],
                                [0, 0, 0, 0, 0.001]])

        # Q ----Process Noise

        self.R = np.array([     [2, 0],
                                [0, 2]])
        
        self.R_fuse = np.array([    [0.5, 0, 0, 0, 0],
                                    [0, 0.5, 0, 0, 0],
                                    [0, 0, 0.1, 0, 0],
                                    [0, 0, 0, 0.1, 0],
                                    [0, 0, 0, 0, 0.1]])

        # R ----Measurement Noise


        rospy.Subscriber('/uav' + str(uav_id) + '/target_position', target_position, self.target_callback)
        
            
        for i in range(uav_total + 1):
            if (i != uav_id and i != 0):
                rospy.Subscriber('/uav' + str(i) + '/target_position_fuse', target_position_fuse, self.target_callback_fuse)
                

            


        self.kf = KalmanFilter(F = self.F, H = self.H, H_fuse = self.H_fuse , Q = self.Q , R = self.R, R_fuse = self.R_fuse, x0= self.x0, dt = self.dt)
        
    def predict(self):
        
        self.kf.predict_nonlinear()
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.theta = self.kf.x[2]
        state.v = self.kf.x[3]
        state.w = self.kf.x[4]
        state.timestamp = rospy.Time.now()

        #rospy.loginfo('Kalman ' + str(self.uav_id) + '---------Prediction was made')
        return state

        
    def target_callback(self, msg):
        
        measurment = np.array([[msg.x], [msg.y]])

        self.kf.update(measurment)
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.theta = self.kf.x[2]
        state.v = self.kf.x[3]
        state.w = self.kf.x[4]
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
            msg.theta = self.msg_ant.theta + (msg.theta - self.msg_ant.theta) / (time_now - self.timestamp_ant) * (timestamp - self.timestamp_ant)
            msg.v = self.msg_ant.v + (msg.v - self.msg_ant.v) / (time_now - self.timestamp_ant) * (timestamp - self.timestamp_ant)
            msg.w = self.msg_ant.w + (msg.w - self.msg_ant.w) / (time_now - self.timestamp_ant) * (timestamp - self.timestamp_ant)
        
        self.timestamp_ant = timestamp
        self.msg_ant = msg
        
        #pub_msg_correction.publish(msg)
        
        #rospy.loginfo('MSG correction %f, %f, %f, %f', msg.x, msg.y, msg.v_x, msg.v_y) 
        
        measurment = np.array([[msg.x], [msg.y], [msg.theta], [msg.v], [msg.w]])

        #rospy.loginfo('N %f', N)
        
        pub_samples.publish(N)
        
        self.kf.update_fuse(measurment)
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.theta = self.kf.x[2]
        state.v = self.kf.x[3]
        state.w = self.kf.x[4]
        state.timestamp = rospy.Time.now()
        #rospy.loginfo('Kalman %d ---------Update estimation was made', self.uav_id)
        





if __name__ == "__main__":


    rospy.init_node("kalman_filter")
    uav_id = rospy.get_param("~uav_id")
    uav_total = rospy.get_param("/total_uav")
    rospy.loginfo('Fusion Kalman filter %d start', uav_id)
        

    
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
        
