#! /usr/bin/env python3

import rospy
import numpy as np
from kf import KalmanFilter as kf
from kf import DelayKalmanFilter as dkf
from MARS_msgs.msg import TargetTelemetry



class Fusion:
    def __init__(self, f, uav_id, uav_total):
        
        dt = 1 / f

        H = np.array([      [1, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0]])
        
        H_fuse = H
        
        # H ----Measurement function (y)
        
        G = np.array([  [0.5 * (dt ** 2),   0,                  dt, 0, 0],
                        [0,                 0.5 * (dt ** 2),    0, dt, 0],
                        [0,                 0,                  0, 0, dt]])
    
        G = np.transpose(G)

        qv = 0.737433 ** 2
        qw = 1e-10 ** 2


        q = np.array([[qv, 0, 0],
                        [0, qv, 0],
                        [0, 0, qw]])


        Q = np.dot(np.dot(G, q),np.transpose(G))
        
        # Q ----Process Noise
        
        R = np.array([      [100, 0],
                            [0, 100]])
        
        # R ----Measurement Noise
        
        P = np.array([          [6.362137e-2, 0, 0, 0, 0],
                                [0, 6.362137e-2, 0, 0, 0],
                                [0, 0, 2.3496e-1, 0, 0],
                                [0, 0, 0, 2.3496e-1, 0],
                                [0, 0, 0, 0, 6.08288e-2]])
        
        # P ----Covariance matrix
        
        x0 = np.array([     [0],
                            [0],
                            [0],
                            [0],
                            [0]])
        
        # x ----Initial value for the state 


        rospy.Subscriber('/uav' + str(uav_id) + '/target_position_geolocation', TargetTelemetry, self.target_callback)
        
            
        for i in range(uav_total):
            if (i != uav_id):
                rospy.Subscriber('/uav' + str(i) + '/target_position_geolocation', TargetTelemetry, self.target_callback_fuse)
                

            


        self.kf = kf( H = H, H_fuse = H_fuse , Q = Q , R = R, x0= x0, dt = dt, aug= 3, P= P)
        
        self.kf = dkf(kf= self.kf, delay_strategy= 'augmented_state', centr= False)

        
    def predict(self):
        
        self.kf.predict_nonlinear()
        state = TargetTelemetry()
        state.x_pos = self.kf.kf.x[0]
        state.y_pos = self.kf.kf.x[1]
        state.vx = self.kf.kf.x[2]
        state.vy = self.kf.kf.x[3]
        state.omega = self.kf.kf.x[4]
        state.accel = 0
        state.vel = 0
        state.psi = 0
        state.timestamp = rospy.Time.now()

        #rospy.loginfo('Kalman ' + str(self.uav_id) + '---------Prediction was made')
        return state
        

        
    def target_callback(self, msg):
        measurment = np.array([[msg.x_pos], [msg.y_pos]])
        self.kf.update(measurment)



    def target_callback_fuse(self, msg):
        measurment = np.array([[msg.x_pos], [msg.y_pos], [msg.vx], [msg.vy], [msg.omega]])
        t_now = rospy.Time.now().to_nsec() * 1e-9
        t_z = msg.timestamp.to_nsec() * 1e-9
        self.kf.update_fuse(z= measurment, t_now= t_now, t_z= t_z)
        





if __name__ == "__main__":


    rospy.init_node("kalman_filter")
    uav_id = rospy.get_param("~uav_id")
    uav_total = rospy.get_param("/total_uav")
    rospy.loginfo('Fusion Kalman filter %d start', uav_id)
        
    

    
    pub_estimation = rospy.Publisher('target_position', TargetTelemetry, queue_size=1)
    
    
    f = 5
    
    #frequency of the Kalman filter
    
    
    ss = Fusion(f, uav_id, uav_total)

    rate = rospy.Rate(f) 


    while not rospy.is_shutdown(): 
        state = ss.predict()
        pub_estimation.publish(state)
        
        rate.sleep()
