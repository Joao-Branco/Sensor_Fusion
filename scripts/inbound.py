#! /usr/bin/env python

import rospy
import numpy as np
from kf import KalmanFilter as kf
from kf import DelayKalmanFilter as dkf
from sensor_fusion.msg import target_position_fuse
from sensor_fusion.msg import target_position



class Fusion:
    def __init__(self, f, uav_id, uav_total):
        
        dt = i / f

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
        
        R = np.array([      [10, 0],
                            [0, 10]])
        
        # R ----Measurement Noise
        
        P = np.array([          [6.362137e-2, 0, 0, 0, 0],
                                [0, 6.362137e-2, 0, 0, 0],
                                [0, 0, 2.3496e-1, 0, 0],
                                [0, 0, 0, 2.3496e-1, 0],
                                [0, 0, 0, 0, 6.08288e-2]])
        
        # P ----Covariance matrix
        
        x0 = np.array([     [state[0,0]],
                            [state[1,0]],
                            [state[2,0]],
                            [state[3,0]],
                            [0]])
        
        # x ----Initial value for the state 


        rospy.Subscriber('/uav' + str(uav_id) + '/target_position_geolocation', target_position, self.target_callback)
        
            
        for i in range(uav_total + 1):
            if (i != uav_id and i != 0):
                rospy.Subscriber('/uav' + str(i) + '/target_position_geolocation', target_position_fuse, self.target_callback_fuse)
                

            


        self.kf = kf( H = H, H_fuse = H_fuse , Q = Q , R = R, x0= x0, dt = dt, aug= 3)
        
        self.kf = dkf(kf= self.kf, delay_strategy= 'Augmented_state', centr= False)

        
    def predict(self):
        
        self.kf.predict_nonlinear()
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.theta = self.kf.x[2]
        state.v = self.kf.x[3]
        state.w = self.kf.x[4]
        state.uavid = uav_id
        state.timestamp = rospy.Time.now()

        #rospy.loginfo('Kalman ' + str(self.uav_id) + '---------Prediction was made')
        return state
        

        
    def target_callback(self, msg):
        measurment = np.array([[msg.x], [msg.y]])
        self.kf.update(measurment)



    def target_callback_fuse(self, msg):
        measurment = np.array([[msg.x], [msg.y], [msg.theta], [msg.v], [msg.w]])
        self.kf.update_fuse(measurment)
        





if __name__ == "__main__":


    rospy.init_node("kalman_filter")
    uav_id = rospy.get_param("~uav_id")
    uav_total = rospy.get_param("/total_uav")
    rospy.loginfo('Fusion Kalman filter %d start', uav_id)
        
    

    
    pub_estimation = rospy.Publisher('target_position', target_position_fuse, queue_size=1)
    
    
    f = 20
    
    #frequency of the Kalman filter
    
    
    ss = Fusion(f, uav_id, uav_total)

    rate = rospy.Rate(f) 


    while not rospy.is_shutdown(): 
        state = ss.predict()
        pub_estimation.publish(state)
        
        rate.sleep()
