#! /usr/bin/env python

import rospy
import numpy as np
from sensor_fusion.msg import target_position_fuse, delay_corr , target_position
from std_msgs.msg import Float64, Int64
from kalman_filter import KalmanFilter

class Fusion:
    def __init__(self, f, uav_id, uav_total):

        self.msg_mem = {}
        
        self.dt = 1 / f

        self.uav_id = uav_id
        self.uav_total = uav_total

        self.threshold_delay = 5
        self.min_delay = 0.0001

        self.x0 = np.array([    [5],
                                [0],
                                [0],
                                [0],
                                [0]])
        
        # self.x0 = np.array([    [5],
        #                          [0],
        #                          [0],
        #                          [0]])

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
        
        self.Q = np.array([     [0.5, 0, 0, 0, 0],
                                [0, 0.5, 0, 0, 0],
                                [0, 0, 0.1, 0, 0],
                                [0, 0, 0, 0.5, 0],
                                [0, 0, 0, 0, 0.1]])

        # self.Q = np.array([     [0.001, 0, 0, 0],
        #                         [0, 0.001, 0, 0],
        #                         [0, 0, 0.001, 0],
        #                         [0, 0, 0, 0.001]])

        # Q ----Process Noise

        self.R = np.array([     [2, 0],
                                [0, 2]])
        
        self.R_fuse = np.array([    [0.5, 0, 0, 0, 0],
                                    [0, 0.5, 0, 0, 0],
                                    [0, 0, 0.1, 0, 0],
                                    [0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 0.1]])
        
        self.R_delay = np.array([   [2, 0],
                                    [0, 2]])
        
        # self.R_fuse = np.array([    [0.5, 0, 0, 0],
        #                             [0, 0.5, 0, 0],
        #                             [0, 0, 0.1, 0],
        #                             [0, 0, 0, 0.1]])

        # R ----Measurement Noise


        rospy.Subscriber('/uav' + str(uav_id) + '/target_position', target_position, self.target_callback)
        
            
        for i in range(uav_total + 1):
            if (i != uav_id and i != 0):
                rospy.Subscriber('/uav' + str(i) + '/target_position_fuse', target_position_fuse, self.target_callback_fuse)
                

            


        self.kf = KalmanFilter(F = self.F, H = self.H, H_fuse = self.H_fuse , Q = self.Q , R = self.R, R_fuse = self.R_fuse, R_delay = self.R_delay, x0= self.x0, dt = self.dt)
        
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

        if (self.msg_mem.get(msg.uavid) != None):
            dv_x = abs(msg.v * np.cos(msg.theta) - self.msg_mem[msg.uavid].v * np.cos(self.msg_mem[msg.uavid].theta))
            dv_y = abs(msg.v * np.sin(msg.theta) - self.msg_mem[msg.uavid].v * np.sin(self.msg_mem[msg.uavid].theta))

        

        measurment_int = target_position_fuse()

        pub_delay.publish(delay)
    
        
        if (self.msg_mem.get(msg.uavid) != None and ( delay < self.threshold_delay)):
            dt = timestamp - self.msg_mem[msg.uavid].timestamp.to_nsec() * 1e-9
            if dv_x < 0.0001:
                measurment_int.x = msg.x + msg.v * np.cos(msg.theta) * delay
            if dv_y < 0.0001:   
                measurment_int.y = msg.y + msg.v * np.sin(msg.theta) * delay

            #rospy.loginfo('kalman id  %i, id message %i, old message id %i, timestamp %f, self.timestamp_mem % f, declive %f, dx %f, dt %f, interpolation %f, observation % f, delay %f, ', uav_id, msg.uavid, self.msg_mem[msg.uavid].uavid, timestamp, self.msg_mem[msg.uavid].timestamp.to_nsec() * 1e-9, (msg.x - self.msg_mem[msg.uavid].x) / (timestamp - self.msg_mem[msg.uavid].timestamp.to_nsec() * 1e-9), (msg.x - self.msg_mem[msg.uavid].x), (timestamp - self.msg_mem[msg.uavid].timestamp.to_nsec() * 1e-9),  measurment_int.x, msg.x, delay)



            #measurment_int.theta = msg.theta + (msg.theta - self.msg_mem[msg.uavid].theta) / dt * delay
            #measurment_int.x = msg.x + (msg.x - self.msg_mem[msg.uavid].x) /  dt * delay
            #measurment_int.y = msg.y + (msg.y - self.msg_mem[msg.uavid].y) /  dt * delay
            #measurment_int.x = msg.x + msg.v * np.cos(msg.theta) * delay
            #measurment_int.y = msg.y + msg.v * np.sin(msg.theta) * delay
            #measurment_int.v = msg.v + (msg.v - self.msg_mem[msg.uavid].v) /  dt * delay
            #measurment_int.w = msg.w + (msg.w - self.msg_mem[msg.uavid].w) /  dt * delay

            self.msg_mem[msg.uavid] = msg


            pub_interpolation.publish(msg, measurment_int)

            #measurment = np.array([[measurment_int.x], [measurment_int.y]])
            measurment = np.array([[msg.x], [msg.y], [msg.theta], [msg.v], [msg.w]])


            self.kf.update_fuse(measurment)
            state = target_position_fuse()
            state.x = self.kf.x[0]
            state.y = self.kf.x[1]
            state.theta = self.kf.x[2]
            state.v = self.kf.x[3]
            state.w = self.kf.x[4]
            state.timestamp = rospy.Time.now()

        if(self.msg_mem.get(msg.uavid) == None):

            self.msg_mem[msg.uavid] = msg

            measurment = np.array([[msg.x], [msg.y], [msg.theta], [msg.v], [msg.w]])

            self.kf.update_fuse(measurment)
            state = target_position_fuse()
            state.x = self.kf.x[0]
            state.y = self.kf.x[1]
            state.theta = self.kf.x[2]
            state.v = self.kf.x[3]
            state.w = self.kf.x[4]
            state.timestamp = rospy.Time.now()
   
        if (delay > self.threshold_delay):
            pass

     
    

        





if __name__ == "__main__":


    rospy.init_node("kalman_filter")
    uav_id = rospy.get_param("~uav_id")
    uav_total = rospy.get_param("/total_uav")
    rospy.loginfo('Fusion Kalman filter %d start', uav_id)
        

    
    pub_estimation = rospy.Publisher('target_position_estimation', target_position_fuse, queue_size=1)
    
    pub_delay = rospy.Publisher('delay_estimation', Float64, queue_size=1)
    
    
    pub_interpolation = rospy.Publisher('interpolation', delay_corr, queue_size=1)
    
    
    
    
    f = 20
    
    #frequency of the Kalman filter
    ss = Fusion(f, uav_id, uav_total)

    rate = rospy.Rate(f) 

    while not rospy.is_shutdown(): 
        state = ss.predict()
        pub_estimation.publish(state)
        rate.sleep()
        
