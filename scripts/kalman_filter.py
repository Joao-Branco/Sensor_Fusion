#! /usr/bin/env python

#import rospy
import numpy as np
#from sensor_fusion.msg import target_position_fuse
#from sensor_fusion.msg import target_position



def f_nonlinear(x, dt, aug):

    f = np.array([  [x[0,0] + x[3,0] * dt * np.cos(x[2,0])],
                    [x[1,0] + x[3,0] * dt * np.sin(x[2,0])],
                    [x[4,0] * dt + x[2,0]],
                    [x[3,0]],
                    [x[4,0]]])
    
    f_d = []
    for d in range(aug):
        f_d.append(np.array([  [x[0+d*5,0]],
                        [x[1+d*5,0]],
                        [x[2+d*5,0]],
                        [x[3+d*5,0]],
                        [x[4+d*5,0]]]))



    if aug > 0:
        f_d = np.vstack((f_d))
        F = np.vstack((f,f_d))
    else:
        F = f

    return F

def Jacobian(x, dt, aug):

    J = np.array([  [1, 0, -x[3,0] * dt *np.sin(x[2,0]),    dt *np.cos(x[2,0]),   0],
                    [0, 1, x[3,0] * dt *np.cos(x[2,0]),     dt *np.sin(x[2,0]),    0],
                    [0, 0, 1,                                       0,                              dt],
                    [0, 0, 0,                                       1,                              0],
                    [0, 0, 0,                                       0,                              1]])  


    if aug > 0:
        J_a = np.block([[np.block([J, np.zeros((5,aug*5))])], [np.block([np.identity(5*aug), np.zeros((5*aug,5))])]])  
    else:
        J_a = J 

    return J_a



class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, H_fuse = None, Q = None, R = None, R_fuse = None, R_delay = None, P = None, x0 = None, dt = None, aug = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = x0.shape[0]
        self.m = H.shape[1]
        self.dt = dt

        self.F = F
        self.H = H
        self.H_fuse = H_fuse
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.R_fuse = np.eye(self.n) if R_fuse is None else R_fuse
        self.R_delay = np.eye(self.n) if R_delay is None else R_delay
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        self.J = np.eye(self.n)
        self.aug = 0 if aug is None else aug


    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x
    
    def predict_nonlinear(self, u = 0):
        self.J =Jacobian(self.x, self.dt, self.aug)
        self.x = f_nonlinear(self.x, self.dt, self.aug)

        self.P = np.dot(np.dot(self.J, self.P), self.J.T) + self.Q
        return self.x

    def update(self, z):
        # if (self.aug > 0):
        #     state_z = z
        #     z = np.zeros(self.x.shape)
        #     np.put(z, [0, 1], state_z)
        self.y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        self.K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(self.K, self.y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(self.K, self.H), self.P), 
        	(I - np.dot(self.K, self.H)).T) + np.dot(np.dot(self.K, self.R), self.K.T)
             
    def update_fuse(self, z):
        self.y_fuse = z - np.dot(self.H_fuse, self.x)
        S = self.R_fuse + np.dot(self.H_fuse, np.dot(self.P, self.H_fuse.T))
        self.K_fuse = np.dot(np.dot(self.P, self.H_fuse.T), np.linalg.inv(S))
        self.x = self.x + np.dot(self.K_fuse, self.y_fuse)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(self.K_fuse, self.H_fuse), self.P), 
        	(I - np.dot(self.K_fuse, self.H_fuse)).T) + np.dot(np.dot(self.K_fuse, self.R_fuse), self.K_fuse.T)


class Fusion:
    def __init__(self, f, uav_id, uav_total):

        self.dt = 1 / f

        self.uav_id = uav_id
        self.uav_total = uav_total

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
                                 [0, 0, 0, 1]])
        

        # F ----State trasition(A) (Dynamic Model)

        self.H = np.array([     [1, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0]])

        # self.H = np.array([     [1, 0, 0, 0],
        #                         [0, 1, 0, 0]])
        

        self.H_fuse = np.array([    [1, 0, 0, 0, 0],
                                    [0, 1, 0, 0, 0],
                                    [0, 0, 1, 0, 0],
                                    [0, 0, 0, 1, 0],
                                    [0, 0, 0, 0, 1]])
        
        # self.H_fuse = np.array([    [1, 0, 0, 0],
        #                             [0, 1, 0, 0],
        #                             [0, 0, 1, 0],
        #                             [0, 0, 0, 1]])
        

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
        
        
        
        
        # self.R_fuse = np.array([    [0.5, 0, 0, 0],
        #                             [0, 0.5, 0, 0],
        #                             [0, 0, 0.1, 0],
        #                             [0, 0, 0, 0.1]])

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
        state.uavid = uav_id
        # state.v_x = self.kf.x[2]
        # state.v_y = self.kf.x[3]
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
        # state.v_x = self.kf.x[2]
        # state.v_y = self.kf.x[3]
        state.timestamp = rospy.Time.now()
        #rospy.loginfo('Kalman ' + str(self.uav_id) + '---------Update was made')


    def target_callback_fuse(self, msg):
        measurment = np.array([[msg.x], [msg.y], [msg.theta], [msg.v], [msg.w]])
        self.kf.update_fuse(measurment)
        state = target_position_fuse()
        state.x = self.kf.x[0]
        state.y = self.kf.x[1]
        state.theta = self.kf.x[2]
        state.v = self.kf.x[3]
        state.w = self.kf.x[4]
        # state.v_x = self.kf.x[2]
        # state.v_y = self.kf.x[3]
        state.timestamp = rospy.Time.now()
        #rospy.loginfo('Kalman %d ---------Update estimation was made', self.uav_id)
        





if __name__ == "__main__":


    rospy.init_node("kalman_filter")
    uav_id = rospy.get_param("~uav_id")
    uav_total = rospy.get_param("/total_uav")
    rospy.loginfo('Fusion Kalman filter %d start', uav_id)
        
    

    
    pub_estimation = rospy.Publisher('target_position_estimation', target_position_fuse, queue_size=1)
    
    
    f = 20
    
    #frequency of the Kalman filter
    
    
    ss = Fusion(f, uav_id, uav_total)

    rate = rospy.Rate(f) 
    time_ref=0

    while not rospy.is_shutdown(): 
        state = ss.predict()
        pub_estimation.publish(state)
        
        rate.sleep()
