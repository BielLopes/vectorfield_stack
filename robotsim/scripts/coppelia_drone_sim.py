#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage

from pyquaternion import Quaternion as Quaternion_lib
from math import cos, sin, sqrt, pi
import numpy as np
import zmqRemoteApi
#import scipy as sp
#import scipy.spatial



class drone_node(object):
    """
    Navigation control using Action Server
    """


    def __init__(self):

        self.freq = 100.0  # Frequency to simulate the simple drone robot

        # Rotation Quaternions
        self.orientation_a = Quaternion_lib([1, 0, 0, 0])
        self.orientation_d = Quaternion_lib([1, 0, 0, 0])

        # self.vel = [0.0, 0.0] #vx and wz
        self.tau = 9.81
        self.omega = [0.0, 0.0, 0.0]

        self.state =  [0.0, 0.0, 0.0] #x, y, psi
        self.robot_radius =  0.1

        self.obtscles_pos = []
        self.obtscles_r = []

        # Coppelia config params
        self.coppelia_const_velocity = 0.1
        self.sim = None
        try : 
            self.sim = zmqRemoteApi.RemoteAPIClient().getObject('sim')
            print("Connected to simulation")
        except :
            print("Could not connect to the simulation.")

        self.base = self.sim.getObject('/Quadcopter/base')
        self.quadcopter = self.sim.getObject('/Quadcopter')

        # publishers
        # self.pub_pose = None
        # self.pub_odom = None
        # self.pub_rviz_robot = None
        # self.pub_rviz_obst = None



        self.init_node()


    def quat_derivative(self, q, w):
        # Unit quaternion to rotation matrix
        # w x y z
        #velocity w in the world frame

        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        f = [0,0,0,0]
        f[0] = -0.5*(w[0]*qx+w[1]*qy+w[2]*qz)
        f[1] = 0.5*(w[0]*qw+w[1]*qz-w[2]*qy)
        f[2] = 0.5*(w[1]*qw+w[2]*qx-w[0]*qz)
        f[3] = 0.5*(w[2]*qw+w[0]*qy-w[1]*qx)

        return f


    # Unit quaternion to rotation matrix
    def quat2rotm(self,q):
        # w x y z

        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        Rot = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
               [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
               [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]]; #this was checked on matlab
               
        return Rot



    def lin(self, u):

        v = []
        for k in range(len(u)):
            v.append(u[k][0])
        return [v]



    def col(self, u):

        v = []
        for k in range(len(u)):
            v.append([u[k]])
        return v


    def normalize(self, u):

        n = 0
        for k in range(len(u)):
            n = n + u[k]**2
        n = sqrt(n)

        v = []
        for k in range(len(u)):
            v.append(u[k]/n)

        return v


    def run(self):
        """Execute the controller loop
        """
        rate = rospy.Rate(self.freq)


        pose_msg = Pose()
        odom_msg = Odometry()
        point_msg = Point()
        point_msg2 = Point()


        count1 = 0

        count2 = 0

        t_last = rospy.Time.now().to_sec()
        t_0 = rospy.Time.now().to_sec()
        count0 = 0


        time_now = rospy.Time.now().to_sec()

        count_freq = 0
        while not rospy.is_shutdown():

            time_last = time_now
            time_now = rospy.Time.now().to_sec()
            time_step = time_now-time_last

            #time_step = 1.0/self.freq

            # if ((rospy.Time.now().to_sec() - t_last) < 1.0/self.freq):
            #     print ("a: %f" % (rospy.Time.now().to_sec() - t_last))
            #     continue
            # print ("b")
            # t_last = rospy.Time.now().to_sec()

            # count0 = count0 + 1
            # print("\33[96mfreq: %f\33[0m" % ((count0)/(rospy.Time.now().to_sec()-t_0)))

            # t = rospy.Time.now().to_sec() - t_0
            # if(t < count_freq*(1.0/self.freq)):
            #     # print ("\ta")
            #     continue
            # count_freq = count_freq + 1
            # print ("b")
            # print (t)

            pos = [self.state[0], self.state[1], self.state[2]]
            quat_bw = [self.state[3], self.state[4], self.state[5], self.state[6]]
            vel_b = [self.state[7], self.state[8], self.state[9]]

            R_bw = self.quat2rotm(quat_bw)

            vel_w = np.matrix(R_bw)*(np.matrix(vel_b).transpose())
            vel_w = vel_w.transpose().tolist()[0]



            omega_b = [self.omega[0], self.omega[1], self.omega[2]]
            # omega_w = self.lin(self.multiply(R_bw,self.col(omega_b)))[0]
            omega_w = np.matrix(R_bw)*(np.matrix(omega_b).transpose())
            omega_w = omega_w.transpose().tolist()[0]
            # print(omega_w)


            # print (np.matrix(R_bw))
            # print (np.matrix(self.col(omega_b)))
            # print (np.matrix(omega_w))


            # print(np.matrix(R_bw).dot(np.matrix(omega_b).transpose()))
            # omega_w = omega_w.transpose().tolist()[0]

            quat_dot = self.quat_derivative(quat_bw, omega_w)


            # # A = [[1,2,3],[4,5,6],[7,8,9]]
            # # B = [[1,2],[1,2],[1,2]]
            # # C = self.multiply(A,B)
            # # print(np.matrix(A))
            # # print(np.matrix(B))
            # # print(np.matrix(C))

            # # print("\33[96momega_b = [%f, %f, %f]\33[0m" % (self.omega[0],self.omega[1],self.omega[2]))
            # # print("\33[96momega_w = [%f, %f, %f]\33[0m" % (omega_w[0],omega_w[1],omega_w[2]))
            # # print("\33[96mquat_dot = [%f, %f, %f, %f]\33[0m" % (quat_dot[0],quat_dot[1],quat_dot[2],quat_dot[3]))
            # # print("")

            g_vec_body = np.matrix(R_bw).transpose()*(np.matrix([0,0,9.81]).transpose())
            g_vec_body = g_vec_body.transpose().tolist()[0]

            F_drag = [-0.01*vel_b[0], -0.01*vel_b[1], -0.01*vel_b[2]]

            tau_vec = [0, 0, self.robot_m*self.tau]

            # print(g_vec_body)
            acc = [0, 0, 0]
            acc[0] = tau_vec[0]/self.robot_m - g_vec_body[0] + F_drag[0]/self.robot_m
            acc[1] = tau_vec[1]/self.robot_m - g_vec_body[1] + F_drag[1]/self.robot_m
            acc[2] = tau_vec[2]/self.robot_m - g_vec_body[2] + F_drag[2]/self.robot_m

            # print(acc)





            # #Model integration
            # self.state[0] = self.state[0] + vel_w[0]*time_step
            # self.state[1] = self.state[1] + vel_w[1]*time_step
            # self.state[2] = self.state[2] + vel_w[2]*time_step

            # # self.state[3] = self.state[3] + quat_dot[0]*(1.0/self.freq)
            # # self.state[4] = self.state[4] + quat_dot[1]*(1.0/self.freq)
            # # self.state[5] = self.state[5] + quat_dot[2]*(1.0/self.freq)
            # # self.state[6] = self.state[6] + quat_dot[3]*(1.0/self.freq)
            # q_new = [0,0,0,0]
            # q_new[0] = self.state[3] + quat_dot[0]*time_step
            # q_new[1] = self.state[4] + quat_dot[1]*time_step
            # q_new[2] = self.state[5] + quat_dot[2]*time_step
            # q_new[3] = self.state[6] + quat_dot[3]*time_step
            # q_new = self.normalize(q_new)
            # self.state[3] = q_new[0]
            # self.state[4] = q_new[1]
            # self.state[5] = q_new[2]
            # self.state[6] = q_new[3]
            
            self.state[7] = self.state[7] + acc[0]*time_step
            self.state[8] = self.state[8] + acc[1]*time_step
            self.state[9] = self.state[9] + acc[2]*time_step

            velocity_x = self.state[7] * self.coppelia_const_velocity
            velocity_y = self.state[8] * self.coppelia_const_velocity
            velocity_z = self.state[9] * self.coppelia_const_velocity


            # position = self.sim.getObjectPosition(self.quadcopter, -1)
            # orientation = self.sim.getObjectOrientation(self.quadcopter, -1)
            # orientation.append(sqrt(orientation[0]**2 + orientation[1]**2 + orientation[2]**2))
            
            # self.state[0] = position[0]
            # self.state[1] = position[1]
            # self.state[2] = position[2]

            # self.state[3] = orientation[3]
            # self.state[4] = orientation[0]
            # self.state[5] = orientation[1]
            # self.state[6] = orientation[2]


            #Publis robots pose
            # pose_msg.position.x = self.state[0]
            # pose_msg.position.y = self.state[1]
            # pose_msg.position.z = self.state[2]
            # pose_msg.orientation.x = self.state[4]
            # pose_msg.orientation.y = self.state[5]
            # pose_msg.orientation.z = self.state[6]
            # pose_msg.orientation.w = self.state[3]
            
            # self.pub_pose.publish(pose_msg)



            #Publish robots odomtry (with velocity)
            # odom_msg.header.stamp = rospy.Time.now()
            # odom_msg.header.frame_id = "drone"
            # odom_msg.child_frame_id = "world"
            # odom_msg.pose.pose.position.x = self.state[0]
            # odom_msg.pose.pose.position.y = self.state[1]
            # odom_msg.pose.pose.position.z = self.state[2]
            # odom_msg.pose.pose.orientation.x = self.state[4]
            # odom_msg.pose.pose.orientation.y = self.state[5]
            # odom_msg.pose.pose.orientation.z = self.state[6]
            # odom_msg.pose.pose.orientation.w = self.state[3]

            # odom_msg.twist.twist.linear.x = self.state[7]
            # odom_msg.twist.twist.linear.y = self.state[8]
            # odom_msg.twist.twist.linear.z = self.state[9]
            # odom_msg.twist.twist.angular.x = self.omega[0]
            # odom_msg.twist.twist.angular.y = self.omega[1]
            # odom_msg.twist.twist.angular.z = self.omega[2]
            
            # self.pub_odom.publish(odom_msg)



            # count2 = count2 + 1
            # if (count2 == 4):
            #     count2 = 0

            #     #Compute closest point - with respect to the world frame
            #     n_obst = min([len(self.obtscles_r), len(self.obtscles_pos)])
            #     D_close = float("inf")
            #     o_close = 0
            #     for o in range(n_obst):
            #         Dvec = [self.state[0]-self.obtscles_pos[o][0], self.state[1]-self.obtscles_pos[o][1], self.state[2]-self.obtscles_pos[o][2]]
            #         D = sqrt(Dvec[0]**2 + Dvec[1]**2 + Dvec[2]**2) - self.obtscles_r[o]
            #         if (D<D_close):
            #             o_close = o
            #             D_close = D

            #     # Publish vector
            #     D_vec_close = [self.obtscles_pos[o_close][0]-self.state[0], self.obtscles_pos[o_close][1]-self.state[1], self.obtscles_pos[o_close][2]-self.state[2]]
            #     D = sqrt(D_vec_close[0]**2 + D_vec_close[1]**2 + D_vec_close[2]**2)
            #     D_hat = [D_vec_close[0]/(D+1e-8), D_vec_close[1]/(D+1e-8), D_vec_close[2]/(D+1e-8)]
            #     D = D - self.obtscles_r[o_close]
            #     # D_vec_close = [D_hat[0]*D, D_hat[1]*D, D_hat[2]*D]
            #     if D>0:
            #         D_vec_close = [D_vec_close[0]-D_hat[0]*self.obtscles_r[o_close], D_vec_close[1]-D_hat[1]*self.obtscles_r[o_close], D_vec_close[2]-D_hat[2]*self.obtscles_r[o_close]]
            #         close_point_world = [self.state[0] + D_hat[0]*D, self.state[1] + D_hat[1]*D, self.state[2] + D_hat[2]*D]
            #     else:
            #         D_vec_close = [0.0,0.0,0.0]
            #         close_point_world = [self.state[0], self.state[1], self.state[2]]

            #     # # Publish vector
            #     # point_msg.x = D_vec_close[0]
            #     # point_msg.y = D_vec_close[1]
            #     # point_msg.z = 0.0
            #     # Publish point
            #     point_msg.x = close_point_world[0]
            #     point_msg.y = close_point_world[1]
            #     point_msg.z = close_point_world[2]
            #     self.pub_closest_world.publish(point_msg)


            #     #Compute closest point - with respect to the robots frame

            #     p_cw = [[close_point_world[0]],[close_point_world[1]],[close_point_world[2]],[1]]
            #     # print("a")
            #     H_bw = self.quat2rotm(quat_bw)
            #     # print("b")
            #     H_bw[0].append(pos[0])
            #     H_bw[1].append(pos[1])
            #     H_bw[2].append(pos[2])
            #     H_bw.append([0,0,0,1])
            #     H_bw = np.matrix(H_bw)
            #     p_cb = H_bw**(-1)*p_cw

            #     point_msg2.x = p_cb[0,0]
            #     point_msg2.y = p_cb[1,0]
            #     point_msg2.z = p_cb[2,0]
            #     self.pub_closest_body.publish(point_msg2)
            #     # print ("\33[95mp_cb = [%f, %f]\33[0m" % (point_msg2.x, point_msg2.y))


            position = self.sim.getObjectPosition(self.quadcopter, -1)

            #Publish robots odomtry (with velocity)
            # odom_msg.header.stamp = rospy.Time.now()
            # odom_msg.header.frame_id = "drone"
            # odom_msg.child_frame_id = "world"
            # odom_msg.pose.pose.position.x = self.state[0]
            # odom_msg.pose.pose.position.y = self.state[1]
            # odom_msg.pose.pose.position.z = self.state[2]
            # odom_msg.pose.pose.orientation.x = self.state[4]
            # odom_msg.pose.pose.orientation.y = self.state[5]
            # odom_msg.pose.pose.orientation.z = self.state[6]
            # odom_msg.pose.pose.orientation.w = self.state[3]
            print((velocity_x, velocity_y, velocity_z))
            self.sim.setObjectPosition(self.base,-1, [position[0] + velocity_x, position[1] + velocity_y, position[2] + velocity_z])

            # (yaw_a, pitch_a, roll_a) = self.orientation_a.yaw_pitch_roll
            # orientation_error = self.orientation_d * self.orientation_a.inverse
            # (yaw, pitch, roll) = orientation_error.yaw_pitch_roll
            # print(yaw)
            # self.sim.setObjectOrientation(self.base,-1, [roll_a + roll, pitch_a + pitch, yaw_a + yaw])
            # yaw_a_desired = yaw_a + yaw
            # self.sim.setObjectOrientation(self.base,-1, [roll_a, pitch_a, yaw_a + yaw])

            # while abs(yaw_a_desired - yaw_a) > 0.001:
            #     (yaw_a, pitch_a, roll_a) = self.orientation_a.yaw_pitch_roll
            #     rate.sleep()
            
            # print("Acabei a Rotação anterior!")


            # print((yaw, pitch, roll))
            rate.sleep()



    def init_node(self):
        """Initialize ROS related variables, parameters and callbacks
        :return:
        """
        rospy.init_node("drone_sim")
        
        # Start the simulation
        self.sim.startSimulation()

        # parameters (description in yaml file)
        self.state = rospy.get_param("~state_0", 1.0)
        # self.robot_radius = float(rospy.get_param("~robot_radius", 1.0))
        self.robot_arm_len = float(rospy.get_param("~robot_arm_len", 1.0))
        self.robot_m = float(rospy.get_param("~robot_m", 1.0))
        # self.robot_width = float(rospy.get_param("~robot_width", 1.0))
        # self.robot_height = float(rospy.get_param("~robot_height", 1.0))
        # self.robot_a = float(rospy.get_param("~robot_a", 1.0))
        # self.robot_b = float(rospy.get_param("~robot_b", 1.0))
        # self.robot_r = float(rospy.get_param("~robot_r", 1.0))
        self.obtscles_pos = rospy.get_param("~obtscles_pos", [])
        self.obtscles_r = rospy.get_param("~obtscles_r", [])
        self.history = []
        self.history.append([self.state[0], self.state[1], self.state[2]])

        print("self.state: ", self.state)
        print("self.robot_arm_len: ", self.robot_arm_len)
        print("self.robot_m: ", self.robot_m)

        # publishers
        # self.pub_pose = rospy.Publisher("/drone/pose", Pose, queue_size=1)
        self.pub_odom = rospy.Publisher("/odometria_test", Odometry, queue_size=1)
        # self.pub_closest_world = rospy.Publisher("/drone/closest_point_world", Point, queue_size=1)
        # self.pub_closest_body = rospy.Publisher("/drone/closest_point_body", Point, queue_size=1)
        # self.pub_rviz_robot = rospy.Publisher("/drone/robot", MarkerArray, queue_size=1)
        # self.pub_rviz_closest = rospy.Publisher("/drone/closest_marker", Marker, queue_size=1)
        # self.pub_rviz_obst = rospy.Publisher("/drone/obstacles", MarkerArray, queue_size=1)
        # self.pub_rviz_hist = rospy.Publisher("/drone/history", MarkerArray, queue_size=1)


        # self.pub_rviz_robot_w1 = rospy.Publisher("/drone/robot_w1", Marker, queue_size=1)

        # subscribers
        rospy.Subscriber("/drone/acrorate", Quaternion, self.callback_acrorate)

        rospy.Subscriber("/drone/pose", Pose, self.callback_pose)




    def callback_acrorate(self, data):
        """Callback to get the reference velocity for the robot
        :param data: pose ROS message
        """
        self.tau = data.w
        self.omega = [data.x, data.y, data.z]

        self.orientation_d = Quaternion_lib([data.w, data.x, data.y, data.z])
    
    def callback_pose(self, data):
        """Callback to get the reference velocity for the robot
        :param data: pose ROS message
        """
        self.state[0] = data.position.x
        self.state[1] = data.position.y
        self.state[2] = data.position.z
        self.state[4] = data.orientation.x
        self.state[5] = data.orientation.y
        self.state[6] = data.orientation.z
        self.state[3] = data.orientation.w

        self.orientation_a = Quaternion_lib([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])



if __name__ == '__main__':
    node = drone_node()
    node.run()
