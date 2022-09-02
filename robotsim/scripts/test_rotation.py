#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Pose
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos
from pyquaternion import Quaternion as Quaternion_lib
import zmqRemoteApi

sim = None
try : 
    sim = zmqRemoteApi.RemoteAPIClient().getObject('sim')
    print("Connected to simulation")
except :
    print("Could not connect to the simulation.")

quadcopter = sim.getObject('/Quadcopter')
base = sim.getObject('/Quadcopter/base')

orientation_a = Quaternion_lib([1, 0, 0, 0])

def callback_pose(msg: Pose):
    global orientation_a
    orientation_a = Quaternion_lib([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

def callback_acrorate(data: Quaternion):
    """Callback to get the reference velocity for the robot
    :param data: pose ROS message
    """
    global orientation_a

    # print("I am Here")
    (yaw_a, pitch_a, roll_a) = orientation_a.yaw_pitch_roll
    # orientation = sim.getObjectOrientation(quadcopter, -1)
    orientation_d = Quaternion_lib([data.w, data.x, data.y, data.z])
    orientation_error = orientation_d * orientation_a.inverse
    (yaw, pitch, roll) = orientation_error.yaw_pitch_roll
    print(yaw)
    yaw_a_desired = yaw_a + yaw
    sim.setObjectOrientation(base,-1, [roll_a, pitch_a, yaw_a + yaw])

    while abs(yaw_a_desired - yaw_a) > 0.001:
        print("I am Here")
        (yaw_a, pitch_a, roll_a) = orientation_a.yaw_pitch_roll
    
    print("Acabei a Rotação anterior!")

rospy.init_node("test_quaternion")

rospy.Subscriber("/drone/acrorate", Quaternion, callback_acrorate)
rospy.Subscriber("/drone/pose", Pose, callback_pose)

rospy.spin()