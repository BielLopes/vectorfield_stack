#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Pose
import numpy as np
from tf.transformations import quaternion_from_euler
from pyquaternion import Quaternion as Quaternion_lib
import zmqRemoteApi

rospy.init_node("test_quaternion")
pose_publisher = rospy.Publisher("/drone/pose", Pose, queue_size=100)

sim = None
try : 
    sim = zmqRemoteApi.RemoteAPIClient().getObject('sim')
    print("Connected to simulation")
except :
    print("Could not connect to the simulation.")

quadcopter = sim.getObject('/Quadcopter')
base = sim.getObject('/Quadcopter/base')

orientation_a = Quaternion_lib([1, 0, 0, 0])


def run_odometry():
    global orientation_a
    global pose_publisher

    seq_gt = True
    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        pose_msg: Pose = Pose()
        bl = sim.getObjectMatrix(quadcopter, -1)

        if seq_gt:
            seq_gt = False
            H = bl
            sim.invertMatrix(H)

        b2 = sim.multiplyMatrices(H,bl)

        pose_msg.position.x = b2[3] - 4
        pose_msg.position.y = b2[7] - 4
        pose_msg.position.z = b2[11] - 1
        # orientation_a = Quaternion_lib([1, 0, 0, 0])
        pose_msg.orientation.w = orientation_a.w
        pose_msg.orientation.x = orientation_a.x
        pose_msg.orientation.y = orientation_a.y
        pose_msg.orientation.z = orientation_a.z

        pose_publisher.publish(pose_msg)

        rate.sleep()

def callback_acrorate(data: Quaternion):
    """Callback to get the reference velocity for the robot
    :param data: pose ROS message
    """
    global orientation_a

    (yaw_a, pitch_a, roll_a) = orientation_a.yaw_pitch_roll
    orientation_d = Quaternion_lib([data.w, data.x, data.y, data.z])
    orientation_error = orientation_d * orientation_a.inverse
    (yaw, pitch, roll) = orientation_error.yaw_pitch_roll
    
    quat = quaternion_from_euler(roll_a+roll, pitch_a+pitch, yaw_a+yaw)
    orientation_a = Quaternion_lib([quat[3], quat[0], quat[1], quat[2]])

rospy.Subscriber("/drone/acrorate", Quaternion, callback_acrorate)

if __name__ == "__main__":
    run_odometry()