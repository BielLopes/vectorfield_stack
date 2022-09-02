#!/usr/bin/env python

import zmqRemoteApi
import rospy
from geometry_msgs.msg import Point

CONST_VELOCITY = -0.02

sim = None

try : 
    sim = zmqRemoteApi.RemoteAPIClient().getObject('sim')
    print("Connected to simulation")
except :
    print("Could not connect to the simulation.")
    exit()

sim.startSimulation()

base = sim.getObject('/Quadcopter/base')

velocity_x = 0.
velocity_y = 0.
velocity_z = 0.

def robot_velocity_callback(msg: Point) -> None:
    global velocity_x, velocity_y, velocity_z
    velocity_x = msg.x * CONST_VELOCITY
    velocity_y = msg.y * CONST_VELOCITY
    velocity_z = msg.z * CONST_VELOCITY

    # print(f"Vel_x: {velocity_x} | Vel_y: {velocity_y} | Vel_z: {velocity_z}")

    

rospy.init_node("reciver")
rospy.Subscriber("/stack/vel", Point, robot_velocity_callback)

rate = rospy.Rate(10000)
while not rospy.is_shutdown():
    pos = sim.getObjectPosition(base, -1)
    sim.setObjectPosition(base,-1, [pos[0] + velocity_x, pos[1] + velocity_y, pos[2] + velocity_z])
    rate.sleep()

sim.stopSimulation()