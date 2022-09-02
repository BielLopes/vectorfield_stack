#!/usr/bin/env python

import zmqRemoteApi
import rospy
from sensor_msgs.msg import Joy

CONST_VELOCITY = 0.01
NUM_OBSTACLES = 3

sim = None

try : 
    sim = zmqRemoteApi.RemoteAPIClient().getObject('sim')
    print("Connected to simulation")
except :
    print("Could not connect to the simulation.")
    exit()

# sim.startSimulation()

obstacles = [sim.getObject(f'/obstacle_{i+1}') for i in range(NUM_OBSTACLES)]

velocities = [[0, 0, 0] for i in range(NUM_OBSTACLES)]

def robot_velocity_callback(msg: Joy) -> None:
    global velocities
    # Obstacle 1
    velocities[0][0] = msg.axes[0] * CONST_VELOCITY
    velocities[0][1] = msg.axes[1] * CONST_VELOCITY
    if (not abs(msg.axes[0]) > 0.) and abs(msg.axes[2]) > 0:
        velocities[0][2] = msg.axes[2] * CONST_VELOCITY
    else:
        velocities[0][2] = 0
    
    # Obstacle 2
    velocities[1][0] = msg.axes[5] * -1 * CONST_VELOCITY
    velocities[1][1] = msg.axes[6] * CONST_VELOCITY
    if msg.buttons[4]:
        velocities[1][2] = CONST_VELOCITY
    elif msg.buttons[6]:
        velocities[1][2] = -1 * CONST_VELOCITY
    else:
        velocities[1][2] = 0.

    # Obstacle 3
    if msg.buttons[1]:
        velocities[2][0] = CONST_VELOCITY
    elif msg.buttons[3]:
        velocities[2][0] = -1 * CONST_VELOCITY
    else:
        velocities[2][0] = 0.

    if msg.buttons[0]:
        velocities[2][1] = CONST_VELOCITY
    elif msg.buttons[2]:
        velocities[2][1] = -1 * CONST_VELOCITY
    else:
        velocities[2][1] = 0.

    if msg.buttons[5]:
        velocities[2][2] = CONST_VELOCITY
    elif msg.buttons[7]:
        velocities[2][2] = -1 * CONST_VELOCITY
    else:
        velocities[2][2] = 0.
    

rospy.init_node("obstacle_teleop")
rospy.Subscriber("/joy", Joy, robot_velocity_callback)

rate = rospy.Rate(10000)
while not rospy.is_shutdown():
    for i in range(NUM_OBSTACLES):
        pos = sim.getObjectPosition(obstacles[i], -1)
        sim.setObjectPosition(obstacles[i],-1, [pos[0] + velocities[i][0], pos[1] + velocities[i][1], pos[2] + velocities[i][2]])
    rate.sleep()

# sim.stopSimulation()