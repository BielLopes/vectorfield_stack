#!/usr/bin/env python
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped, Point, Quaternion
from nav_msgs.msg import Odometry

class TransformNode(object):

    def __init__(self):

        self.broadcast = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()

        self.position = Point()
        self.position.x = .0
        self.position.y = .0
        self.position.z = .0

        self.rotation = Quaternion(0, 0, 0, 1)

        rospy.Subscriber("/drone/odom", Odometry, self.odom_callback)
        
        # rospy.Subscriber("/imu/data", Imu, self.imu_callback)

    
    def odom_callback(self, data: Odometry):
        self.position = data.pose.pose.position
        self.rotation = data.pose.pose.orientation

    # def imu_callback(self, imu: Imu):
    #     self.rotation = Quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)

    def run(self):
        rate = rospy.Rate(1000)
        self.static_transformStamped.header.frame_id = "map"
        self.static_transformStamped.child_frame_id = "cloud"

        self.static_transformStamped.transform.translation.x = .0
        self.static_transformStamped.transform.translation.y = .0
        self.static_transformStamped.transform.translation.z = .0
        
        self.static_transformStamped.transform.rotation.x = 0
        self.static_transformStamped.transform.rotation.y = 0
        self.static_transformStamped.transform.rotation.z = 0
        self.static_transformStamped.transform.rotation.w = 1

        while not rospy.is_shutdown():
            rate.sleep()
            self.broadcast.sendTransform(self.static_transformStamped)

            self.static_transformStamped.transform.translation.x = self.position.x - 373.98
            self.static_transformStamped.transform.translation.y = self.position.y - 77.575
            self.static_transformStamped.transform.translation.z = self.position.z + 3.176

            self.static_transformStamped.transform.rotation.x = self.rotation.x
            self.static_transformStamped.transform.rotation.y = self.rotation.y
            self.static_transformStamped.transform.rotation.z = self.rotation.z
            self.static_transformStamped.transform.rotation.w = self.rotation.w

            # print(self.static_transformStamped.transform.rotation)
            
            self.static_transformStamped.header.stamp = rospy.Time.now()



if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    node = TransformNode()
    node.run()