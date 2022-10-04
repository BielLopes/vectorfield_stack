#!/usr/bin/env python
import rospy
import tf2_ros
import message_filters
from ros_numpy.point_cloud2 import pointcloud2_to_array, array_to_pointcloud2

from geometry_msgs.msg import TransformStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class TransformNode(object):

    def __init__(self):

        self.broadcast = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = TransformStamped()
        self.static_transformStamped.header.frame_id = "map"
        self.static_transformStamped.child_frame_id = "cloud_sync"

        self.position = Point()
        self.position.x = .0
        self.position.y = .0
        self.position.z = .0

        self.rotation = Quaternion(0, 0, 0, 1)
        
        cloud_sub = message_filters.Subscriber("/cloud", PointCloud2)
        odom_sub = message_filters.Subscriber("/drone/odom", Odometry)

        ts = message_filters.ApproximateTimeSynchronizer([cloud_sub, odom_sub], 100, 0.1)
        ts.registerCallback(self.sync_callback)

        self.sync_cloud_publisher = rospy.Publisher("/cloud_sync", PointCloud2, tcp_nodelay=True)

    def sync_callback(self, cloud:PointCloud2, odom: Odometry):

        array = pointcloud2_to_array(cloud)
        new_cloud = array.copy()
        ros_cloud_sync = array_to_pointcloud2(new_cloud)
        ros_now = rospy.Time.now()
        ros_cloud_sync.header.stamp = ros_now
        ros_cloud_sync.header.frame_id = "cloud_sync"


        self.static_transformStamped.header.stamp = ros_now

        self.static_transformStamped.transform.translation.x = odom.pose.pose.position.x - 373.98
        self.static_transformStamped.transform.translation.y = odom.pose.pose.position.y - 77.575
        self.static_transformStamped.transform.translation.z = odom.pose.pose.position.z + 3.176

        self.static_transformStamped.transform.rotation.x = odom.pose.pose.orientation.x
        self.static_transformStamped.transform.rotation.y = odom.pose.pose.orientation.y
        self.static_transformStamped.transform.rotation.z = odom.pose.pose.orientation.z
        self.static_transformStamped.transform.rotation.w = odom.pose.pose.orientation.w

        self.broadcast.sendTransform(self.static_transformStamped)
        self.sync_cloud_publisher.publish(ros_cloud_sync)



if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    node = TransformNode()
    rospy.spin()