import rospy
from sensor_msgs.point_cloud2 import PointCloud2
from math import inf, sqrt
from ros_numpy.point_cloud2 import pointcloud2_to_array
import numpy as np
import numba as nb

@nb.njit
def calc_minor_point(cloud, x_a, y_a, z_a):
    x_m, y_m, z_m = 0, 0, 0
    minor_distance = inf

    for point in cloud:
        x, y, z = point[0], point[1], point[2]
        distance = sqrt((x - x_a)**2 + (y - y_a)**2 + (z - z_a)**2)
    
        if distance < minor_distance:
            minor_distance = distance
            x_m, y_m, z_m = x, y, z
    
    return [x_m, y_m, z_m]

# nb.njit(nopython=True)
def callback_cloud(ros_cloud: PointCloud2):
    cloud = pointcloud2_to_array(ros_cloud)
    
    # minor_point = calc_minor_point(cloud, np.array([0, 0, 0]))
    minor_point = calc_minor_point(cloud, 0.7, 0.7, 0)

    print(minor_point)

rospy.init_node("teste_node")
rospy.Subscriber("/cloud", PointCloud2, callback_cloud)
rospy.spin()