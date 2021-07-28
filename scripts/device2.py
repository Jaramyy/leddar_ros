import leddar
import rospy
import math
import numpy as np
import time

from rospy.numpy_msg import numpy_msg
import sys
from sensor_msgs.msg import Image
from sensor_msgs.point_cloud2 import create_cloud
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import PointCloud2, Temperature
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from leddar_ros.msg import Specs
import ros_numpy

if __name__ == '__main__':
    rospy.init_node('leddar2_ros')
    print("hello world")
    dev = leddar.Device()
    sensor_list = leddar.get_devices("Serial")
    #print("sensor_list = ",sensor_list)
    print("connect =     ",dev.connect(sensor_list[0]), leddar.device_types["Serial"])
    while(True):
        print("loop")