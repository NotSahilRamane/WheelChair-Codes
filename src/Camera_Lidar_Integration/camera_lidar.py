#!/usr/bin/env python3

import rospy

from ros_numpy import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import copy
import numpy as np 

class Integration():
    def __init__(self):

        self.POINTCLOUD_RECEIVED = 0
        self.IMAGE_RECEIVED = 0
        self.pointcloud = None
        self.image = None
        self.bridge = CvBridge()
        self.FOCAL_LENGTH = 0.0028
        self.u0 = 672/2
        self.v0 = 376/2

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to Topics")
        rospy.Subscriber("/velodyne_points", PointCloud2, self.syncData, queue_size=1, buff_size=2**28)
        rospy.Subscriber("/left/image_rect_color/compressed", CompressedImage, self.syncImage, queue_size=1, buff_size=2**28)

    def syncData(self, data):
        self.POINTCLOUD_RECEIVED = 1
        self.pointcloud = point_cloud2.pointcloud2_to_array(data, squeeze=False)
        self.run()
    
    def syncImage(self, data):
        self.IMAGE_RECEIVED = 1
        self.image = self.bridge.compressed_imgmsg_to_cv2(data)
        self.run()

    def run(self):
        if self.POINTCLOUD_RECEIVED == 1 and self.IMAGE_RECEIVED == 1:
            lidar_image = copy.deepcopy(self.image)
            # print(len(lidar_image[0]))
            for x in self.pointcloud[0]:
                u = ( self.FOCAL_LENGTH * (x[0] - 1.541 + 0.2) / (x[2] + 0.471) ) + self.u0
                v = ( self.FOCAL_LENGTH * x[1] / (x[2] + 0.471) ) + self.v0
                print(u,v)
                lidar_image[int(v)][int(u)] = x[2] + 0.471
            self.IMAGE_RECEIVED = 0
            self.POINTCLOUD_RECEIVED = 0

def main():

    rospy.init_node('camera_lidar_integration')
    camera_lidar = Integration()
    while not rospy.is_shutdown():
        camera_lidar.subscribeToTopics()
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
            