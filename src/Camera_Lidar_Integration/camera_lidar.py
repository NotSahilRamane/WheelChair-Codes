#!/usr/bin/env python3

import rospy

from ros_numpy import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
import ros_numpy
import copy
import numpy as np 
import cv2
import time

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
        self.CAMERA_MODEL = image_geometry.PinholeCameraModel()
        self.image_pub = rospy.Publisher("/camera_lidar", Image, queue_size=5)

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to Topics")
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoSave, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.syncData, queue_size=1, buff_size=2**28)
        rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, self.syncImage, queue_size=1, buff_size=2**28)

    def syncData(self, data):
        self.POINTCLOUD_RECEIVED = 1
        self.pointcloud = data
        self.run()
    
    def syncImage(self, data):
        self.IMAGE_RECEIVED = 1
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.run()
    
    def cameraInfoSave(self, msg):
        self.CAMERA_MODEL.fromCameraInfo(msg)

    def run(self):
        tme = time.time()
        if self.POINTCLOUD_RECEIVED == 1 and self.IMAGE_RECEIVED == 1:
            # lidar_image = copy.deepcopy(self.image)
            print(self.image.shape, "Image shape")
            points3D = ros_numpy.point_cloud2.pointcloud2_to_array(self.pointcloud)
            points3D = np.asarray(points3D.tolist())

            points2D = [ self.CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3] ]
            points2D = np.asarray(points2D)
            
            print(len(points2D) == len(points3D))
            print(time.time() - tme)
            # print(points2D[1200])

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
            