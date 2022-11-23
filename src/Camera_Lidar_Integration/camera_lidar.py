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
import open3d
import random

from PointCloudConversion import convertCloudFromRosToOpen3d, convertCloudFromOpen3dToRos

class Integration():
    def __init__(self):
        self.POINTCLOUD_RECEIVED = 0
        self.IMAGE_RECEIVED = 0
        self.pointcloud = None
        self.pointcloud_message = None
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
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.savePCL2, queue_size=1, buff_size=2**28)


    def syncData(self, data):
        # self.POINTCLOUD_RECEIVED = 1
        self.pointcloud = data
    
    def savePCL2(self,data):
        self.pointcloud_message = self.bridge.imgmsg_to_cv2(data, "32FC1")
        self.POINTCLOUD_RECEIVED =  1
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
            lidar_image = self.image
            imposed_image = self.pointcloud_message
            pc2img = cv2.resize(imposed_image, (800, 800))
            lidar_image = cv2.resize(lidar_image, (800, 800))

            points3D = ros_numpy.point_cloud2.pointcloud2_to_array(self.pointcloud)
            points3D = np.asarray(points3D.tolist())
            # filtering out all pointcloud the back of the camera

            inrange = np.where((points3D[:, 2] > 0))
            points3D = points3D[inrange[0]]
            print("P3D", len(points3D))
            points2D = [ self.CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3] ]
            points2D = np.asarray(points2D)

            # inrange = np.where((points2D[:, 0] >= 0) &
            #            (points2D[:, 1] >= 0))
            # points2D = points2D[inrange[0]].round().astype('int')
            print(len(points2D))

            for i in range(len(pc2img)):
                for j in range(len(pc2img[i])):
                    if len(points2D) == len(points3D) and pc2img[i, j] <= 9000 and pc2img[i, j] >= 300:
                        L = [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                        if random.choice(L):
                            # print("Hello")
                            cv2.circle(lidar_image, (j,i), 1, (0, 255, 0), -1)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(lidar_image, "bgr8"))
            print("Image Published")
            print(len(points2D) == len(points3D))
            print(time.time() - tme)
            self.POINTCLOUD_RECEIVED, self.IMAGE_RECEIVED = 0, 0

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
