#!/usr/bin/env python3

import this
import cv2
import rospy
from deep_sort.yoloV5 import YOLO_Fast
from ros_numpy import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, CameraInfo
# from sensor_msgs.msg import Twist
from av_messages.msg import objects, object_
import image_geometry
import ros_numpy
import copy
import numpy as np 
import cv2
import time
import open3d
import random

from PointCloudConversion import convertCloudFromRosToOpen3d, convertCloudFromOpen3dToRos
import ros_numpy

# import torch
import time
import numpy as np
from std_msgs.msg import Header
import open3d as o3d
import matplotlib.pyplot as plt


class Detector:
    def __init__(self):
        self.loadParameters()
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.l_h = None
        self.bridge = CvBridge()
        self.FOCAL_LENGTH = 0.0028
        self.u0 = 672/2
        self.v0 = 376/2
        self.yolo = YOLO_Fast(sc_thresh=.5, nms_thresh=.45, cnf_thresh=.45, model='/home/sahil/WheelChair-Codes/src/object_detector/scripts/deep_sort/onnx_models/yolov5s.onnx')
        rospy.loginfo("Loaded Model")
        self.RGB_IMAGE_RECEIVED = 0
        self.CAMERA_MODEL = image_geometry.PinholeCameraModel()
        self.DEPTH_IMAGE_RECEIVED = 0

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.image_topicname, CompressedImage,
                         self.storeImage, buff_size = 2**24, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoSave, queue_size=1)
        
        rospy.Subscriber(self.depth_image_topicname, Image,
                        self.storeDepthImage, queue_size=1)

    def cameraInfoSave(self, msg):
        self.CAMERA_MODEL.fromCameraInfo(msg)

    def loadParameters(self):
        '''
        do something
        '''
        rospy.loginfo("Params Loaded")
        self.image_topicname = "/camera/color/image_raw/compressed"
        self.depth_image_topicname = "/camera/depth/image_rect_raw"
        self.pub_topic_name = "/yolo_depth/detected_image"
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, Image, queue_size=1)
        self.CoordsPublisher = rospy.Publisher("/perception/vision/coords", objects, queue_size=1)

    def plot_callback(self, data):
        print("IN CALLBACK")
        # self.pointcloud = data
        
        xy = []
        pcd = convertCloudFromRosToOpen3d(data)
        # pcd_arr = np.asarray(pcd.points)
        # print(pcd_arr[0])
        # arr_front = np.where(pcd_arr[:, 1] >= 0, 1, 0) # picked all points in front of the pointcloud

        _, inliers = pcd.segment_plane(distance_threshold=0.009,ransac_n=3,num_iterations=1000)
        
        inlier_cloud=pcd.select_by_index(inliers, invert=True)
        outlier_cloud=pcd.select_by_index(inliers)

        inlier_cloud.paint_uniform_color([1,0,0])
        outlier_cloud.paint_uniform_color([0,0,1])
        labels = np.array(inlier_cloud.cluster_dbscan(eps=0.119, min_points=7))
        max_label = labels.max()
        # print(max_label, "CLUSTERS")
        colors = plt.get_cmap("tab20")(labels / (max_label 
        if max_label > 0 else 1))
        colors[labels < 0] = 0
        clusters = {}
        points = np.asarray(inlier_cloud.points)
        for i in range(len(points)):
            point = points[i]
            group = labels[i]

            if group != -1 and point[1] > 0:
                if group not in list(clusters.keys()):
                    clusters[group] = []
                    clusters[group].append(point)
                else:
                    clusters[group].append(point)

        # print(len(clusters), "Final")
        # kes = clusters.keys()
        # for k in kes:
        #     pts = np.asarray(clusters[k])
        #     height = max(pts[:, 2])
        #     depth = max(pts[:, 1])
        #     lateral = np.mean(pts[:, 0])
        #     if height>=0 and depth >=0:
        #         print(k, height, depth, lateral)

        
        inlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

        return clusters
        
    def storeImage(self, img): # Copy for Obj Detection
        if self.RGB_IMAGE_RECEIVED == 0:
            try:
                frame = self.bridge.compressed_imgmsg_to_cv2(img, 'bgr8')
                rospy.loginfo("RGB Image Stored")
            except CvBridgeError as e:
                rospy.loginfo(str(e))
            self.rgb_image = frame
            self.RGB_IMAGE_RECEIVED = 1
            self.sync_frames()

    def storeDepthImage(self, img): # Copy for Obj Detection
        # frame=None
        if self.DEPTH_IMAGE_RECEIVED == 0:
            try:
                frame = self.bridge.imgmsg_to_cv2(img, "32FC1")
                rospy.loginfo("Depth Image Stored")
            except CvBridgeError as e:
                rospy.loginfo(str(e))
            self.depth_image = frame
            self.DEPTH_IMAGE_RECEIVED = 1
            self.sync_frames()

    def sync_frames(self): # Copy for Obj Detection
        if self.RGB_IMAGE_RECEIVED == 1 and self.DEPTH_IMAGE_RECEIVED == 1:
            print("IN FRAMES")
            self.callObjectDetector(self.rgb_image, self.depth_image, self.pointcloud)
            self.DEPTH_IMAGE_RECEIVED = 0
            self.RGB_IMAGE_RECEIVED = 0


    def calculateParamsForDistance(self, img_cv): # Copy for Obj Detection
        FOV = 87   # FOV of camera used
        H, W = img_cv.shape[:2]
    
        # print(H, W, "H, W") 
        CX = W / 2  # center of x-axis
        CY = H/2
        FX = CX / (np.tan(FOV / 2))  # focal length of x-axis
        FY = FX
        orig_dim = (H, W)

        return orig_dim, CX, CY, FX, FY

    def getHeight(self, lateral_depth, obj_height_px, sensor_height, img_height, focal_length=4.81):
        # real height of the object (mm) = distance to object (mm) * obj height (px) * sensor height (mm) / (focal length (mm) * img height (px))
        return (lateral_depth * obj_height_px * sensor_height) / (focal_length * img_height)  


    def callObjectDetector(self, image, depth_image): # Copy for Obj Detection remove for loops
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''

        print("Tracker called")
        obj_message = objects()
        # print("Image",image)
        heights, bottom_left, x_and_ys, _, _, classes, nums, image = self.yolo.object_detection(image, visualise = True)
        # print(nums, "No. of dets")
        # clusters = self.plot_callback(pointcloud)
        orig_dim, CX, CY, FX, FY = self.calculateParamsForDistance(depth_image)
        # self.callPublisher(self.yolo.image)
        # print("Image Published")
        img_height = image.shape[0]
        # kes = clusters.keys()
        # for k in kes:
        #     pts = np.asarray(clusters[k])
        #     height = max(pts[:, 2])
        #     depth = max(pts[:, 1])
        #     lateral = np.mean(pts[:, 0])
        #     if height>=0 and depth >=0:
        #         print(k, height, depth, lateral)

    
        # print(image.shape[0], "SHAPE")
        for hgt, (b, l), (x, y) in zip(heights, bottom_left, x_and_ys):
            single_obj = object_()

            camera_y = depth_image[int(y)][int(x)] # depth

            camera_x = (x - CX) * camera_y / FX # lateral

            camera_y = camera_y/1000 # CONVERT VALUES TO METERS
            camera_x = camera_x/1000
            # lidar_z =None
            # for k in kes:
            #     pts = np.asarray(clusters[k])
            #     height = max(pts[:, 2])
            #     depth = max(pts[:, 1])
            #     lateral = np.mean(pts[:, 0])
            #     percent = 0.8
            #     if height>=0 and depth>0.85*camera_y and depth<1.15*camera_y and lateral>0.85*camera_x and lateral<1.15*camera_x:
            #         lidar_y = depth
            #         lidar_x = lateral
            #         lidar_z = height
            #         print("Height", lidar_z)
            #         print("Lateral", lidar_x, camera_x)
            #         print("Depth", lidar_y, camera_y)
            # if lidar_z != None:
            #     cv2.putText(image, str(lidar_z), (int(l), int(b)+20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255,255), thickness=2)
            object_height = self.getHeight(camera_y, hgt, 3, img_height, focal_length=4.81)
            str_object_height = '%.2f' % (object_height)
            # print("YOLO_depth: ", image.shape)
            cv2.putText(image, str_object_height, (int(l), int(b)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255,255), thickness=2)




            # first para is the horizontal pos, second is the vertical pos.

            single_obj.position.x = x
            single_obj.position.y = y
            single_obj.id.data = 0
            single_obj.object_state_dt.x = 0
            single_obj.object_state_dt.y= 0

            single_obj.object_state_dt.theta = 0
            single_obj.position.z = camera_y
            obj_message.object_detections.append(single_obj)

        self.callPublisher(image)
        self.CoordsPublisher.publish(obj_message)
        


    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        image2publish = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        # if segmented_image:
            # print("Segmented")
        self.DetectionsPublisher.publish(image2publish)