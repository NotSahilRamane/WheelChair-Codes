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
import math
from PointCloudConversion import convertCloudFromRosToOpen3d, convertCloudFromOpen3dToRos
import ros_numpy

# import torch
import time
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import open3d as o3d
import matplotlib.pyplot as plt


class Detector:
    def __init__(self):
        self.loadParameters()
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.POINTCLOUD_RECEIVED = 0
        self.pointcloud = None
        self.pointcloud_message = None
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
        rospy.Subscriber(self.image_topicname, Image,
                         self.storeImage, buff_size = 2**24, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoSave, queue_size=1)
        
        rospy.Subscriber("/velodyne_points", PointCloud2, self.syncData, queue_size=1, buff_size=2**28)
        rospy.Subscriber(self.depth_image_topicname, Image,
                        self.storeDepthImage, queue_size=1)
        
    def syncData(self, data):
        if self.POINTCLOUD_RECEIVED == 0:
            self.pointcloud = data
            # print("POINTCLOUD SAVED")
            self.POINTCLOUD_RECEIVED = 1
            self.sync_frames()

    def cameraInfoSave(self, msg):
        self.CAMERA_MODEL.fromCameraInfo(msg)

    def loadParameters(self):
        '''
        do something
        '''
        rospy.loginfo("Params Loaded")
        self.image_topicname = "/camera/color/image_raw"
        self.depth_image_topicname = "/camera/depth/image_rect_raw"
        self.pub_topic_name = "/yolo_depth/detected_image"
    
    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.DetectionsPublisher = rospy.Publisher(
            self.pub_topic_name, Image, queue_size=1)
        self.CoordsPublisher = rospy.Publisher("/perception/vision/coords", objects, queue_size=1)

    def plot_callback(self, data):
        # print("IN CALLBACK")
        # self.pointcloud = data
        
        xy = []
        pcd = convertCloudFromRosToOpen3d(data)
        # pcd_arr = np.asarray(pcd.points)
        # print(pcd_arr[0])
        # arr_front = np.where(pcd_arr[:, 1] >= 0, 1, 0) # picked all points in front of the pointcloud
        points_arr = np.asarray(pcd.points)
        print("POINTS", len(points_arr))
        points_list = []
        for point in points_arr:
            if point[1] > -5 and point[1] < 5 and point[0] > 0:
                points_list.append(point)
        pcd_fil = o3d.geometry.PointCloud()
        pcd_fil.points = np.asarray(points_list)
        _, inliers = pcd_fil.segment_plane(distance_threshold=0.09,ransac_n=3,num_iterations=1000)
        
        inlier_cloud=pcd_fil.select_by_index(inliers, invert=True)
        outlier_cloud=pcd_fil.select_by_index(inliers)

        inlier_cloud.paint_uniform_color([1,0,0])
        outlier_cloud.paint_uniform_color([0,0,1])
        labels = np.array(inlier_cloud.cluster_dbscan(eps=0.05, min_points=3))
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
                frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
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
        if self.RGB_IMAGE_RECEIVED == 1 and self.DEPTH_IMAGE_RECEIVED == 1 and self.POINTCLOUD_RECEIVED == 1:
            # print("IN FRAMES")
            try:
                self.callObjectDetector(self.rgb_image, self.depth_image, self.pointcloud)
                self.DEPTH_IMAGE_RECEIVED = 0
                self.RGB_IMAGE_RECEIVED = 0
                self.POINTCLOUD_RECEIVED = 0
            except:
                print("NO DETECTIONS")
                self.DEPTH_IMAGE_RECEIVED = 0
                self.RGB_IMAGE_RECEIVED = 0
                self.POINTCLOUD_RECEIVED = 0
                


    def calculateParamsForDistance(self, img_cv): # Copy for Obj Detection
        FOVH = 87   # FOV of camera used
        # FOVV = 58
        H, W = img_cv.shape[:2]

        # print(H, W, "H, W") 
        CX = W / 2  # center of x-axis
        FX = CX / (np.tan(FOVH / 2))  # focal length of x-axis
        # FY = CY / (np.tan(FOVV / 2))
        orig_dim = (H, W)

        return orig_dim, CX, FX

    def getHeight(self, lateral_depth, obj_height_px, sensor_height, img_height, focal_length=1.93):
        # real height of the object (mm) = distance to object (mm) * obj height (px) * sensor height (mm) / (focal length (mm) * img height (px))
        return (lateral_depth * obj_height_px * sensor_height) / (focal_length * img_height)  


    def callObjectDetector(self, image, depth_image, pointcloud): # Copy for Obj Detection remove for loops
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''

        # print("Tracker called")
        obj_message = objects()
        # print("Image",image)
        # depth_image = cv2.resize(depth_image, (720, 1280))
        heights, bottom_left, x_and_ys, _, _, classes, nums, image = self.yolo.object_detection(image, visualise = True)
        # print(nums, "No. of dets")
        clusters = self.plot_callback(pointcloud)
        
        orig_dim, CX, FX = self.calculateParamsForDistance(depth_image)
        # self.callPublisher(self.yolo.image)
        # print("Image Published")
        img_height = image.shape[0]

        # print(image.shape, depth_image.shape, "IMAGE SHAPE")
        kes = clusters.keys()
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
            
            camera_y = depth_image[int(x)][int(y)] # depth
            print(depth_image.shape, "DEPTH SHAPE")
            camera_x = (y - CX) * camera_y / FX # lateral

            camera_y = camera_y/1000 # CONVERT VALUES TO METERS

            ratio = y/depth_image.shape[1]
            # print("RATIO", y, depth_image.shape[1], ratio)
            # print(depth_image.shape[1], "SHAPE !")
            if ratio > 0.5:
                camera_angle = 43.5*(ratio-0.5)
            elif ratio < 0.5:
                camera_angle = -1*43.5*(0.5-ratio)
            else:
                camera_angle = 0

            # print(ratio, angle, "RATIO")
            camera_x = -1*camera_x/1000
            lidar_z =None
            for k in kes:
                pts = np.asarray(clusters[k])
                height = max(pts[:, 2])
                depth = max(pts[:, 0])
                lateral = np.mean(pts[:, 1])
                angle = (math.atan(depth/lateral)) * 180 / 3.14
                if height>=0:
                    # print("Angle ISSU")
                    if angle>0.80*camera_angle and angle<1.20*camera_angle:
                        # print("DEPTH ISSUE")
                        # print("INSIDE ANGLE")
                        print(depth, camera_y, "DEPPTH")
                        if depth>0.80*camera_y and depth<1.20*camera_y:
                            lidar_y = depth
                            lidar_x = lateral
                            lidar_z = height
                            lidar_angle = angle
                            print("TRUE")
                            # print(angle, camera_angle, "ANGLE")

                            # print("angle", lidar_angle, camera_angle)
                            # print("Depth", lidar_y, camera_y)
            if lidar_z:

                cv2.putText(image, str(lidar_z), (int(l), int(b)-30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0,255), thickness=2)

            # if lidar_z != None:
            #     cv2.putText(image, str(lidar_y), (int(l), int(b)+20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0), thickness=2)
            object_height = self.getHeight(camera_y, hgt, 3, img_height, focal_length=1.93)
            str_object_height = '%.2f' % (object_height)
            # print("YOLO_depth: ", image.shape)
            cv2.putText(image, str(round(camera_y, 2)) + " " + str(round(camera_angle, 2)), (int(l), int(b)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255,255), thickness=2)
            # cv2.putText(image, str(lidar_y) + " " + str(lidar_angle), (int(l), int(b)+30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0), thickness=2)


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