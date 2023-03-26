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

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf_conversions
import tf
import geometry_msgs
from PointCloudConversion import convertCloudFromRosToOpen3d, convertCloudFromOpen3dToRos
import ros_numpy
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
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
        self.pos = None
        self.l_h = None
        self.bridge = CvBridge()
        self.FOCAL_LENGTH = 0.0028
        self.u0 = 672/2
        self.v0 = 376/2
        self.yolo = YOLO_Fast(sc_thresh=.5, nms_thresh=.45, cnf_thresh=.45, model='/home/sahil/map/src/perception/object_detector/scripts/deep_sort/onnx_models/yolov5s.onnx')
        rospy.loginfo("Loaded Model")
        self.RGB_IMAGE_RECEIVED = 0
        self.CAMERA_MODEL = image_geometry.PinholeCameraModel()
        self.DEPTH_IMAGE_RECEIVED = 0
        self.POS_RECEIVED = 0

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.image_topicname, Image,
                         self.storeImage, buff_size = 2**24, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoSave, queue_size=1)
        rospy.Subscriber('/formatted_odom_msg', Odometry, self.storePos, buff_size = 2**24, queue_size=1)
        rospy.Subscriber(self.depth_image_topicname, Image,
                        self.storeDepthImage, queue_size=1)

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
        self.CoordsPublisher = rospy.Publisher("/perception/vision/coords", MarkerArray, queue_size=1)
        
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
            
    def local_to_global(self, local_xyz, global_xyz, yaw):
        yaw_rad = yaw
        # Define rotation matrix
        rot_matrix = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                               [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                               [0, 0, 1]])
        # Convert local coordinates to global coordinates
        global_xyz = np.dot(rot_matrix, local_xyz) + global_xyz
        return global_xyz

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
    
    def storePos(self, msg): # Copy for Obj Detection
        # frame=None
        if self.POS_RECEIVED == 0:
            self.pos = msg
            self.POS_RECEIVED = 1
            self.sync_frames()

    def sync_frames(self): # Copy for Obj Detection
        if self.RGB_IMAGE_RECEIVED == 1 and self.DEPTH_IMAGE_RECEIVED == 1 and self.POS_RECEIVED == 1:
            print("IN FRAMES")
            self.callObjectDetector(self.rgb_image, self.depth_image, self.pos)
            self.DEPTH_IMAGE_RECEIVED = 0
            self.RGB_IMAGE_RECEIVED = 0
            self.POS_RECEIVED = 0

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


    def callObjectDetector(self, image, depth_image, pos): # Copy for Obj Detection remove for loops
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''

        print("Tracker called")
        obj_message = objects()
        # print("Image",image)
        marker_array = MarkerArray()
        markers = []
        heights, bottom_left, x_and_ys, _, _, classes, nums, image = self.yolo.object_detection(image, visualise = True)

        orig_dim, CX, CY, FX, FY = self.calculateParamsForDistance(depth_image)
        # self.callPublisher(self.yolo.image)
        # print("Image Published")
        img_height = image.shape[0]
        ap = 1
        for hgt, (b, l), (x, y) in zip(heights, bottom_left, x_and_ys):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "map"
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.ns = "cone_"+str(x)
            marker.id = int(x)
            
            single_obj = object_()

            camera_y = depth_image[int(x)][int(y)] # depth

            camera_x = (x - CX) * camera_y / FX # lateral

            camera_y = camera_y/1000 # CONVERT VALUES TO METERS
            camera_x = camera_x/1000
            object_height = self.getHeight(camera_y, hgt, 3, img_height, focal_length=4.81)
            str_object_height = '%.2f' % (object_height)
            # print("YOLO_depth: ", image.shape)
            cv2.putText(image, str_object_height, (int(l), int(b)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255,255), thickness=2)
            orientation_q = pos.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            global_xyz = self.local_to_global(np.array([pos.pose.pose.position.y, pos.pose.pose.position.x, 0]), np.array([camera_x, camera_y, 0]), yaw)
            marker.pose.position.x = global_xyz[0]
            marker.pose.position.y = global_xyz[1]
            marker.pose.position.z = 0.0
            if ap == 1:
                marker.action = marker.DELETEALL
                ap = 0
            marker.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 0.7
            marker.color.r, marker.color.g, marker.color.b = (0, 0, 255)
            markers.append(marker)
            single_obj.position.x = x
            single_obj.position.y = y
            single_obj.id.data = 0
            single_obj.object_state_dt.x = 0
            single_obj.object_state_dt.y= 0

            single_obj.object_state_dt.theta = 0
            single_obj.position.z = camera_y
            obj_message.object_detections.append(single_obj)
        marker_array.markers = markers

        self.callPublisher(image)
        self.CoordsPublisher.publish(marker_array)
        


    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        image2publish = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        # if segmented_image:
            # print("Segmented")
        self.DetectionsPublisher.publish(image2publish)
