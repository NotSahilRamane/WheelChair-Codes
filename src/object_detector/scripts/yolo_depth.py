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


# import torch
import time
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

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
        rospy.Subscriber(self.image_topicname, CompressedImage,
                         self.storeImage, buff_size = 2**24, queue_size=1)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraInfoSave, queue_size=1)
        
        rospy.Subscriber("/velodyne_points", PointCloud2, self.syncData, queue_size=1, buff_size=2**28)
        rospy.Subscriber(self.depth_image_topicname, Image,
                        self.storeDepthImage, queue_size=1)
        
    def syncData(self, data):
        self.POINTCLOUD_RECEIVED = 1
        self.pointcloud = data

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
            self.callObjectDetector(self.rgb_image, self.depth_image)
            self.DEPTH_IMAGE_RECEIVED = 0
            self.RGB_IMAGE_RECEIVED = 0

    def calculateParamsForDistance(self, img_cv): # Copy for Obj Detection
        FOV = 87   # FOV of camera used
        H, W = img_cv.shape[:2]
    
        print(H, W, "H, W") 
        CX = W / 2  # center of x-axis
        FX = CX / (2*np.tan(FOV*3.14 /360))  # focal length of x-axis
        orig_dim = (H, W)

        return orig_dim, CX, FX

    def getHeight(self, lateral_depth, obj_height_px, sensor_height, img_height, focal_length=4.81):
        # real height of the object (mm) = distance to object (mm) * obj height (px) * sensor height (mm) / (focal length (mm) * img height (px))
        return (lateral_depth * obj_height_px * sensor_height) / (focal_length * img_height)  


    def callObjectDetector(self, image, depth_image): # Copy for Obj Detection remove for loops
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        # print("Tracker called")
        obj_message = objects()
        # print("Image",image)
        heights, bottom_left, x_and_ys, _, _, classes, nums, image = self.yolo.object_detection(image, visualise = True)
        # print(nums, "No. of dets")
  
        orig_dim, CX, FX = self.calculateParamsForDistance(depth_image)
        # self.callPublisher(self.yolo.image)
        # print("Image Published")
        img_height = image.shape[0]
        print(image.shape[0], "SHAPE")
        for hgt, (b, l), (x, y) in zip(heights, bottom_left, x_and_ys):
            single_obj = object_()

            depth = depth_image[int(y)][int(x)] # instead of x, y, give pixel coordinates of Bounding boxes
            lidar_image = image
            pc2img = depth_image
            ld_img = pc2img
            print("Depth, ", depth)
            lateral = (x - CX) * depth / FX
            # print(lateral, depth, id, "Veh coordinates")
            # print("x: ", x, "y: ", y)
            # print("b: ", b, "r: ", l)
            object_height = self.getHeight(depth, hgt, 3, img_height, focal_length=4.81)
            str_object_height = '%.2f' % (object_height)
            # print("YOLO_depth: ", image.shape)
            cv2.putText(image, str_object_height, (int(l), int(b)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255,255), thickness=2)
            
            
            color = (0, 255, 0) # Green :)

            if self.POINTCLOUD_RECEIVED == 1:
                points3D = ros_numpy.point_cloud2.pointcloud2_to_array(self.pointcloud)
                points3D = np.asarray(points3D.tolist())
                inrange = np.where((points3D[:, 2] > 0))
                points3D = points3D[inrange[0]]
                points2D = [ self.CAMERA_MODEL.project3dToPixel(point) for point in points3D[:, :3] ]
                points2D = np.asarray(points2D)
                for i in range(len(pc2img)):
                    for j in range(len(pc2img[i])):
                        if len(points2D) == len(points3D) and pc2img[i, j] <= 9000 and pc2img[i, j] >= 300:
                            L = [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                            if random.choice(L):
                                # print("Hello")
                                continue
                        else:
                            continue
                lat = ld_img[int(y)][int(x)] * hgt

                lon = 2*615
                lidar_height = lat/lon
                str_lidar_height = '%.2f' % (lidar_height)
            cv2.putText(image, str_lidar_height, (int(l), int(b)+15), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255,255), thickness=2)

                

                
            # first para is the horizontal pos, second is the vertical pos.

            single_obj.position.x = x
            single_obj.position.y = y
            single_obj.id.data = 0
            single_obj.object_state_dt.x = 0
            single_obj.object_state_dt.y= 0

            single_obj.object_state_dt.theta = 0
            single_obj.position.z = depth
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