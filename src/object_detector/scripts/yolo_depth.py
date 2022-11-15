#!/usr/bin/env python3

import this
import cv2
import rospy
from deep_sort.yoloV5 import YOLO_Fast

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
# from sensor_msgs.msg import Twist
from av_messages.msg import objects, object_


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

        self.yolo = YOLO_Fast(sc_thresh=.5, nms_thresh=.45, cnf_thresh=.45, model='/home/sahil/Robin/src/object_detector/scripts/deep_sort/onnx_models/yolov5s.onnx')
        rospy.loginfo("Loaded Model")
        self.RGB_IMAGE_RECEIVED = 0
        self.DEPTH_IMAGE_RECEIVED = 0

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")
        rospy.Subscriber(self.image_topicname, CompressedImage,
                         self.storeImage, buff_size = 2**24, queue_size=1)
        rospy.Subscriber(self.depth_image_topicname, Image,
                        self.storeDepthImage, queue_size=1)

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
    
    def callObjectDetector(self, image, depth_image): # Copy for Obj Detection remove for loops
        '''
        Call the segmentation model related functions here (Reuben, Mayur)
        and the final publish function (To be done by sahil)
        '''
        # print("Tracker called")
        obj_message = objects()
        # print("Image",image)
        x_and_ys, _, _, classes, nums = self.yolo.object_detection(image, visualise = True)
        print(nums, "No. of dets")
  
        orig_dim, CX, FX = self.calculateParamsForDistance(depth_image)
        self.callPublisher(self.yolo.image)
        print("Image Published")
        for x, y in x_and_ys:
            single_obj = object_()
            # print("Inside forloop main")
            # if self.loop_number == 0:
            #     print("loop 0")
            # print(y,x, "Y, X")
            depth = depth_image[int(y)][int(x)] # instead of x, y, give pixel coordinates of Bounding boxes
            lateral = (x - CX) * depth / FX
            print(lateral, depth, id, "Veh coordinates")
            # if lateral > -1.5 and lateral < 1.5:
            # self.last_obj_pos_depth = depth
            # self.last_obj_pos_lateral = lateral
            # print(self.id_to_track, "ID TO TRACK")

            # print(lateral, depth)
            single_obj.position.x = x
            single_obj.position.y = y
            single_obj.id.data = 0
            single_obj.object_state_dt.x = 0
            single_obj.object_state_dt.y= 0

            single_obj.object_state_dt.theta = 0
            single_obj.position.z = depth
            obj_message.object_detections.append(single_obj)
            # self.loop_number = 1
        self.CoordsPublisher.publish(obj_message)
            # print("published loop 0")
            # self.last_time = time.time()


    def callPublisher(self, image):
        '''
        the final publisher function
        '''
        image2publish = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        # if segmented_image:
            # print("Segmented")
        self.DetectionsPublisher.publish(image2publish)