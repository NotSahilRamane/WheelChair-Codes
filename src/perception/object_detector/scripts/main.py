#!/usr/bin/env python3
import rospy
from yolo_depth_camera import Detector


def main():
    rospy.init_node('object_tracking')
    detector = Detector()
    detector.subscribeToTopics()
    detector.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
