#!/usr/bin/env python3
import rospy
from segmentation_ros import Detector


def main():
    rospy.init_node('segmentation')
    detector = Detector()
    detector.subscribeToTopics()
    detector.publishToTopics()
    rospy.spin()


if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
