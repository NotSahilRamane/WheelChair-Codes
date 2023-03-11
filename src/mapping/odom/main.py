import rospy
from GpsToOdom import GPS2Odom


def main():
    rospy.init_node('gps_to_odom')
    gps_to_odom = GPS2Odom()
    gps_to_odom.subscribeToTopics()
    gps_to_odom.publishToTopics()
    rospy.spin()

if __name__ == u'__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass