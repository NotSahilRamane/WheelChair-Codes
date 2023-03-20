import rospy
from nav_msgs.msg import Odometry
from marvelmind_nav.msg import hedge_pos_ang
import math
import tf_conversions
# from tf2_geometry_msgs.msg import 
import tf
import geometry_msgs
import time

class GPS2Odom:
    def __init__(self):
        self.loadTopics()
        self.br = tf.TransformBroadcaster()


    def loadTopics(self):
        print("Topics loaded")
        self.gps_topicname = '/hedge_pos_ang'
        self.publisher_topicname = '/formatted_odom_msg'

    def subscribeToTopics(self):
        rospy.loginfo("Subscribed to topics")

        rospy.Subscriber(self.gps_topicname, hedge_pos_ang, self.format_msg_cb)

    def publishToTopics(self):
        rospy.loginfo("Published to topics")
        self.gps2odomPublisher = rospy.Publisher(
            self.publisher_topicname, Odometry, queue_size=1
        )

    def format_msg_cb(self, msg):
        Odom_msg = Odometry()
        print(msg.timestamp_ms)
        # Odom_msg.header.stamp
        # self.br.sendTransform((msg.x_m, msg.y_m, msg.z_m),tf.transformations.quaternion_from_euler(0, 0, self.normalize_angle(msg.angle)),rospy.Time.now(),"base_link","map")
        self.br.sendTransform((2.732, 0.069, 0),tf.transformations.quaternion_from_euler(0, 0, self.normalize_angle(0)),rospy.Time.now(),"base_link","map")

        Odom_msg.header.frame_id = "map"
        # Odom_msg.pose.pose.position.x = msg.x_m
        # Odom_msg.pose.pose.position.y = msg.y_m
        # Odom_msg.pose.pose.position.z = msg.z_m
        # angle = self.normalize_angle(msg.angle)
        # Odom_msg.pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, angle))
        
        Odom_msg.pose.pose.position.x = 2.732
        Odom_msg.pose.pose.position.y = 0.069
        Odom_msg.pose.pose.position.z = 0
        angle = self.normalize_angle(0)
        Odom_msg.pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, angle))
        
        print(Odom_msg)
        self.callPublisher(Odom_msg)
        print("Published")


    def deg2rad(self, degrees):
        return degrees * math.pi / 180.0

    def normalize_angle(self, angle_degrees):
        angle_radians = self.deg2rad(angle_degrees)
        normalized_angle = math.atan2(math.sin(angle_radians), math.cos(angle_radians))
        return normalized_angle


    def callPublisher(self, formatted_msg):
        '''
        the final publisher function
        '''
        self.gps2odomPublisher.publish(formatted_msg)

