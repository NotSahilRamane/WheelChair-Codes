;; Auto-generated. Do not edit!


(when (boundp 'av_messages::objects)
  (if (not (find-package "AV_MESSAGES"))
    (make-package "AV_MESSAGES"))
  (shadow 'objects (find-package "AV_MESSAGES")))
(unless (find-package "AV_MESSAGES::OBJECTS")
  (make-package "AV_MESSAGES::OBJECTS"))

(in-package "ROS")
;;//! \htmlinclude objects.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass av_messages::objects
  :super ros::object
  :slots (_header _object_detections ))

(defmethod av_messages::objects
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:object_detections __object_detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _object_detections __object_detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:object_detections
   (&rest __object_detections)
   (if (keywordp (car __object_detections))
       (send* _object_detections __object_detections)
     (progn
       (if __object_detections (setq _object_detections (car __object_detections)))
       _object_detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; av_messages/object_[] _object_detections
    (apply #'+ (send-all _object_detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; av_messages/object_[] _object_detections
     (write-long (length _object_detections) s)
     (dolist (elem _object_detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; av_messages/object_[] _object_detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _object_detections (let (r) (dotimes (i n) (push (instance av_messages::object_ :init) r)) r))
     (dolist (elem- _object_detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get av_messages::objects :md5sum-) "29c2393b11acc79632a74bbd645a8019")
(setf (get av_messages::objects :datatype-) "av_messages/objects")
(setf (get av_messages::objects :definition-)
      "std_msgs/Header header
av_messages/object_[] object_detections
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: av_messages/object_
# A standard object message for perception

geometry_msgs/Point position # position in x, y ,z in meters
std_msgs/String class # string for the actual object class, i.e. tree, pedestrian, car, truck, etc.

std_msgs/String status # wether the object is static/dynamic in nature etc.
geometry_msgs/Pose2D object_state_dt # Velocity in x, y, theta of a dynamic object

std_msgs/Int16 id # unique object id for every object
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta

================================================================================
MSG: std_msgs/Int16
int16 data

")



(provide :av_messages/objects "29c2393b11acc79632a74bbd645a8019")


