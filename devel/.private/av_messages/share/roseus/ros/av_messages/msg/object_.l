;; Auto-generated. Do not edit!


(when (boundp 'av_messages::object_)
  (if (not (find-package "AV_MESSAGES"))
    (make-package "AV_MESSAGES"))
  (shadow 'object_ (find-package "AV_MESSAGES")))
(unless (find-package "AV_MESSAGES::OBJECT_")
  (make-package "AV_MESSAGES::OBJECT_"))

(in-package "ROS")
;;//! \htmlinclude object_.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass av_messages::object_
  :super ros::object
  :slots (_position _class _status _object_state_dt _id ))

(defmethod av_messages::object_
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:class __class) (instance std_msgs::String :init))
    ((:status __status) (instance std_msgs::String :init))
    ((:object_state_dt __object_state_dt) (instance geometry_msgs::Pose2D :init))
    ((:id __id) (instance std_msgs::Int16 :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _class __class)
   (setq _status __status)
   (setq _object_state_dt __object_state_dt)
   (setq _id __id)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:class
   (&rest __class)
   (if (keywordp (car __class))
       (send* _class __class)
     (progn
       (if __class (setq _class (car __class)))
       _class)))
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:object_state_dt
   (&rest __object_state_dt)
   (if (keywordp (car __object_state_dt))
       (send* _object_state_dt __object_state_dt)
     (progn
       (if __object_state_dt (setq _object_state_dt (car __object_state_dt)))
       _object_state_dt)))
  (:id
   (&rest __id)
   (if (keywordp (car __id))
       (send* _id __id)
     (progn
       (if __id (setq _id (car __id)))
       _id)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; std_msgs/String _class
    (send _class :serialization-length)
    ;; std_msgs/String _status
    (send _status :serialization-length)
    ;; geometry_msgs/Pose2D _object_state_dt
    (send _object_state_dt :serialization-length)
    ;; std_msgs/Int16 _id
    (send _id :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; std_msgs/String _class
       (send _class :serialize s)
     ;; std_msgs/String _status
       (send _status :serialize s)
     ;; geometry_msgs/Pose2D _object_state_dt
       (send _object_state_dt :serialize s)
     ;; std_msgs/Int16 _id
       (send _id :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; std_msgs/String _class
     (send _class :deserialize buf ptr-) (incf ptr- (send _class :serialization-length))
   ;; std_msgs/String _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; geometry_msgs/Pose2D _object_state_dt
     (send _object_state_dt :deserialize buf ptr-) (incf ptr- (send _object_state_dt :serialization-length))
   ;; std_msgs/Int16 _id
     (send _id :deserialize buf ptr-) (incf ptr- (send _id :serialization-length))
   ;;
   self)
  )

(setf (get av_messages::object_ :md5sum-) "3d13e84609bddf054577bea1e144b7e8")
(setf (get av_messages::object_ :datatype-) "av_messages/object_")
(setf (get av_messages::object_ :definition-)
      "# A standard object message for perception

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



(provide :av_messages/object_ "3d13e84609bddf054577bea1e144b7e8")


