; Auto-generated. Do not edit!


(cl:in-package av_messages-msg)


;//! \htmlinclude object_.msg.html

(cl:defclass <object_> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (class
    :reader class
    :initarg :class
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (status
    :reader status
    :initarg :status
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (object_state_dt
    :reader object_state_dt
    :initarg :object_state_dt
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (id
    :reader id
    :initarg :id
    :type std_msgs-msg:Int16
    :initform (cl:make-instance 'std_msgs-msg:Int16)))
)

(cl:defclass object_ (<object_>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object_>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object_)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name av_messages-msg:<object_> is deprecated: use av_messages-msg:object_ instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <object_>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader av_messages-msg:position-val is deprecated.  Use av_messages-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'class-val :lambda-list '(m))
(cl:defmethod class-val ((m <object_>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader av_messages-msg:class-val is deprecated.  Use av_messages-msg:class instead.")
  (class m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <object_>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader av_messages-msg:status-val is deprecated.  Use av_messages-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'object_state_dt-val :lambda-list '(m))
(cl:defmethod object_state_dt-val ((m <object_>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader av_messages-msg:object_state_dt-val is deprecated.  Use av_messages-msg:object_state_dt instead.")
  (object_state_dt m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <object_>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader av_messages-msg:id-val is deprecated.  Use av_messages-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object_>) ostream)
  "Serializes a message object of type '<object_>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'class) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_state_dt) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'id) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object_>) istream)
  "Deserializes a message object of type '<object_>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'class) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_state_dt) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'id) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object_>)))
  "Returns string type for a message object of type '<object_>"
  "av_messages/object_")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_)))
  "Returns string type for a message object of type 'object_"
  "av_messages/object_")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object_>)))
  "Returns md5sum for a message object of type '<object_>"
  "3d13e84609bddf054577bea1e144b7e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object_)))
  "Returns md5sum for a message object of type 'object_"
  "3d13e84609bddf054577bea1e144b7e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object_>)))
  "Returns full string definition for message of type '<object_>"
  (cl:format cl:nil "# A standard object message for perception~%~%geometry_msgs/Point position # position in x, y ,z in meters~%std_msgs/String class # string for the actual object class, i.e. tree, pedestrian, car, truck, etc.~%~%std_msgs/String status # wether the object is static/dynamic in nature etc.~%geometry_msgs/Pose2D object_state_dt # Velocity in x, y, theta of a dynamic object~%~%std_msgs/Int16 id # unique object id for every object~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: std_msgs/Int16~%int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object_)))
  "Returns full string definition for message of type 'object_"
  (cl:format cl:nil "# A standard object message for perception~%~%geometry_msgs/Point position # position in x, y ,z in meters~%std_msgs/String class # string for the actual object class, i.e. tree, pedestrian, car, truck, etc.~%~%std_msgs/String status # wether the object is static/dynamic in nature etc.~%geometry_msgs/Pose2D object_state_dt # Velocity in x, y, theta of a dynamic object~%~%std_msgs/Int16 id # unique object id for every object~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: std_msgs/Int16~%int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object_>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'class))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_state_dt))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object_>))
  "Converts a ROS message object to a list"
  (cl:list 'object_
    (cl:cons ':position (position msg))
    (cl:cons ':class (class msg))
    (cl:cons ':status (status msg))
    (cl:cons ':object_state_dt (object_state_dt msg))
    (cl:cons ':id (id msg))
))
