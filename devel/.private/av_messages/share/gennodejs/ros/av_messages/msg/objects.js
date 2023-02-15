// Auto-generated. Do not edit!

// (in-package av_messages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let object_ = require('./object_.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class objects {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.object_detections = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('object_detections')) {
        this.object_detections = initObj.object_detections
      }
      else {
        this.object_detections = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type objects
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [object_detections]
    // Serialize the length for message field [object_detections]
    bufferOffset = _serializer.uint32(obj.object_detections.length, buffer, bufferOffset);
    obj.object_detections.forEach((val) => {
      bufferOffset = object_.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type objects
    let len;
    let data = new objects(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [object_detections]
    // Deserialize array length for message field [object_detections]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.object_detections = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.object_detections[i] = object_.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.object_detections.forEach((val) => {
      length += object_.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'av_messages/objects';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29c2393b11acc79632a74bbd645a8019';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new objects(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.object_detections !== undefined) {
      resolved.object_detections = new Array(msg.object_detections.length);
      for (let i = 0; i < resolved.object_detections.length; ++i) {
        resolved.object_detections[i] = object_.Resolve(msg.object_detections[i]);
      }
    }
    else {
      resolved.object_detections = []
    }

    return resolved;
    }
};

module.exports = objects;
