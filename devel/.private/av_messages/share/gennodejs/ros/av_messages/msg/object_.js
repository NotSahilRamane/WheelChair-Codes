// Auto-generated. Do not edit!

// (in-package av_messages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class object_ {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.class = null;
      this.status = null;
      this.object_state_dt = null;
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('class')) {
        this.class = initObj.class
      }
      else {
        this.class = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('object_state_dt')) {
        this.object_state_dt = initObj.object_state_dt
      }
      else {
        this.object_state_dt = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = new std_msgs.msg.Int16();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type object_
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [class]
    bufferOffset = std_msgs.msg.String.serialize(obj.class, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = std_msgs.msg.String.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [object_state_dt]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.object_state_dt, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = std_msgs.msg.Int16.serialize(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type object_
    let len;
    let data = new object_(null);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [class]
    data.class = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [object_state_dt]
    data.object_state_dt = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = std_msgs.msg.Int16.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.class);
    length += std_msgs.msg.String.getMessageSize(object.status);
    return length + 50;
  }

  static datatype() {
    // Returns string type for a message object
    return 'av_messages/object_';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d13e84609bddf054577bea1e144b7e8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new object_(null);
    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.class !== undefined) {
      resolved.class = std_msgs.msg.String.Resolve(msg.class)
    }
    else {
      resolved.class = new std_msgs.msg.String()
    }

    if (msg.status !== undefined) {
      resolved.status = std_msgs.msg.String.Resolve(msg.status)
    }
    else {
      resolved.status = new std_msgs.msg.String()
    }

    if (msg.object_state_dt !== undefined) {
      resolved.object_state_dt = geometry_msgs.msg.Pose2D.Resolve(msg.object_state_dt)
    }
    else {
      resolved.object_state_dt = new geometry_msgs.msg.Pose2D()
    }

    if (msg.id !== undefined) {
      resolved.id = std_msgs.msg.Int16.Resolve(msg.id)
    }
    else {
      resolved.id = new std_msgs.msg.Int16()
    }

    return resolved;
    }
};

module.exports = object_;
