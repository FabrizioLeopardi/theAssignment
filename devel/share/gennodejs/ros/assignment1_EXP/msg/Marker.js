// Auto-generated. Do not edit!

// (in-package assignment1_EXP.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Marker {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.corners = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('corners')) {
        this.corners = initObj.corners
      }
      else {
        this.corners = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Marker
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    // Serialize message field [corners]
    // Serialize the length for message field [corners]
    bufferOffset = _serializer.uint32(obj.corners.length, buffer, bufferOffset);
    obj.corners.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Marker
    let len;
    let data = new Marker(null);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [corners]
    // Deserialize array length for message field [corners]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.corners = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.corners[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.corners.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'assignment1_EXP/Marker';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29e916ca9398fbd7b2424ca3f887845d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 id
    geometry_msgs/Point[] corners
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Marker(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.corners !== undefined) {
      resolved.corners = new Array(msg.corners.length);
      for (let i = 0; i < resolved.corners.length; ++i) {
        resolved.corners[i] = geometry_msgs.msg.Point.Resolve(msg.corners[i]);
      }
    }
    else {
      resolved.corners = []
    }

    return resolved;
    }
};

module.exports = Marker;
