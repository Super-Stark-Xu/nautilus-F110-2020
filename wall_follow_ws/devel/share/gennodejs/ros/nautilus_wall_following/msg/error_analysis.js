// Auto-generated. Do not edit!

// (in-package nautilus_wall_following.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class error_analysis {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.avg_error = null;
      this.max_error = null;
    }
    else {
      if (initObj.hasOwnProperty('avg_error')) {
        this.avg_error = initObj.avg_error
      }
      else {
        this.avg_error = 0.0;
      }
      if (initObj.hasOwnProperty('max_error')) {
        this.max_error = initObj.max_error
      }
      else {
        this.max_error = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type error_analysis
    // Serialize message field [avg_error]
    bufferOffset = _serializer.float32(obj.avg_error, buffer, bufferOffset);
    // Serialize message field [max_error]
    bufferOffset = _serializer.float32(obj.max_error, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type error_analysis
    let len;
    let data = new error_analysis(null);
    // Deserialize message field [avg_error]
    data.avg_error = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_error]
    data.max_error = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nautilus_wall_following/error_analysis';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02c9ccf4f42efdb87083aa4f2e65fcfe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 avg_error
    float32 max_error
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new error_analysis(null);
    if (msg.avg_error !== undefined) {
      resolved.avg_error = msg.avg_error;
    }
    else {
      resolved.avg_error = 0.0
    }

    if (msg.max_error !== undefined) {
      resolved.max_error = msg.max_error;
    }
    else {
      resolved.max_error = 0.0
    }

    return resolved;
    }
};

module.exports = error_analysis;
