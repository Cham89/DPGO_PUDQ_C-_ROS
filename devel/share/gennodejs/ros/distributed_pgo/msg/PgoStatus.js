// Auto-generated. Do not edit!

// (in-package distributed_pgo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PgoStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sender_robot_id = null;
      this.is_converged = null;
      this.current_grad_norm = null;
    }
    else {
      if (initObj.hasOwnProperty('sender_robot_id')) {
        this.sender_robot_id = initObj.sender_robot_id
      }
      else {
        this.sender_robot_id = 0;
      }
      if (initObj.hasOwnProperty('is_converged')) {
        this.is_converged = initObj.is_converged
      }
      else {
        this.is_converged = false;
      }
      if (initObj.hasOwnProperty('current_grad_norm')) {
        this.current_grad_norm = initObj.current_grad_norm
      }
      else {
        this.current_grad_norm = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PgoStatus
    // Serialize message field [sender_robot_id]
    bufferOffset = _serializer.int32(obj.sender_robot_id, buffer, bufferOffset);
    // Serialize message field [is_converged]
    bufferOffset = _serializer.bool(obj.is_converged, buffer, bufferOffset);
    // Serialize message field [current_grad_norm]
    bufferOffset = _serializer.float64(obj.current_grad_norm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PgoStatus
    let len;
    let data = new PgoStatus(null);
    // Deserialize message field [sender_robot_id]
    data.sender_robot_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [is_converged]
    data.is_converged = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [current_grad_norm]
    data.current_grad_norm = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'distributed_pgo/PgoStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60f16a25306673f726276922d7211a68';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 sender_robot_id
    bool is_converged
    float64 current_grad_norm
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PgoStatus(null);
    if (msg.sender_robot_id !== undefined) {
      resolved.sender_robot_id = msg.sender_robot_id;
    }
    else {
      resolved.sender_robot_id = 0
    }

    if (msg.is_converged !== undefined) {
      resolved.is_converged = msg.is_converged;
    }
    else {
      resolved.is_converged = false
    }

    if (msg.current_grad_norm !== undefined) {
      resolved.current_grad_norm = msg.current_grad_norm;
    }
    else {
      resolved.current_grad_norm = 0.0
    }

    return resolved;
    }
};

module.exports = PgoStatus;
