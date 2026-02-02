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

class PgoUpdate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sender_robot_id = null;
      this.local_vertex_index = null;
      this.pose_pudq = null;
      this.pose_vec3 = null;
    }
    else {
      if (initObj.hasOwnProperty('sender_robot_id')) {
        this.sender_robot_id = initObj.sender_robot_id
      }
      else {
        this.sender_robot_id = 0;
      }
      if (initObj.hasOwnProperty('local_vertex_index')) {
        this.local_vertex_index = initObj.local_vertex_index
      }
      else {
        this.local_vertex_index = 0;
      }
      if (initObj.hasOwnProperty('pose_pudq')) {
        this.pose_pudq = initObj.pose_pudq
      }
      else {
        this.pose_pudq = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('pose_vec3')) {
        this.pose_vec3 = initObj.pose_vec3
      }
      else {
        this.pose_vec3 = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PgoUpdate
    // Serialize message field [sender_robot_id]
    bufferOffset = _serializer.int32(obj.sender_robot_id, buffer, bufferOffset);
    // Serialize message field [local_vertex_index]
    bufferOffset = _serializer.int32(obj.local_vertex_index, buffer, bufferOffset);
    // Check that the constant length array field [pose_pudq] has the right length
    if (obj.pose_pudq.length !== 4) {
      throw new Error('Unable to serialize array field pose_pudq - length must be 4')
    }
    // Serialize message field [pose_pudq]
    bufferOffset = _arraySerializer.float64(obj.pose_pudq, buffer, bufferOffset, 4);
    // Check that the constant length array field [pose_vec3] has the right length
    if (obj.pose_vec3.length !== 3) {
      throw new Error('Unable to serialize array field pose_vec3 - length must be 3')
    }
    // Serialize message field [pose_vec3]
    bufferOffset = _arraySerializer.float64(obj.pose_vec3, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PgoUpdate
    let len;
    let data = new PgoUpdate(null);
    // Deserialize message field [sender_robot_id]
    data.sender_robot_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [local_vertex_index]
    data.local_vertex_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pose_pudq]
    data.pose_pudq = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [pose_vec3]
    data.pose_vec3 = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    return 64;
  }

  static datatype() {
    // Returns string type for a message object
    return 'distributed_pgo/PgoUpdate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bbcafeecfadfbe48488d6fc36fc38955';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 sender_robot_id
    int32 local_vertex_index
    float64[4] pose_pudq
    float64[3] pose_vec3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PgoUpdate(null);
    if (msg.sender_robot_id !== undefined) {
      resolved.sender_robot_id = msg.sender_robot_id;
    }
    else {
      resolved.sender_robot_id = 0
    }

    if (msg.local_vertex_index !== undefined) {
      resolved.local_vertex_index = msg.local_vertex_index;
    }
    else {
      resolved.local_vertex_index = 0
    }

    if (msg.pose_pudq !== undefined) {
      resolved.pose_pudq = msg.pose_pudq;
    }
    else {
      resolved.pose_pudq = new Array(4).fill(0)
    }

    if (msg.pose_vec3 !== undefined) {
      resolved.pose_vec3 = msg.pose_vec3;
    }
    else {
      resolved.pose_vec3 = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = PgoUpdate;
