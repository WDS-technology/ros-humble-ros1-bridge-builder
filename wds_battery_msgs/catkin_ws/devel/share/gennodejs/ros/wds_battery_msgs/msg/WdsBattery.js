// Auto-generated. Do not edit!

// (in-package wds_battery_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WdsBattery {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.voltage = null;
      this.current = null;
      this.charge = null;
      this.capacity = null;
      this.design_capacity = null;
      this.percentage = null;
      this.power_supply_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0.0;
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('charge')) {
        this.charge = initObj.charge
      }
      else {
        this.charge = 0.0;
      }
      if (initObj.hasOwnProperty('capacity')) {
        this.capacity = initObj.capacity
      }
      else {
        this.capacity = 0.0;
      }
      if (initObj.hasOwnProperty('design_capacity')) {
        this.design_capacity = initObj.design_capacity
      }
      else {
        this.design_capacity = 0.0;
      }
      if (initObj.hasOwnProperty('percentage')) {
        this.percentage = initObj.percentage
      }
      else {
        this.percentage = 0.0;
      }
      if (initObj.hasOwnProperty('power_supply_status')) {
        this.power_supply_status = initObj.power_supply_status
      }
      else {
        this.power_supply_status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WdsBattery
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.float32(obj.voltage, buffer, bufferOffset);
    // Serialize message field [current]
    bufferOffset = _serializer.float32(obj.current, buffer, bufferOffset);
    // Serialize message field [charge]
    bufferOffset = _serializer.float32(obj.charge, buffer, bufferOffset);
    // Serialize message field [capacity]
    bufferOffset = _serializer.float32(obj.capacity, buffer, bufferOffset);
    // Serialize message field [design_capacity]
    bufferOffset = _serializer.float32(obj.design_capacity, buffer, bufferOffset);
    // Serialize message field [percentage]
    bufferOffset = _serializer.float32(obj.percentage, buffer, bufferOffset);
    // Serialize message field [power_supply_status]
    bufferOffset = _serializer.uint8(obj.power_supply_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WdsBattery
    let len;
    let data = new WdsBattery(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current]
    data.current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [charge]
    data.charge = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [capacity]
    data.capacity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [design_capacity]
    data.design_capacity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [percentage]
    data.percentage = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [power_supply_status]
    data.power_supply_status = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'wds_battery_msgs/WdsBattery';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4958115a128b6f8273fcd7ed62879d62';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # WDS simplified battery status message
    # Contains only essential battery metrics for drone monitoring
    
    std_msgs/Header header
      # uint32 seq        # Included in Header
      # time stamp        # Included in Header
      # string frame_id   # Included in Header
    
    float32 voltage             # Voltage in Volts (V)
    float32 current             # Current in Amperes (A)
    float32 charge              # Remaining charge in Ampere-hours (Ah)
    float32 capacity            # Current battery capacity in Ampere-hours (Ah)
    float32 design_capacity     # Design capacity in Ampere-hours (Ah)
    float32 percentage          # Charge percentage (0-100)
    uint8 power_supply_status   # Power supply status (constants from sensor_msgs/BatteryState)
    
    # Power supply status constants (matching sensor_msgs/BatteryState)
    uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
    uint8 POWER_SUPPLY_STATUS_CHARGING = 1
    uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
    uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
    uint8 POWER_SUPPLY_STATUS_FULL = 4
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WdsBattery(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0.0
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.charge !== undefined) {
      resolved.charge = msg.charge;
    }
    else {
      resolved.charge = 0.0
    }

    if (msg.capacity !== undefined) {
      resolved.capacity = msg.capacity;
    }
    else {
      resolved.capacity = 0.0
    }

    if (msg.design_capacity !== undefined) {
      resolved.design_capacity = msg.design_capacity;
    }
    else {
      resolved.design_capacity = 0.0
    }

    if (msg.percentage !== undefined) {
      resolved.percentage = msg.percentage;
    }
    else {
      resolved.percentage = 0.0
    }

    if (msg.power_supply_status !== undefined) {
      resolved.power_supply_status = msg.power_supply_status;
    }
    else {
      resolved.power_supply_status = 0
    }

    return resolved;
    }
};

// Constants for message
WdsBattery.Constants = {
  POWER_SUPPLY_STATUS_UNKNOWN: 0,
  POWER_SUPPLY_STATUS_CHARGING: 1,
  POWER_SUPPLY_STATUS_DISCHARGING: 2,
  POWER_SUPPLY_STATUS_NOT_CHARGING: 3,
  POWER_SUPPLY_STATUS_FULL: 4,
}

module.exports = WdsBattery;
