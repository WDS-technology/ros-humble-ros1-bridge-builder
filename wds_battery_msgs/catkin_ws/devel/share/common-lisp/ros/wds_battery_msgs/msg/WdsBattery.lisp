; Auto-generated. Do not edit!


(cl:in-package wds_battery_msgs-msg)


;//! \htmlinclude WdsBattery.msg.html

(cl:defclass <WdsBattery> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0)
   (current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (charge
    :reader charge
    :initarg :charge
    :type cl:float
    :initform 0.0)
   (capacity
    :reader capacity
    :initarg :capacity
    :type cl:float
    :initform 0.0)
   (design_capacity
    :reader design_capacity
    :initarg :design_capacity
    :type cl:float
    :initform 0.0)
   (percentage
    :reader percentage
    :initarg :percentage
    :type cl:float
    :initform 0.0)
   (power_supply_status
    :reader power_supply_status
    :initarg :power_supply_status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WdsBattery (<WdsBattery>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WdsBattery>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WdsBattery)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wds_battery_msgs-msg:<WdsBattery> is deprecated: use wds_battery_msgs-msg:WdsBattery instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:header-val is deprecated.  Use wds_battery_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:voltage-val is deprecated.  Use wds_battery_msgs-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:current-val is deprecated.  Use wds_battery_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'charge-val :lambda-list '(m))
(cl:defmethod charge-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:charge-val is deprecated.  Use wds_battery_msgs-msg:charge instead.")
  (charge m))

(cl:ensure-generic-function 'capacity-val :lambda-list '(m))
(cl:defmethod capacity-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:capacity-val is deprecated.  Use wds_battery_msgs-msg:capacity instead.")
  (capacity m))

(cl:ensure-generic-function 'design_capacity-val :lambda-list '(m))
(cl:defmethod design_capacity-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:design_capacity-val is deprecated.  Use wds_battery_msgs-msg:design_capacity instead.")
  (design_capacity m))

(cl:ensure-generic-function 'percentage-val :lambda-list '(m))
(cl:defmethod percentage-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:percentage-val is deprecated.  Use wds_battery_msgs-msg:percentage instead.")
  (percentage m))

(cl:ensure-generic-function 'power_supply_status-val :lambda-list '(m))
(cl:defmethod power_supply_status-val ((m <WdsBattery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wds_battery_msgs-msg:power_supply_status-val is deprecated.  Use wds_battery_msgs-msg:power_supply_status instead.")
  (power_supply_status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WdsBattery>)))
    "Constants for message type '<WdsBattery>"
  '((:POWER_SUPPLY_STATUS_UNKNOWN . 0)
    (:POWER_SUPPLY_STATUS_CHARGING . 1)
    (:POWER_SUPPLY_STATUS_DISCHARGING . 2)
    (:POWER_SUPPLY_STATUS_NOT_CHARGING . 3)
    (:POWER_SUPPLY_STATUS_FULL . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WdsBattery)))
    "Constants for message type 'WdsBattery"
  '((:POWER_SUPPLY_STATUS_UNKNOWN . 0)
    (:POWER_SUPPLY_STATUS_CHARGING . 1)
    (:POWER_SUPPLY_STATUS_DISCHARGING . 2)
    (:POWER_SUPPLY_STATUS_NOT_CHARGING . 3)
    (:POWER_SUPPLY_STATUS_FULL . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WdsBattery>) ostream)
  "Serializes a message object of type '<WdsBattery>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'charge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'capacity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'design_capacity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'percentage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'power_supply_status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WdsBattery>) istream)
  "Deserializes a message object of type '<WdsBattery>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'charge) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'capacity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'design_capacity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'percentage) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'power_supply_status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WdsBattery>)))
  "Returns string type for a message object of type '<WdsBattery>"
  "wds_battery_msgs/WdsBattery")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WdsBattery)))
  "Returns string type for a message object of type 'WdsBattery"
  "wds_battery_msgs/WdsBattery")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WdsBattery>)))
  "Returns md5sum for a message object of type '<WdsBattery>"
  "4958115a128b6f8273fcd7ed62879d62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WdsBattery)))
  "Returns md5sum for a message object of type 'WdsBattery"
  "4958115a128b6f8273fcd7ed62879d62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WdsBattery>)))
  "Returns full string definition for message of type '<WdsBattery>"
  (cl:format cl:nil "# WDS simplified battery status message~%# Contains only essential battery metrics for drone monitoring~%~%std_msgs/Header header~%  # uint32 seq        # Included in Header~%  # time stamp        # Included in Header~%  # string frame_id   # Included in Header~%~%float32 voltage             # Voltage in Volts (V)~%float32 current             # Current in Amperes (A)~%float32 charge              # Remaining charge in Ampere-hours (Ah)~%float32 capacity            # Current battery capacity in Ampere-hours (Ah)~%float32 design_capacity     # Design capacity in Ampere-hours (Ah)~%float32 percentage          # Charge percentage (0-100)~%uint8 power_supply_status   # Power supply status (constants from sensor_msgs/BatteryState)~%~%# Power supply status constants (matching sensor_msgs/BatteryState)~%uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0~%uint8 POWER_SUPPLY_STATUS_CHARGING = 1~%uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2~%uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3~%uint8 POWER_SUPPLY_STATUS_FULL = 4~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WdsBattery)))
  "Returns full string definition for message of type 'WdsBattery"
  (cl:format cl:nil "# WDS simplified battery status message~%# Contains only essential battery metrics for drone monitoring~%~%std_msgs/Header header~%  # uint32 seq        # Included in Header~%  # time stamp        # Included in Header~%  # string frame_id   # Included in Header~%~%float32 voltage             # Voltage in Volts (V)~%float32 current             # Current in Amperes (A)~%float32 charge              # Remaining charge in Ampere-hours (Ah)~%float32 capacity            # Current battery capacity in Ampere-hours (Ah)~%float32 design_capacity     # Design capacity in Ampere-hours (Ah)~%float32 percentage          # Charge percentage (0-100)~%uint8 power_supply_status   # Power supply status (constants from sensor_msgs/BatteryState)~%~%# Power supply status constants (matching sensor_msgs/BatteryState)~%uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0~%uint8 POWER_SUPPLY_STATUS_CHARGING = 1~%uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2~%uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3~%uint8 POWER_SUPPLY_STATUS_FULL = 4~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WdsBattery>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WdsBattery>))
  "Converts a ROS message object to a list"
  (cl:list 'WdsBattery
    (cl:cons ':header (header msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':current (current msg))
    (cl:cons ':charge (charge msg))
    (cl:cons ':capacity (capacity msg))
    (cl:cons ':design_capacity (design_capacity msg))
    (cl:cons ':percentage (percentage msg))
    (cl:cons ':power_supply_status (power_supply_status msg))
))
