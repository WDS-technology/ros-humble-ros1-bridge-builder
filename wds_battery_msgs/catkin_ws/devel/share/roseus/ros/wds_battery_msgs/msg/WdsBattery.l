;; Auto-generated. Do not edit!


(when (boundp 'wds_battery_msgs::WdsBattery)
  (if (not (find-package "WDS_BATTERY_MSGS"))
    (make-package "WDS_BATTERY_MSGS"))
  (shadow 'WdsBattery (find-package "WDS_BATTERY_MSGS")))
(unless (find-package "WDS_BATTERY_MSGS::WDSBATTERY")
  (make-package "WDS_BATTERY_MSGS::WDSBATTERY"))

(in-package "ROS")
;;//! \htmlinclude WdsBattery.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*POWER_SUPPLY_STATUS_UNKNOWN*" (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(shadow '*POWER_SUPPLY_STATUS_UNKNOWN* (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(defconstant wds_battery_msgs::WdsBattery::*POWER_SUPPLY_STATUS_UNKNOWN* 0)
(intern "*POWER_SUPPLY_STATUS_CHARGING*" (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(shadow '*POWER_SUPPLY_STATUS_CHARGING* (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(defconstant wds_battery_msgs::WdsBattery::*POWER_SUPPLY_STATUS_CHARGING* 1)
(intern "*POWER_SUPPLY_STATUS_DISCHARGING*" (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(shadow '*POWER_SUPPLY_STATUS_DISCHARGING* (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(defconstant wds_battery_msgs::WdsBattery::*POWER_SUPPLY_STATUS_DISCHARGING* 2)
(intern "*POWER_SUPPLY_STATUS_NOT_CHARGING*" (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(shadow '*POWER_SUPPLY_STATUS_NOT_CHARGING* (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(defconstant wds_battery_msgs::WdsBattery::*POWER_SUPPLY_STATUS_NOT_CHARGING* 3)
(intern "*POWER_SUPPLY_STATUS_FULL*" (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(shadow '*POWER_SUPPLY_STATUS_FULL* (find-package "WDS_BATTERY_MSGS::WDSBATTERY"))
(defconstant wds_battery_msgs::WdsBattery::*POWER_SUPPLY_STATUS_FULL* 4)
(defclass wds_battery_msgs::WdsBattery
  :super ros::object
  :slots (_header _voltage _current _charge _capacity _design_capacity _percentage _power_supply_status ))

(defmethod wds_battery_msgs::WdsBattery
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:voltage __voltage) 0.0)
    ((:current __current) 0.0)
    ((:charge __charge) 0.0)
    ((:capacity __capacity) 0.0)
    ((:design_capacity __design_capacity) 0.0)
    ((:percentage __percentage) 0.0)
    ((:power_supply_status __power_supply_status) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _voltage (float __voltage))
   (setq _current (float __current))
   (setq _charge (float __charge))
   (setq _capacity (float __capacity))
   (setq _design_capacity (float __design_capacity))
   (setq _percentage (float __percentage))
   (setq _power_supply_status (round __power_supply_status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:charge
   (&optional __charge)
   (if __charge (setq _charge __charge)) _charge)
  (:capacity
   (&optional __capacity)
   (if __capacity (setq _capacity __capacity)) _capacity)
  (:design_capacity
   (&optional __design_capacity)
   (if __design_capacity (setq _design_capacity __design_capacity)) _design_capacity)
  (:percentage
   (&optional __percentage)
   (if __percentage (setq _percentage __percentage)) _percentage)
  (:power_supply_status
   (&optional __power_supply_status)
   (if __power_supply_status (setq _power_supply_status __power_supply_status)) _power_supply_status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _voltage
    4
    ;; float32 _current
    4
    ;; float32 _charge
    4
    ;; float32 _capacity
    4
    ;; float32 _design_capacity
    4
    ;; float32 _percentage
    4
    ;; uint8 _power_supply_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current
       (sys::poke _current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _charge
       (sys::poke _charge (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _capacity
       (sys::poke _capacity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _design_capacity
       (sys::poke _design_capacity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _percentage
       (sys::poke _percentage (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _power_supply_status
       (write-byte _power_supply_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _voltage
     (setq _voltage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current
     (setq _current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _charge
     (setq _charge (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _capacity
     (setq _capacity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _design_capacity
     (setq _design_capacity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _percentage
     (setq _percentage (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _power_supply_status
     (setq _power_supply_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get wds_battery_msgs::WdsBattery :md5sum-) "4958115a128b6f8273fcd7ed62879d62")
(setf (get wds_battery_msgs::WdsBattery :datatype-) "wds_battery_msgs/WdsBattery")
(setf (get wds_battery_msgs::WdsBattery :definition-)
      "# WDS simplified battery status message
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

")



(provide :wds_battery_msgs/WdsBattery "4958115a128b6f8273fcd7ed62879d62")


