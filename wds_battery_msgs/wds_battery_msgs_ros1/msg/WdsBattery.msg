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
