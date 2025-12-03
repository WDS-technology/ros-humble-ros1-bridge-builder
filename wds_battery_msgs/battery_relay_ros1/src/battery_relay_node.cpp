/**
 * @file battery_relay_node.cpp
 * @brief ROS1 node that converts sensor_msgs/BatteryState to wds_battery_msgs/WdsBattery
 *
 * This node subscribes to /corvus/powermodule/battery (sensor_msgs/BatteryState)
 * and republishes the essential fields to /wds/battery (wds_battery_msgs/WdsBattery)
 * for bridging to ROS2.
 */

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <wds_battery_msgs/WdsBattery.h>

class BatteryRelay {
private:
    ros::NodeHandle nh_;
    ros::Subscriber battery_sub_;
    ros::Publisher wds_battery_pub_;

    std::string input_topic_;
    std::string output_topic_;

    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
        // Create WdsBattery message with simplified fields
        wds_battery_msgs::WdsBattery wds_msg;

        // Copy header
        wds_msg.header = msg->header;

        // Copy essential battery metrics
        wds_msg.voltage = msg->voltage;
        wds_msg.current = msg->current;
        wds_msg.charge = msg->charge;
        wds_msg.capacity = msg->capacity;
        wds_msg.design_capacity = msg->design_capacity;
        wds_msg.percentage = msg->percentage;
        wds_msg.power_supply_status = msg->power_supply_status;

        // Publish the converted message
        wds_battery_pub_.publish(wds_msg);

        ROS_DEBUG_THROTTLE(5.0, "Battery relay: %.2fV, %.2f%%, Status: %d",
                          wds_msg.voltage, wds_msg.percentage, wds_msg.power_supply_status);
    }

public:
    BatteryRelay() : nh_("~") {
        // Get topic names from parameters (with defaults)
        nh_.param<std::string>("input_topic", input_topic_, "/corvus/powermodule/battery");
        nh_.param<std::string>("output_topic", output_topic_, "/wds/battery");

        // Subscribe to standard BatteryState message
        battery_sub_ = nh_.subscribe<sensor_msgs::BatteryState>(
            input_topic_, 10, &BatteryRelay::batteryCallback, this);

        // Publish WdsBattery message
        wds_battery_pub_ = nh_.advertise<wds_battery_msgs::WdsBattery>(output_topic_, 10);

        ROS_INFO("Battery relay node started");
        ROS_INFO("  Input:  %s (sensor_msgs/BatteryState)", input_topic_.c_str());
        ROS_INFO("  Output: %s (wds_battery_msgs/WdsBattery)", output_topic_.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wds_battery_relay_node");

    try {
        BatteryRelay relay;
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Battery relay node exception: %s", e.what());
        return 1;
    }

    return 0;
}
