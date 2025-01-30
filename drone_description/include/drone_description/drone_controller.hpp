#ifndef DRONE_CONTROLLER_HPP_
#define DRONE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class DroneController : public rclcpp::Node {
public:
    DroneController();

private:
    // Publishers for each propeller
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fr_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bl_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr br_pub_;

    // Timer for periodic control
    rclcpp::TimerBase::SharedPtr timer_;

    // Control function
    void control_callback();

    // Helper function to publish velocities
    void publish_velocity(
        const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& publisher,
        double velocity);
};

#endif  // DRONE_CONTROLLER_HPP_