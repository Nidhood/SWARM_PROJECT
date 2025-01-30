#include "drone_controller.hpp"

DroneController::DroneController() : Node("drone_controller") 
{
    // Initialize publishers
    fl_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/fl_propeller_controller/commands", 10);
    fr_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/fr_propeller_controller/commands", 10);
    bl_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/bl_propeller_controller/commands", 10);
    br_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/br_propeller_controller/commands", 10);

    // Create timer for periodic control (100Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&DroneController::control_callback, this));
}

void DroneController::publish_velocity(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& publisher,
    double velocity)
{
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {velocity};
    publisher->publish(msg);
}

void DroneController::control_callback()
{
    // Example control logic - you can modify this as needed
    static double test_velocity = 500.0;  // RPM
    
    // Publish the same velocity to all propellers
    publish_velocity(fl_pub_, test_velocity);
    publish_velocity(fr_pub_, -test_velocity);  // Note: opposite direction for diagonal propellers
    publish_velocity(bl_pub_, test_velocity);
    publish_velocity(br_pub_, -test_velocity);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}