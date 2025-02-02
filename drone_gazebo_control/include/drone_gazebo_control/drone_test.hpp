#ifndef DRONE_TEST_HPP
#define DRONE_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

class DroneTest : public rclcpp::Node {
public:
    DroneTest();
    
private:
    void publish_test_command();
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Direcciones de los motores según configuración del URDF
    const std::array<int, 4> motor_directions_{-1, 1, 1, -1};
    const double base_speed_ = 2600.0;  // RPM
};

#endif // DRONE_TEST_HPP