#include "drone_test.hpp"

using namespace std::chrono_literals;

DroneTest::DroneTest() : Node("drone_test_node") {
    // Crear publisher con cola de 10 mensajes
    command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/drone_thrust_controller/commands", 10);
    
    // Configurar timer para publicar cada 100ms
    timer_ = this->create_wall_timer(
        100ms, std::bind(&DroneTest::publish_test_command, this));
    
    RCLCPP_INFO(this->get_logger(), "Test node initialized. Sending commands...");
}

void DroneTest::publish_test_command() {
    auto message = std_msgs::msg::Float64MultiArray();
    
    // Aplicar direcciones y velocidad base
    message.data.resize(4);
    for(size_t i = 0; i < 4; ++i) {
        message.data[i] = motor_directions_[i] * base_speed_;
    }
    
    // Publicar el comando
    command_publisher_->publish(message);
    
    // Loggear cada 10 publicaciones
    static int counter = 0;
    if(++counter % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "Published command: [%.1f, %.1f, %.1f, %.1f]",
                   message.data[0], message.data[1],
                   message.data[2], message.data[3]);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}