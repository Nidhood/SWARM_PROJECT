#pragma once

#include <actuator_msgs/msg/actuators.hpp> // ROS2 message for actuator commands
#include <gz/msgs/actuators.pb.h> // Gazebo message for actuator commands
#include <gz/sim/System.hh>       // Base class for Gazebo systems
#include <gz/transport/Node.hh>   // Gazebo transport node
#include <memory>
#include <rclcpp/rclcpp.hpp> // ROS2 C++ client library
#include <string>

#include <gz/plugin/Register.hh>

namespace rotors_command_plugin {

class RotorsCommandPlugin : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemPostUpdate {
  public:
    RotorsCommandPlugin ();
    ~RotorsCommandPlugin () override;

    // Configure the plugin using SDF parameters.
    void Configure (const gz::sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager & /*_eventMgr*/) override;

    // Called before the simulation update (not used here).
    void PreUpdate (const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) override;

    // Called after the simulation update (not used here).
    void PostUpdate (const gz::sim::UpdateInfo &_info,
                     const gz::sim::EntityComponentManager &_ecm) override;

    // ROS2 callback for actuator command messages.
    void
    Ros2CommandCallback (const actuator_msgs::msg::Actuators::SharedPtr msg);

  private:
    // Gazebo transport node and publisher.
    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher gz_pub_;

    // ROS2 node, publisher and subscriber.
    std::shared_ptr<rclcpp::Node> ros2_node_;
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr ros2_sub_;

    // Topic names (configurable via SDF).
    std::string ros2_command_topic_{"rotors_speed"};
    std::string gz_command_topic_{"swarm_drone/gazebo/command/motor_speed"};

    // Buffer to store the latest command message (if needed).
    actuator_msgs::msg::Actuators command_msg_;
};

} // namespace rotors_command_plugin
