#pragma once

#include <actuator_msgs/msg/actuators.hpp> // ROS2 message for rotors command
#include <algorithm>
#include <gz/msgs/actuators.pb.h>    // Gazebo message for rotors command
#include <gz/sim/Link.hh>            // Access to the link
#include <gz/sim/Model.hh>           // Access to the model
#include <gz/sim/System.hh>          // Inherit from System
#include <gz/sim/World.hh>           // Access to the world
#include <gz/sim/config.hh>          // Gazebo configuration
#include <gz/transport/Node.hh>      // Transport node for Gazebo
#include <gz/transport/Publisher.hh> // Gazebo publisher
#include <memory>                    // std::shared_ptr
#include <rclcpp/rclcpp.hpp>         // ROS2 node
#include <string>
#include <vector>

#include "gz/sim/Util.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Pose.hh"
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

namespace rotors_command_plugin {
class RotorsCommandPlugin : public gz::sim::System,
                            public gz::sim::ISystemPostUpdate,
                            public gz::sim::ISystemPreUpdate,
                            public gz::sim::ISystemConfigure {
    // Life cycle of the plugin, can be:
    // - PreUpdate: Called before the simulation is updated.
    // - PostUpdate: Called after the simulation is updated.
    // - Configure: Called when the plugin is configured.
  public:
    RotorsCommandPlugin ();
    ~RotorsCommandPlugin () override;

    void Configure (const gz::sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager & /*_eventMgr*/) override;

    void PreUpdate (const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate (const gz::sim::UpdateInfo &_info,
                     const gz::sim::EntityComponentManager &_ecm) override;

    // Callback for rotors command subscription.
    void
    Ros2CommandCallback (const actuator_msgs::msg::Actuators::SharedPtr msg);

  private:
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity model_link_{gz::sim::kNullEntity};
    gz::transport::Node gz_node_;
    std::shared_ptr<rclcpp::Node> ros2_node_;
    gz::transport::Node::Publisher gz_pub_;
    actuator_msgs::msg::Actuators actuator_msg_;

    // Topic for rotors command:
    std::string ros2_command_topic_{"rotors_speed"};
    std::string gz_command_topic_ = {"/swarm_drone/gazebo/command/motor_speed"};
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr ros2_pub_;
    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr ros2_sub_;

    // The allowed joint names for each group (ordered as you want to map
    // commands)
    const std::vector<std::string> allowedPropellerJoints_ = {
        "propeller_FL_link_joint", "propeller_FR_link_joint",
        "propeller_BL_link_joint", "propeller_BR_link_joint"};
};
} // namespace rotors_command_plugin
