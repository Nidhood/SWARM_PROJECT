#include "rotors_command_plugin.hh"
#include <actuator_msgs/msg/actuators.hpp>
#include <functional>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// Register the plugin with Gazebo.
GZ_ADD_PLUGIN (rotors_command_plugin::RotorsCommandPlugin, gz::sim::System,
               rotors_command_plugin::RotorsCommandPlugin::ISystemConfigure,
               rotors_command_plugin::RotorsCommandPlugin::ISystemPreUpdate,
               rotors_command_plugin::RotorsCommandPlugin::ISystemPostUpdate)

namespace rotors_command_plugin {

RotorsCommandPlugin::RotorsCommandPlugin ()
{
    // Initialize Gazebo publisher on the specified command topic.
    gz_pub_ = gz_node_.Advertise<gz::msgs::Actuators> (gz_command_topic_);

    // Initialize ROS2 node (if not already initialized) and create
    // subscription.
    if (!rclcpp::ok ()) {
        rclcpp::init (0, nullptr);
    }
    ros2_node_ = std::make_shared<rclcpp::Node> ("rotors_command_plugin_node");
    ros2_sub_ = ros2_node_->create_subscription<actuator_msgs::msg::Actuators> (
        ros2_command_topic_, 10,
        std::bind (&RotorsCommandPlugin::Ros2CommandCallback, this,
                   std::placeholders::_1));

    // Spin ROS2 in a separate thread to process callbacks.
    std::thread spin_thread ([this] () { rclcpp::spin (this->ros2_node_); });
    spin_thread.detach ();
}

RotorsCommandPlugin::~RotorsCommandPlugin ()
{
}

void RotorsCommandPlugin::Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
    // (Optional) Load SDF parameters if provided.
    if (_sdf->HasElement ("ros2_topic")) {
        ros2_command_topic_ = _sdf->Get<std::string> ("ros2_topic");
    }
    if (_sdf->HasElement ("gz_topic")) {
        gz_command_topic_ = _sdf->Get<std::string> ("gz_topic");
    }
    // (Optional) Additional configuration can ir here.
}

void RotorsCommandPlugin::PreUpdate (const gz::sim::UpdateInfo &_info,
                                     gz::sim::EntityComponentManager &_ecm)
{
    // No pre-update processing required.
}

void RotorsCommandPlugin::PostUpdate (
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    // No post-update processing required.
}

void RotorsCommandPlugin::Ros2CommandCallback (
    const actuator_msgs::msg::Actuators::SharedPtr msg)
{
    // Update the internal command message (if needed).
    command_msg_ = *msg;

    // Convert the ROS2 message to a Gazebo message.
    gz::msgs::Actuators gz_msg;
    // Here, we assume that the "velocity" field carries the rotor speed
    // commands.
    for (const auto &val : msg->velocity) {
        gz_msg.add_velocity (val);
    }

    // Publish the converted message to the Gazebo topic.
    gz_pub_.Publish (gz_msg);
}

} // namespace rotors_command_plugin
