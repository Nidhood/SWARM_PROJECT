#include "rotors_command_plugin.hh"

#include <actuator_msgs/msg/actuators.hpp>
#include <gz/common/Console.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/AdvertiseOptions.hh>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <thread>

// Register the plugin
GZ_ADD_PLUGIN (rotors_command_plugin::RotorsCommandPlugin, gz::sim::System,
               rotors_command_plugin::RotorsCommandPlugin::ISystemConfigure,
               rotors_command_plugin::RotorsCommandPlugin::ISystemPreUpdate,
               rotors_command_plugin::RotorsCommandPlugin::ISystemPostUpdate)

using namespace rotors_command_plugin;

RotorsCommandPlugin::RotorsCommandPlugin ()
{

    // Publish the rotors command topic.
    gz_pub_ = this->gz_node_.Advertise<gz::msgs::Actuators> (gz_command_topic_);
    if (!rclcpp::ok ()) {
        rclcpp::init (0, nullptr);
    }
    ros2_node_ = std::make_shared<rclcpp::Node> ("rotors_command_plugin_node");
    ros2_pub_ = ros2_node_->create_publisher<actuator_msgs::msg::Actuators> (
        ros2_command_topic_, 10);
    ros2_sub_ = ros2_node_->create_subscription<actuator_msgs::msg::Actuators> (
        ros2_command_topic_, 10,
        std::bind (&RotorsCommandPlugin::Ros2CommandCallback, this,
                   std::placeholders::_1));
}

RotorsCommandPlugin::~RotorsCommandPlugin ()
{
}
//---------------------------------------------------------------------
// Configure Method
// Initializes the plugin by storing the model entity, printing the model name,
// and creating two ROS2 publishers for the propellers and camera trajectories.
//---------------------------------------------------------------------
void RotorsCommandPlugin::Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{

    // Save the model in the model_ variable.
    model_ = gz::sim::Model (_entity);

    // Initialize a variable with the model description.
    const gz::sim::Model my_model = gz::sim::Model (this->model_);
    ignmsg << "Model found: " << my_model.Name (_ecm) << std::endl;

    // Find the link with the name "base_link" in the model.
    gz::sim::Entity link_ptr = my_model.LinkByName (_ecm, "base_link");
    if (!link_ptr) {
        ignerr << "Link pointer not found" << std::endl;
        return;
    }

    // Access the link with the name "base_link" in the model.
    gz::sim::Link link_base = gz::sim::Link (link_ptr);

    // Get the joint with the name "propeller_FL_link_joint" in the model.
    gz::sim::Entity propeller_FL_link_joint_ptr =
        my_model.JointByName (_ecm, "propeller_FL_link_joint");
    if (!propeller_FL_link_joint_ptr) {
        ignerr << "Joint 'propeller_FL_link_joint' not found" << std::endl;
        return;
    }

    // Get the joint with the name "propeller_FR_link_joint" in the model.
    gz::sim::Entity propeller_FR_link_joint_ptr =
        my_model.JointByName (_ecm, "propeller_FR_link_joint");
    if (!propeller_FR_link_joint_ptr) {
        ignerr << "Joint 'propeller_FR_link_joint' not found" << std::endl;
        return;
    }

    // Get the joint with the name "propeller_BL_link_joint" in the model.
    gz::sim::Entity propeller_BL_link_joint_ptr =
        my_model.JointByName (_ecm, "propeller_BL_link_joint");
    if (!propeller_BL_link_joint_ptr) {
        ignerr << "Joint 'propeller_BL_link_joint' not found" << std::endl;
        return;
    }

    // Get the joint with the name "propeller_BR_link_joint" in the model.
    gz::sim::Entity propeller_BR_link_joint_ptr =
        my_model.JointByName (_ecm, "propeller_BR_link_joint");
    if (!propeller_BR_link_joint_ptr) {
        ignerr << "Joint 'propeller_BR_link_joint' not found" << std::endl;
        return;
    }

    // Get rotors command topic:
    if (!_sdf->HasElement (gz_command_topic_)) {
        ignerr << "Missing " << gz_command_topic_ << " element" << std::endl;
    }
}

void RotorsCommandPlugin::PreUpdate (const gz::sim::UpdateInfo &_info,
                                     gz::sim::EntityComponentManager &_ecm)
{
    // No pre-update processing is performed.
}

// PostUpdate Method
void RotorsCommandPlugin::PostUpdate (
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    if (!actuator_msg_.position.empty () || !actuator_msg_.velocity.empty () ||
        !actuator_msg_.normalized.empty ()) {
        gz::msgs::Actuators gz_msg;
        for (const auto &pos : actuator_msg_.position)
            gz_msg.add_velocity (pos); // Por ejemplo, usamos los valores de
        gz_pub_.Publish (gz_msg);
        ros2_pub_->publish (actuator_msg_);
    }
}

// ROS2 callback for rotors command subscription.
void RotorsCommandPlugin::Ros2CommandCallback (
    const actuator_msgs::msg::Actuators::SharedPtr msg)
{
    // Assign the received message to the lastCommand_ variable.
    actuator_msg_ = *msg;

    // Send the received message to the Gazebo topic.
    gz::msgs::Actuators gz_msg;
    for (const auto &pos : msg->position)
        gz_msg.add_velocity (pos);
    gz_pub_.Publish (gz_msg);
}