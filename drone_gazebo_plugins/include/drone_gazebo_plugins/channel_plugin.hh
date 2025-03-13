#pragma once

#include <gz/msgs/float.pb.h> // Library for Gazebo messages.
#include <gz/sim/Link.hh> // Library for Gazebo simulation link, which is used to get the link of the model.
#include <gz/sim/Model.hh> // Library for Gazebo simulation model, which is used to get the model.
#include <gz/sim/System.hh> // Library for Gazebo simulation system, which is used to create plugins
#include <gz/sim/World.hh> // Library for Gazebo simulation world, which is used to get the world.
#include <gz/sim/config.hh>     // Library for Gazebo simulation configuration
#include <gz/transport/Node.hh> // Library for Gazebo transport node, WHICH IS THE MOST IMPORTANT LIBRARY FOR GAZEBO TO COMMUNICATE WITH ROS2
#include <gz/transport/Publisher.hh> // Library for Gazebo transport publisher, WHICH IS THE MOST IMPORTANT LIBRARY FOR GAZEBO TO PUBLISH WITH ROS2
#include <memory>                    // Library for shared_ptr
#include <rclcpp/rclcpp.hpp>         // Library for ROS2 cpp
#include <std_msgs/msg/float64.hpp>  // Library for ROS2 messages.
#include <string>

#include "gz/sim/components/AngularVelocity.hh" // Library for Gazebo simulation angular velocity, which is used to get the angular velocity of the model.
#include "gz/sim/components/LinearVelocity.hh" // Library for Gazebo simulation linear velocity, which is used to get the linear velocity of the model.
#include "gz/sim/components/Pose.hh" // Library for Gazebo simulation pose, which is used to get the pose of the model.
#include "gz/sim/components/World.hh" // Library for Gazebo simulation world, which is used to get the world.
#include <gz/common/Console.hh> // Library for Gazebo console, which is used to print messages when gazebo is running.

#include <gz/math/Pose3.hh> // Library for Gazebo math pose3, which is used to get the pose3 of the model.
#include <gz/math/Vector3.hh> // Library for Gazebo math vector3, which is used to get the vector3 of the model.
#include <gz/plugin/Register.hh> // Library for Gazebo plugin register, which is used to register the plugin.
#include <gz/sim/Util.hh> // Library for Gazebo simulation util, which is used to get the util of the model.

#include <gz/sim/Joint.hh> // Library for Gazebo simulation joint, which is used to get the joint of the model.
#include <trajectory_msgs/msg/joint_trajectory_point.hpp> // Library for ROS2 messages, which is used to get the joint trajectory point of the model.

#include <gz/msgs/joint_trajectory.pb.h>       // Gazebo JointTrajectory message
#include <gz/msgs/joint_trajectory_point.pb.h> // Gazebo JointTrajectoryPoint message
#include <gz/transport/AdvertiseOptions.hh> // For AdvertiseMessageOptions to disable lazy

class Channel : public gz::sim::System,
                public gz::sim::ISystemPostUpdate,
                public gz::sim::ISystemConfigure {
    // Life cycle of the plugin, can be:
    // - PreUpdate: Called before the simulation is updated.
    // - PostUpdate: Called after the simulation is updated.
    // - Configure: Called when the plugin is configured.
  public:
    // Called on every simulation iteration.
    void PostUpdate (const gz::sim::UpdateInfo &_info,
                     const gz::sim::EntityComponentManager &_ecm) override;

    void Configure (const gz::sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager & /*_eventMgr*/) override;

    // Callback for propeller command subscription.
    void PropellerCommandCallback (
        const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);

    // Callback for camera command subscription.
    void CameraCommandCallback (
        const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);

  private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_prop_;
    gz::transport::Node::Publisher pub_cam_;

    // Default topic names for state publishing.
    std::string propeller_trajectory_node{"propellers_trajectory"};
    std::string camera_360_trajectory_node{"camera_360_trajectory"};

    // Command topics to be read from SDF (or defaults)
    std::string topic_propeller_velocity_msgs{"propellers_velocity_topic"};
    std::string topic_camera_360_velocity_msgs{"camera_360_velocity_topic"};

    // ROS2 node and publishers for state publishing.
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>>
        trajectory_prop_pub_;
    std::shared_ptr<
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>>
        trajectory_cam_pub_;

    // ROS2 subscriptions for receiving command messages.
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
        prop_command_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
        camera_command_sub_;

    // Store the latest received command messages.
    std::optional<trajectory_msgs::msg::JointTrajectoryPoint>
        commanded_propeller_;
    std::optional<trajectory_msgs::msg::JointTrajectoryPoint> commanded_camera_;

    // The model entity.
    gz::sim::Entity model_;

    // The allowed joint names for each group (ordered as you want to map
    // commands)
    const std::vector<std::string> allowedPropellerJoints_ = {
        "propeller_FL_link_joint", "propeller_FR_link_joint",
        "propeller_BL_link_joint", "propeller_BR_link_joint"};

    const std::vector<std::string> allowedCameraJoints_ = {
        "camera_360_link_joint", "camera_joint_1_link_joint",
        "camera_joint_2_link_joint"};
};