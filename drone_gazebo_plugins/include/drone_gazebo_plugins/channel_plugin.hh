#pragma once

#include <string>
#include <memory>                       // Library for shared_ptr
#include <rclcpp/rclcpp.hpp>            // Library for ROS2 cpp 
#include <gz/sim/System.hh>             // Library for Gazebo simulation system, which is used to create plugins
#include <gz/sim/config.hh>             // Library for Gazebo simulation configuration
#include <gz/transport/Node.hh>         // Library for Gazebo transport node, WHICH IS THE MOST IMPORTANT LIBRARY FOR GAZEBO TO COMMUNICATE WITH ROS2
#include <gz/transport/Publisher.hh>    // Library for Gazebo transport publisher, WHICH IS THE MOST IMPORTANT LIBRARY FOR GAZEBO TO PUBLISH WITH ROS2
#include <gz/msgs/float.pb.h>           // Library for Gazebo messages.
#include <std_msgs/msg/float64.hpp>     // Library for ROS2 messages.
#include <gz/sim/Link.hh>               // Library for Gazebo simulation link, which is used to get the link of the model.
#include <gz/sim/Model.hh>              // Library for Gazebo simulation model, which is used to get the model.
#include <gz/sim/World.hh>              // Library for Gazebo simulation world, which is used to get the world.

#include <gz/common/Console.hh>                     // Library for Gazebo console, which is used to print messages when gazebo is running.
#include "gz/sim/components/AngularVelocity.hh"     // Library for Gazebo simulation angular velocity, which is used to get the angular velocity of the model.
#include "gz/sim/components/LinearVelocity.hh"      // Library for Gazebo simulation linear velocity, which is used to get the linear velocity of the model.
#include "gz/sim/components/Pose.hh"                // Library for Gazebo simulation pose, which is used to get the pose of the model.
#include "gz/sim/components/World.hh"               // Library for Gazebo simulation world, which is used to get the world.

#include <gz/plugin/Register.hh>                    // Library for Gazebo plugin register, which is used to register the plugin.
#include <gz/math/Vector3.hh>                       // Library for Gazebo math vector3, which is used to get the vector3 of the model.
#include <gz/math/Pose3.hh>                         // Library for Gazebo math pose3, which is used to get the pose3 of the model.
#include <gz/sim/Util.hh>                           // Library for Gazebo simulation util, which is used to get the util of the model.

#include <trajectory_msgs/msg/joint_trajectory_point.hpp> // Library for ROS2 messages, which is used to get the joint trajectory point of the model.
#include <gz/sim/Joint.hh>

class Channel : public gz::sim::System, public gz::sim::ISystemPostUpdate, public gz::sim::ISystemConfigure
{
    // Life cycle of the plugin, can be:
    // - PreUpdate: Called before the simulation is updated.
    // - PostUpdate: Called after the simulation is updated.
    // - Configure: Called when the plugin is configured.
public:
    void PostUpdate(const gz::sim::UpdateInfo& _info,const gz::sim::EntityComponentManager& _ecm) override;
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override;

private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_;
    std::string propeller_trajectory_node{"propellers_trajectory"};
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>> trajectory_pub_;
    gz::sim::Entity model_;
};