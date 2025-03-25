#pragma once

#include <gz/msgs/imu.pb.h>          // Gazebo message for IMU
#include <gz/sim/Link.hh>            // Access to the link
#include <gz/sim/Model.hh>           // Access to the model
#include <gz/sim/System.hh>          // Inherit from System
#include <gz/sim/World.hh>           // Access to the world
#include <gz/sim/config.hh>          // Gazebo configuration
#include <gz/transport/Node.hh>      // Transport node for Gazebo
#include <gz/transport/Publisher.hh> // Gazebo publisher
#include <memory>                    // std::shared_ptr
#include <rclcpp/rclcpp.hpp>         // ROS2 node
#include <sensor_msgs/msg/imu.hpp>   // ROS2 message for IMU
#include <string>

namespace imu_channel_plugin {
class ImuChannelPlugin : public gz::sim::System,
                         public gz::sim::ISystemPostUpdate,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemConfigure {
  public:
    ImuChannelPlugin ();
    ~ImuChannelPlugin () override;
    void Configure (const gz::sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager & /*_eventMgr*/) override;
    void PreUpdate (const gz::sim::UpdateInfo &_info,
                    gz::sim::EntityComponentManager &_ecm) override;
    void PostUpdate (const gz::sim::UpdateInfo &_info,
                     const gz::sim::EntityComponentManager &_ecm) override;
    void OnGazeboImuMsg (const gz::msgs::IMU &_msg);

  private:
    // Variables to save the model and the link of the model:
    gz::sim::Model model_{gz::sim::kNullEntity};
    gz::sim::Entity model_link_{gz::sim::kNullEntity};

    // Ros2 node and publisher for IMU data:
    std::shared_ptr<rclcpp::Node> ros2_node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros2_pub_;

    // Configuration for the IMU plugin (Using SDF):
    std::string ros2_imu_topic_{"imu"};
    std::string gz_imu_topic_ = {"imu"};
    double update_rate_{10.0};

    // Gazebo node for communication:
    gz::transport::Node gz_node_;

    // Variables to save the IMU message in the cycle:
    gz::msgs::IMU latest_gz_imu_msg_;
    bool new_msg_available_{false};
};
} // namespace imu_channel_plugin