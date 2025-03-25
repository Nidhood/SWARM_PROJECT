#include "imu_channel_plugin.hh"
#include <gz/plugin/Register.hh>

// Register the plugin
GZ_ADD_PLUGIN (imu_channel_plugin::ImuChannelPlugin, gz::sim::System,
               imu_channel_plugin::ImuChannelPlugin::ISystemConfigure,
               imu_channel_plugin::ImuChannelPlugin::ISystemPreUpdate,
               imu_channel_plugin::ImuChannelPlugin::ISystemPostUpdate)

using namespace imu_channel_plugin;

ImuChannelPlugin::ImuChannelPlugin ()
{
    // Publish the IMU data topic (just for ros2):
    if (!rclcpp::ok ()) {
        rclcpp::init (0, nullptr);
    }
    ros2_node_ = std::make_shared<rclcpp::Node> ("imu_channel_plugin_node");
    ros2_pub_ = ros2_node_->create_publisher<sensor_msgs::msg::Imu> (
        ros2_imu_topic_, update_rate_);

    // Subscribe to the IMU gazebo topic:
    gz_node_.Subscribe (gz_imu_topic_, &ImuChannelPlugin::OnGazeboImuMsg, this);
}

ImuChannelPlugin::~ImuChannelPlugin ()
{
}

void ImuChannelPlugin::Configure (
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{

    // Load parameters from SDF:
    if (_sdf->HasElement ("ros2_topic")) {
        ros2_imu_topic_ = _sdf->Get<std::string> ("ros2_topic");
    }
    if (_sdf->HasElement ("gz_topic")) {
        gz_imu_topic_ = _sdf->Get<std::string> ("gz_topic");
    }
    if (_sdf->HasElement ("update_rate")) {
        update_rate_ = _sdf->Get<double> ("update_rate");
    }

    // Save the model in the model_ variable.
    model_ = gz::sim::Model (_entity);
    gz::sim::Entity link_ptr = model_.LinkByName (_ecm, "base_link");
    gz::sim::Entity linkEntity = model_.LinkByName (_ecm, "base_link");
    if (linkEntity == gz::sim::kNullEntity) {
        ignerr << "The base_link link was not found.\n";
        return;
    }
    model_link_ = linkEntity;
}

void ImuChannelPlugin::PreUpdate (const gz::sim::UpdateInfo &_info,
                                  gz::sim::EntityComponentManager &_ecm)
{
    // No pre-update processing is performed.
}

void ImuChannelPlugin::PostUpdate (const gz::sim::UpdateInfo &_info,
                                   const gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    // If a new message is available, publish it.
    if (new_msg_available_) {
        sensor_msgs::msg::Imu ros_imu;

        // Fill the header using the current time of the ROS2 node.
        ros_imu.header.stamp = ros2_node_->now ();
        ros_imu.header.frame_id = "base_link"; // Puedes parametrizarlo

        // If the Gazebo message has orientation, copy it; otherwise, set it to
        // identity.
        if (latest_gz_imu_msg_.has_orientation ()) {
            ros_imu.orientation.x = latest_gz_imu_msg_.orientation ().x ();
            ros_imu.orientation.y = latest_gz_imu_msg_.orientation ().y ();
            ros_imu.orientation.z = latest_gz_imu_msg_.orientation ().z ();
            ros_imu.orientation.w = latest_gz_imu_msg_.orientation ().w ();
        }
        else {
            ros_imu.orientation.x = 0.0;
            ros_imu.orientation.y = 0.0;
            ros_imu.orientation.z = 0.0;
            ros_imu.orientation.w = 1.0;
        }

        // Copy the angular velocity and linear acceleration data.
        ros_imu.angular_velocity.x =
            latest_gz_imu_msg_.angular_velocity ().x ();
        ros_imu.angular_velocity.y =
            latest_gz_imu_msg_.angular_velocity ().y ();
        ros_imu.angular_velocity.z =
            latest_gz_imu_msg_.angular_velocity ().z ();
        ros_imu.linear_acceleration.x =
            latest_gz_imu_msg_.linear_acceleration ().x ();
        ros_imu.linear_acceleration.y =
            latest_gz_imu_msg_.linear_acceleration ().y ();
        ros_imu.linear_acceleration.z =
            latest_gz_imu_msg_.linear_acceleration ().z ();

        // Publish the message in ROS2.
        ros2_pub_->publish (ros_imu);

        // Reset the flag.
        new_msg_available_ = false;
    }
}

void ImuChannelPlugin::OnGazeboImuMsg (const gz::msgs::IMU &_msg)
{
    // Save the latest IMU message:
    latest_gz_imu_msg_ = _msg;
    new_msg_available_ = true;
}