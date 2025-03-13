#include "channel_plugin.hh"

// Register the plugin
GZ_ADD_PLUGIN (Channel, gz::sim::System, gz::sim::ISystemPostUpdate,
               gz::sim::ISystemConfigure)

// Register the plugin alias
GZ_ADD_PLUGIN_ALIAS (Channel, "gz::sim::system::ChannelPlugin")

//---------------------------------------------------------------------
// Configure Method
// Initializes the plugin by storing the model entity, printing the model name,
// and creating two ROS2 publishers for the propellers and camera trajectories.
//---------------------------------------------------------------------
void Channel::Configure (const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager & /*_eventMgr*/)
{

    // Save the model in the model_ variable.
    model_ = _entity;

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

    // Get the joint with the name "camera_360_joint" in the model.
    gz::sim::Entity camera_360_joint_ptr =
        my_model.JointByName (_ecm, "camera_360_link_joint");
    if (!camera_360_joint_ptr) {
        ignerr << "Joint 'camera_360_joint' not found" << std::endl;
        return;
    }

    // Get the joint with the name "camera_joint_1_link_joint" in the model.
    gz::sim::Entity camera_joint_1_link_joint_ptr =
        my_model.JointByName (_ecm, "camera_joint_1_link_joint");
    if (!camera_joint_1_link_joint_ptr) {
        ignerr << "Joint 'camera_joint_1_link_joint' not found" << std::endl;
        return;
    }

    // Get the joint with the name "camera_joint_2_link_joint" in the model.
    gz::sim::Entity camera_joint_2_link_joint_ptr =
        my_model.JointByName (_ecm, "camera_joint_2_link_joint");
    if (!camera_joint_2_link_joint_ptr) {
        ignerr << "Joint 'camera_joint_2_link_joint' not found" << std::endl;
        return;
    }

    // Get proppeller topic:
    if (_sdf->HasElement ("propellers_velocity")) {
        topic_propeller_velocity_msgs =
            _sdf->Get<std::string> ("propellers_velocity");
    }

    // Get camera topic:
    if (_sdf->HasElement ("camera_360_velocity")) {
        topic_camera_360_velocity_msgs =
            _sdf->Get<std::string> ("camera_360_velocity");
    }

    // Now we have the link and the joints, we can get the pose of the link and
    // the angular velocity of the joints.
    if (!rclcpp::ok ()) {
        rclcpp::init (0, nullptr);
    }

    // Start the ROS2 node for the plugin.
    this->ros_node_ = std::make_shared<rclcpp::Node> ("channel_plugin_node");

    // Create the state publishers.
    this->trajectory_prop_pub_ =
        ros_node_
            ->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint> (
                propeller_trajectory_node, 10);
    this->trajectory_cam_pub_ =
        ros_node_
            ->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint> (
                camera_360_trajectory_node, 10);

    // Create advertisement options for the propeller and camera topics.
    this->pub_prop_ = node_.Advertise<gz::msgs::JointTrajectory> (
        "/propellers_velocity_topic");
    this->pub_cam_ = node_.Advertise<gz::msgs::JointTrajectory> (
        "/camera_360_velocity_topic");

    ignmsg << "Advertising Gazebo topics: /propellers_velocity_topic and "
              "/camera_360_velocity_topic"
           << std::endl;
}

//---------------------------------------------------------------------
// PostUpdate Method
// 1) Publishes joint state to ROS 2 (existing logic).
// 2) Applies commands if any (existing logic).
// 3) Publishes minimal gz::msgs::JointTrajectory on two separate Gazebo topics.
//---------------------------------------------------------------------
void Channel::PostUpdate (const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    // 1) Build trajectory messages for ROS 2
    trajectory_msgs::msg::JointTrajectoryPoint traj_prop_msg;
    trajectory_msgs::msg::JointTrajectoryPoint traj_cam_msg;

    gz::sim::Model model (model_);
    std::vector<gz::sim::Entity> joints = model.Joints (_ecm);

    for (auto &jointEntity : joints) {
        gz::sim::Joint joint (jointEntity);
        auto maybeName = joint.Name (_ecm);
        if (!maybeName.has_value ())
            continue;
        std::string jointName = maybeName.value ();

        double position = 0.0, velocity = 0.0;
        auto maybePosition = joint.Position (_ecm);
        if (maybePosition.has_value () && !maybePosition.value ().empty ())
            position = maybePosition.value ()[0];
        auto maybeVelocity = joint.Velocity (_ecm);
        if (maybeVelocity.has_value () && !maybeVelocity.value ().empty ())
            velocity = maybeVelocity.value ()[0];
        double acceleration = 0.0; // default

        // If joint belongs to propeller group
        if (std::find (allowedPropellerJoints_.begin (),
                       allowedPropellerJoints_.end (),
                       jointName) != allowedPropellerJoints_.end ()) {
            traj_prop_msg.positions.push_back (position);
            traj_prop_msg.velocities.push_back (velocity);
            traj_prop_msg.accelerations.push_back (acceleration);
        }
        // If joint belongs to camera group
        else if (std::find (allowedCameraJoints_.begin (),
                            allowedCameraJoints_.end (),
                            jointName) != allowedCameraJoints_.end ()) {
            traj_cam_msg.positions.push_back (position);
            traj_cam_msg.velocities.push_back (velocity);
            traj_cam_msg.accelerations.push_back (acceleration);
        }
    }

    // Publish to ROS 2 if data is available
    if (!traj_prop_msg.positions.empty ())
        trajectory_prop_pub_->publish (traj_prop_msg);
    if (!traj_cam_msg.positions.empty ())
        trajectory_cam_pub_->publish (traj_cam_msg);

    // 2) Command handling
    if (commanded_propeller_.has_value ()) {
        const auto &cmd = commanded_propeller_.value ();
        if (cmd.velocities.size () >= allowedPropellerJoints_.size ()) {
            for (size_t i = 0; i < allowedPropellerJoints_.size (); ++i) {
                std::string jointName = allowedPropellerJoints_[i];
                gz::sim::Entity jointEntity =
                    model.JointByName (_ecm, jointName);
                if (jointEntity == gz::sim::kNullEntity) {
                    ignerr << "Propeller joint " << jointName
                           << " not found for command update." << std::endl;
                    continue;
                }
                gz::sim::Joint joint (jointEntity);
                std::vector<double> commandVelocity = {cmd.velocities[i]};
                // Update joint velocity
                joint.SetVelocity (
                    const_cast<gz::sim::EntityComponentManager &> (_ecm),
                    commandVelocity);
            }
        }
    }

    if (commanded_camera_.has_value ()) {
        const auto &cmd = commanded_camera_.value ();
        if (cmd.positions.size () >= allowedCameraJoints_.size ()) {
            for (size_t i = 0; i < allowedCameraJoints_.size (); ++i) {
                std::string jointName = allowedCameraJoints_[i];
                gz::sim::Entity jointEntity =
                    model.JointByName (_ecm, jointName);
                if (jointEntity == gz::sim::kNullEntity) {
                    ignerr << "Camera joint " << jointName
                           << " not found for command update." << std::endl;
                    continue;
                }
                gz::sim::Joint joint (jointEntity);
                std::vector<double> commandPosition = {cmd.positions[i]};
                // Update joint position
                joint.ResetPosition (
                    const_cast<gz::sim::EntityComponentManager &> (_ecm),
                    commandPosition);
            }
        }
    }

    // 3) Publish minimal JointTrajectory to each Gazebo topic
    {
        // For propellers
        gz::msgs::JointTrajectory gzPropTraj;
        auto *point = gzPropTraj.add_points ();
        point->add_positions (0.1);  // example
        point->add_velocities (0.2); // example
        this->pub_prop_.Publish (gzPropTraj);
    }
    {
        // For camera
        gz::msgs::JointTrajectory gzCamTraj;
        auto *point = gzCamTraj.add_points ();
        point->add_positions (1.0);  // example
        point->add_velocities (0.0); // example
        this->pub_cam_.Publish (gzCamTraj);
    }
}

//---------------------------------------------------------------------
// Callback for propeller command subscription.
// Stores the received propeller command message.
//---------------------------------------------------------------------
void Channel::PropellerCommandCallback (
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    commanded_propeller_ = *msg;
    ignmsg << "Received propeller command" << std::endl;
}

//---------------------------------------------------------------------
// Callback for camera command subscription.
// Stores the received camera command message.
//---------------------------------------------------------------------
void Channel::CameraCommandCallback (
    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
{
    commanded_camera_ = *msg;
    ignmsg << "Received camera command" << std::endl;
}