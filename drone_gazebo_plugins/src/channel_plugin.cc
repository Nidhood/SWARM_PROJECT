#include "channel_plugin.hh"

// This is required to register the plugin. Make sure the interfaces match what's in the header.
GZ_ADD_PLUGIN(Channel,
              gz::sim::System,
              gz::sim::ISystemPostUpdate,
              gz::sim::ISystemConfigure)

// This allows to use the plugin with the alias ChannelPlugin in the model file.
GZ_ADD_PLUGIN_ALIAS(Channel, "gz::sim::system::ChannelPlugin")

void Channel::Configure(const gz::sim::Entity &_entity,                  // _entity is the variable that contains the model information.
                        const std::shared_ptr<const sdf::Element> &_sdf, // _sdf is the variable that contains the sdf information.
                        gz::sim::EntityComponentManager &_ecm,           // _ecm is the variable that contains the entity component manager.
                        gz::sim::EventManager & /*_eventMgr*/)
{
  // Save the model in the model_ variable.
  model_ = _entity;

  // Initialize a variable with the model description.
  const gz::sim::Model my_model = gz::sim::Model(this->model_);
  ignmsg << "Model found: " << my_model.Name(_ecm) << std::endl;

  // Find the link with the name "base_link" in the model.
  gz::sim::Entity link_ptr = my_model.LinkByName(_ecm, "base_link");
  if (!link_ptr)
  {
    ignerr << "Link pointer not found" << std::endl;
    return;
  }

  // Access the link with the name "base_link" in the model.
  gz::sim::Link link_base = gz::sim::Link(link_ptr);

  // Get the joint with the name "propeller_FL_link_joint" in the model.
  gz::sim::Entity propeller_FL_link_joint_ptr = my_model.JointByName(_ecm, "propeller_FL_link_joint");
  if (!propeller_FL_link_joint_ptr)
  {
    ignerr << "Joint 'propeller_FL_link_joint' not found" << std::endl;
    return;
  }

  // Get the joint with the name "propeller_FR_link_joint" in the model.
  gz::sim::Entity propeller_FR_link_joint_ptr = my_model.JointByName(_ecm, "propeller_FR_link_joint");
  if (!propeller_FR_link_joint_ptr)
  {
    ignerr << "Joint 'propeller_FR_link_joint' not found" << std::endl;
    return;
  }

  // Get the joint with the name "propeller_BL_link_joint" in the model.
  gz::sim::Entity propeller_BL_link_joint_ptr = my_model.JointByName(_ecm, "propeller_BL_link_joint");
  if (!propeller_BL_link_joint_ptr)
  {
    ignerr << "Joint 'propeller_BL_link_joint' not found" << std::endl;
    return;
  }

  // Get the joint with the name "propeller_BR_link_joint" in the model.
  gz::sim::Entity propeller_BR_link_joint_ptr = my_model.JointByName(_ecm, "propeller_BR_link_joint");
  if (!propeller_BR_link_joint_ptr)
  {
    ignerr << "Joint 'propeller_BR_link_joint' not found" << std::endl;
    return;
  }

  // Now we have the link and the joints, we can get the pose of the link and the angular velocity of the joints.
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  // Start the ROS2 node for the propeller trajectory.
  this->ros_node_ = std::make_shared<rclcpp::Node>("channel_plugin_node");
  this->trajectory_pub_ = ros_node_->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(propeller_trajectory_node, 10);

}

// Here we implement the PostUpdate function, which is called at every iteration.
void Channel::PostUpdate(const gz::sim::UpdateInfo& _info,const gz::sim::EntityComponentManager& _ecm)
{
  {
    if (_info.paused)
      return;


    std::unordered_set<std::string> allowedJoints = {
      "propeller_FL_link_joint",
      "propeller_FR_link_joint",
      "propeller_BL_link_joint",
      "propeller_BR_link_joint",
      "camera_360_link_joint",
      "camera_joint_1_link_joint",
      "camera_joint_2_link_joint"
    };


    trajectory_msgs::msg::JointTrajectoryPoint traj_msg;

    gz::sim::Model model(this->model_);

    std::vector<gz::sim::Entity> joints = model.Joints(_ecm);

    for (auto &jointEntity : joints)
    {
      gz::sim::Joint joint(jointEntity);
      auto maybeName = joint.Name(_ecm);
      if (!maybeName.has_value())
        continue;
      std::string jointName = maybeName.value();

      if (allowedJoints.find(jointName) == allowedJoints.end())
        continue;
      auto maybeVelocity = joint.Velocity(_ecm);
      double velocity = maybeVelocity.has_value() ? maybeVelocity.value()[0] : 0.0;
      auto maybePosition = joint.Position(_ecm);
      double position = maybePosition.has_value() ? maybePosition.value()[0] : 0.0;
      double acceleration = 0.0;
      traj_msg.positions.push_back(position);
      traj_msg.velocities.push_back(velocity);
      traj_msg.accelerations.push_back(acceleration);
    }
    trajectory_pub_->publish(traj_msg);
  }
}