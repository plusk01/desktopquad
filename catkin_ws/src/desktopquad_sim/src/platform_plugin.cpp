#include "desktopquad_sim/platform_plugin.h"

namespace gazebo
{

PlatformPlugin::PlatformPlugin() : ModelPlugin() {}

// ----------------------------------------------------------------------------

PlatformPlugin::~PlatformPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

// ----------------------------------------------------------------------------

void PlatformPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load PlatformPlugin");
    return;
  }
  ROS_INFO("Loaded the PlatformPlugin");

  // Configure Gazebo Integration
  model_ = _model;
  world_ = model_->GetWorld();

  // Load SDF parameters
  if (_sdf->HasElement("baseLink"))
  {
    base_link_ = _model->GetLink(_sdf->GetElement("baseLink")->Get<std::string>());
  }
  else
  {
    gzerr << "[PlatformPlugin] Please specify a baseLink";
  }


  // Connect Gazebo Update
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PlatformPlugin::OnUpdate, this, _1));

  // Initialize
  // this->Reset();
}

// ----------------------------------------------------------------------------

void PlatformPlugin::Reset()
{

}

// ----------------------------------------------------------------------------

void PlatformPlugin::OnUpdate(const common::UpdateInfo & _info)
{
    math::Pose inertial = base_link_->GetWorldPose();

    // Note the conversion from NWU to NED via the negative signs on y and z -- both on pos and rot.
    tf::Vector3 origin_ned = tf::Vector3(inertial.pos.x, -inertial.pos.y, -inertial.pos.z);
    tf::Quaternion quat_ned = tf::Quaternion(inertial.rot.x, -inertial.rot.y, -inertial.rot.z, inertial.rot.w);

    // Publish the transform to get from the world_ned (parent) frame to the platform_base (child) frame
    tf::StampedTransform transform;
    transform.setOrigin(origin_ned);
    transform.setRotation(quat_ned);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_ned", "platform_base"));
}

// ----------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN(PlatformPlugin);

} // namespace gazebo
