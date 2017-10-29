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

  //
  // Configure Gazebo Integration
  //

  model_ = _model;
  world_ = model_->GetWorld();

  //
  // Get elements from the robot urdf/sdf file
  //

  if (_sdf->HasElement("baseHeight"))
    base_height_ = _sdf->GetElement("baseHeight")->Get<double>();
  else
    ROS_ERROR("[PlatformPlugin] Please specify a baseHeight.");

  if (_sdf->HasElement("baseLink"))
    base_link_ = _model->GetLink(_sdf->GetElement("baseLink")->Get<std::string>());
  else
    ROS_ERROR("[PlatformPlugin] Please specify a baseLink");

  // Connect Gazebo Update
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PlatformPlugin::OnUpdate, this, _1));
}

// ----------------------------------------------------------------------------

void PlatformPlugin::OnUpdate(const common::UpdateInfo & _info)
{

  // Create a transform object to be used throughout
  tf::StampedTransform transform;

  //
  // Link the inertial NWU world to world_NED
  //

  transform.setIdentity();
  tf::Quaternion qNWU2NED; qNWU2NED.setRPY(M_PI, 0.0, 0.0);
  transform.setRotation(qNWU2NED);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world_NED"));

  //
  // Link world (NWU) frame to platform (NWU)
  //

  math::Pose inertial = base_link_->GetWorldPose();

  // The inertial pose gets us to the bottom of the platform base. Since the
  // platform coordinate system starts on the top-side of the base, we add
  // the height (thickness) of the base to the z in the NWU Gazebo frame.
  double z_nwu = inertial.pos.z + base_height_;

  tf::Vector3 origin_nwu = tf::Vector3(inertial.pos.x, inertial.pos.y, z_nwu);
  tf::Quaternion quat_nwu = tf::Quaternion(inertial.rot.x, inertial.rot.y, inertial.rot.z, inertial.rot.w);

  // Publish the transform to get from the PARENT world (NWU) frame to the CHILD platform (NWU) frame
  transform.setIdentity();
  transform.setOrigin(origin_nwu);
  transform.setRotation(quat_nwu);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "platform"));
}

// ----------------------------------------------------------------------------

GZ_REGISTER_MODEL_PLUGIN(PlatformPlugin);

} // namespace gazebo