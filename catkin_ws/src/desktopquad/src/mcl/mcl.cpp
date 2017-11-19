#include "mcl/mcl.h"

namespace mcl {

MCL::MCL() :
  nh_(ros::NodeHandle())
{
  ros::NodeHandle nh_private("~");

  // retrieve parameters from rosparam server
  params_.M = nh_private.param<int>("M", 100);
  if (!nh_private.getParam("platform_volume/x", params_.valid_x) ||
      !nh_private.getParam("platform_volume/y", params_.valid_y) ||
      !nh_private.getParam("platform_volume/z", params_.valid_z) ||
      !nh_private.getParam("platform_volume/R", params_.valid_R) ||
      !nh_private.getParam("platform_volume/P", params_.valid_P) ||
      !nh_private.getParam("platform_volume/Y", params_.valid_Y))
  {
    ROS_ERROR("[MCL] Please specify valid platform volume using the rosparam server.");
    ros::shutdown();
  }
  if (!nh_private.getParam("initial_volume/x", params_.initial_x) ||
      !nh_private.getParam("initial_volume/y", params_.initial_y) ||
      !nh_private.getParam("initial_volume/z", params_.initial_z) ||
      !nh_private.getParam("initial_volume/R", params_.initial_R) ||
      !nh_private.getParam("initial_volume/P", params_.initial_P) ||
      !nh_private.getParam("initial_volume/Y", params_.initial_Y))
  {
    ROS_ERROR("[MCL] Please specify initial particle volume using the rosparam server.");
    ros::shutdown();
  }

  // create map representation from rosparam server
  XmlRpc::XmlRpcValue xMap;
  if (nh_private.getParam("map", xMap)) {
    ROS_ASSERT(xMap.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    create_map(xMap);
  } else {
    ROS_ERROR("[MCL] Please load a map using the rosparam server.");
    ros::shutdown();
  }

  // Connect publishers and subscribers
  map_pub_ = nh_private.advertise<geometry_msgs::PoseArray>("map", 1);
  particles_pub_ = nh_private.advertise<geometry_msgs::PoseArray>("particles", 1);

  // initialize the particles
  init_particles();
}

// ----------------------------------------------------------------------------

void MCL::tick()
{


  publish_map();
  publish_particles();
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void MCL::create_map(XmlRpc::XmlRpcValue& xMap)
{
  ROS_INFO_STREAM("[MCL] Using ArUco map `" << xMap["name"] << "` with "
          << xMap["num_landmarks"] << " landmarks defined in the `"
          << xMap["frame_id"] << "` coordinate frame.");

  // set the coordinate frame that the map landmarks are defined in
  map_frame_ = (std::string)xMap["frame_id"];

  for (int i=0; i<xMap["landmarks"].size(); i++) {
    // Grab the XML RPC version of this landmark (struct) in the list
    auto xLandmark = xMap["landmarks"][i];

    // Grab the position list in the landmark
    auto xPosition = xLandmark["position"];
    double x = static_cast<double>(xPosition[0]);
    double y = static_cast<double>(xPosition[1]);
    double z = static_cast<double>(xPosition[2]);

    landmarks_.insert({xLandmark["id"], {x,y,z}});
  }
}

// ----------------------------------------------------------------------------

void MCL::init_particles()
{

  double xmax = params_.initial_x[0];
  double xmin = params_.initial_x[1];

  double ymax = params_.initial_y[0];
  double ymin = params_.initial_y[1];

  double zmax = params_.initial_z[0];
  double zmin = params_.initial_z[1];

  double Rmax = params_.initial_R[0];
  double Rmin = params_.initial_R[1];

  double Pmax = params_.initial_P[0];
  double Pmin = params_.initial_P[1];

  double Ymax = params_.initial_Y[0];
  double Ymin = params_.initial_Y[1];

  // initialize random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> xDis(xmin, xmax);
  std::uniform_real_distribution<> yDis(ymin, ymax);
  std::uniform_real_distribution<> zDis(zmin, zmax);
  std::uniform_real_distribution<> RDis(Rmin, Rmax);
  std::uniform_real_distribution<> PDis(Pmin, Pmax);
  std::uniform_real_distribution<> YDis(Ymin, Ymax);

  // reserve the memory for M new particles
  particles_.clear();
  particles_.reserve(params_.M);

  // Create M particles, uniformly sampled in the initial platform volume.
  for (int i=0; i<params_.M; i++) {
    particles_.push_back({
      {xDis(gen),yDis(gen),zDis(gen),RDis(gen),PDis(gen),YDis(gen)},
      0
    });
  }

  ROS_INFO_STREAM("[MCL] Initialized with " << params_.M << " particles.");
}

// ----------------------------------------------------------------------------

void MCL::sample_motion_model()
{

}

// ----------------------------------------------------------------------------

void MCL::perceptual_model()
{
  
}

// ----------------------------------------------------------------------------

void MCL::publish_map()
{
  geometry_msgs::PoseArray map_msg;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = map_frame_;


  for (auto& l: landmarks_) {
    geometry_msgs::Pose landmark;

    landmark.position.x = l.second.x;
    landmark.position.y = l.second.y;
    landmark.position.z = l.second.z;

    // Identity orientation
    landmark.orientation.w = 1;

    map_msg.poses.push_back(landmark);
  }


  map_pub_.publish(map_msg);
}

// ----------------------------------------------------------------------------

void MCL::publish_particles()
{
  geometry_msgs::PoseArray particles_msg;
  particles_msg.header.stamp = ros::Time::now();
  particles_msg.header.frame_id = map_frame_;

  for (auto& p: particles_) {
    geometry_msgs::Pose particle;

    particle.position.x = p.state.x;
    particle.position.y = p.state.y;
    particle.position.z = p.state.z;
    particle.orientation = tf::createQuaternionMsgFromRollPitchYaw(p.state.R, p.state.P, p.state.Y);

    particles_msg.poses.push_back(particle);
  }

  particles_pub_.publish(particles_msg);
}

}