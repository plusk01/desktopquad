#include "mcl/mcl.h"

namespace mcl {

MCL::MCL() :
  nh_(ros::NodeHandle())
{
  ros::NodeHandle nh_private("~");

  // retrieve parameters from rosparam server
  params_.M = nh_private.param<int>("M", 100);

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

  init();
}

// ----------------------------------------------------------------------------

void MCL::tick()
{

  publish_map();
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

void MCL::init()
{

  double xmax = 10;
  double xmin = 5;

  // Create M particles, uniformly sampled in the platform volume.
  Eigen::VectorXd xx = VectorXd_rand(params_.M, xmin, xmax);
  Eigen::VectorXd yy = VectorXd_rand(params_.M, xmin, xmax);
  Eigen::VectorXd zz = VectorXd_rand(params_.M, xmin, xmax);
  Eigen::VectorXd RR = VectorXd_rand(params_.M, xmin, xmax);
  Eigen::VectorXd PP = VectorXd_rand(params_.M, xmin, xmax);
  Eigen::VectorXd YY = VectorXd_rand(params_.M, xmin, xmax);


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

// http://wiki.ros.org/bfl/Tutorials/Example%20of%20using%20a%20particle%20filter%20for%20localization%20by%20bfl%20library
void MCL::publish_particles()
{
//   geometry_msgs::PoseArray particles_msg;
//   particles_msg.header.stamp = ros::Time::now();
//   particles_msg.header.frame_id = "/map";

//   vector<WeightedSample<ColumnVector> >::iterator sample_it;
//   vector<WeightedSample<ColumnVector> > samples;

//   samples = filter->getNewSamples();

//   for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
//   {
//     geometry_msgs::Pose pose;
//     ColumnVector sample = (*sample_it).ValueGet();

//     pose.position.x = sample(1);
//     pose.position.y = sample(2);
//     pose.orientation.z = sample(3);

//     particles_msg.poses.insert(particles_msg.poses.begin(), pose);
//   }
//   particle_pub.publish(particles_msg);
}

// ----------------------------------------------------------------------------

Eigen::VectorXd MCL::VectorXd_rand(int length, double low, double high)
{
  Eigen::VectorXd V = 0.5*Eigen::VectorXd::Random(length);
  V = (high-low)*(Eigen::VectorXd::Constant(length, 0.5) + V);
  V = (V + Eigen::VectorXd::Constant(length, low));

  return V;
}

}