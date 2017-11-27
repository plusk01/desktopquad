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

  // create the specified motion model
  int mm = nh_private.param<int>("motion_model", MM_SDNCV);
  if (mm == MM_SDNCV) {
    double x = nh_private.param<double>("sdncv/x", 0.01);
    double y = nh_private.param<double>("sdncv/y", 0.01);
    double z = nh_private.param<double>("sdncv/z", 0.01);
    double ax = nh_private.param<double>("sdncv/ax", 0.001);
    double ay = nh_private.param<double>("sdncv/ay", 0.001);
    double az = nh_private.param<double>("sdncv/az", 0.001);
    mm_ = std::make_shared<SDNCV>(x, y, z, ax, ay, az);
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
  meas_sub_ = nh_private.subscribe("measurements", 1, &MCL::measurements_cb, this);

  // initialize the particles
  init_particles();
}

// ----------------------------------------------------------------------------

void MCL::tick()
{
  //
  // Loop time management
  //

  static double now_d1 = 0;

  if (now_d1 == 0) {
    now_d1 = ros::Time::now().toSec();
    return; // since we can't know the dt yet
  }

  // Use the time from the last loop to calcualte dt
  double now = ros::Time::now().toSec();
  double dt = now - now_d1;
  now_d1 = now;

  //
  // Particle filter algorithm
  //

  // store the sum of all probability weights to normalize with
  double w_sum = 0;

  for (auto& p: particles_) {
    // Prediction
    mm_->sample(p, dt);

    // If there are measurements to process
    while (landmark_measurements_.size() > 0) {
      // Get the current landmark measurements (this is an array of marker measurements)
      auto Z = landmark_measurements_.front();

      // For each of the marker measurements: How good is this particle?
      // Aggregate the likelihood of this particle having seen all of these measurements
      p->w = 0;
      for (const auto &z : Z->poses) {
        p->w += perceptual_model(z, p);
      }

      // Undo log-space stuff
      p->w = std::exp(p->w);
      w_sum += p->w;

      // Destroy the element on the front of the queue
      landmark_measurements_.pop();
    }
  }

  resample(w_sum);

  //
  // Publish the MCL data
  //

  publish_map();
  publish_particles();
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void MCL::measurements_cb(const aruco_localization::MarkerMeasurementArrayConstPtr& msg)
{
  landmark_measurements_.push(msg);
}

// ----------------------------------------------------------------------------

void MCL::create_map(XmlRpc::XmlRpcValue& xMap)
{
  ROS_INFO_STREAM("[MCL] Using ArUco map `" << xMap["name"] << "` with "
          << xMap["num_landmarks"] << " landmarks defined in the `"
          << xMap["frame_id"] << "` coordinate frame.");

  // set the coordinate frame that the map landmarks are defined in
  working_frame_ = (std::string)xMap["frame_id"];

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
    particles_.emplace_back(std::make_shared<Particle>(
        // position (in working frame)
        xDis(gen), yDis(gen), zDis(gen),
        // orientation (from working to measurement frame)
        RDis(gen),PDis(gen),YDis(gen)
      ));
  }

  ROS_INFO_STREAM("[MCL] Initialized with " << params_.M << " particles.");
}

// ----------------------------------------------------------------------------

// This method returns the likelihood of a landmark measurement: p(z|x,m)
double MCL::perceptual_model(const aruco_localization::MarkerMeasurement& z, ParticlePtr& p)
{

  //
  // Data preparation
  //

  // Make sure that the received landmark measurement
  // corresponds to a landmark that exists in the map.
  auto search = landmarks_.find(z.aruco_id);
  if (search == landmarks_.end()) {
    ROS_ERROR_STREAM("[MCL] Landmark (" << z.aruco_id << ") does not exist in map.");
    return 0;
  }

  // form vector of landmark position (in the working frame)
  Eigen::Vector3d L_k;
  L_k << search->second.x, search->second.y, search->second.z;

  // extract Euler (3-2-1) angles from the landmark measurement
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(z.orientation, q);
  auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // [ yaw pitch roll ]

  // form the measurement vector
  Eigen::Matrix<double,6,1> zvec;
  zvec << z.position.x, z.position.y, z.position.z, euler(2), euler(1), euler(0);

  //
  // Measurement model
  //

  // given the state, what should the measurement be?
  Eigen::Vector3d zhat_pos = p->quat.toRotationMatrix() * (p->pos - L_k);
  Eigen::Vector3d zhat_eul = p->quat.toRotationMatrix().eulerAngles(2, 1, 0); // [ yaw pitch roll ]

  // combine the position and orientation components
  Eigen::Matrix<double,6,1> zhat;
  zhat << zhat_pos(0), zhat_pos(1), zhat_pos(2), zhat_eul(2), zhat_eul(1), zhat_eul(0);

  //
  // Calculate likelihood:p(z|x,m)
  //

  // compute the residual
  Eigen::Matrix<double,6,1> rvec = zvec - zhat;

  // build noise
  Eigen::Matrix<double,6,1> R_var;
  R_var << std::pow(0.1,2), std::pow(0.1,2), std::pow(0.1,2), std::pow(0.1,2), std::pow(0.1,2), std::pow(0.1,2);

  return logmvnpdf(zvec, zhat, R_var.asDiagonal());
}

// ----------------------------------------------------------------------------

void MCL::resample(double w_norm)
{

  // How many particles are there?
  int M = particles_.size();

  // sample a single uniform random number \in [0 1/M]
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1.0/M);
  double r = dis(gen);

  // initialize the selected particle and its (normalized) weight
  int i = 0;
  double c = (particles_[0]->w/w_norm);

  std::vector<ParticlePtr> particles;
  for (int m=0; m<M; m++) {

    // The current location of the 'pointer'
    double U = r + m/M;

    // Make sure that we are in the correct bin
    while (U > c) {
      i++;
      c += (particles_[i]->w/w_norm);
    }

    std::cout << "Selecting particle " << i << std::endl;

    // Add a copy of the ith particle to the new set
    particles.push_back(std::make_shared<Particle>(*particles_[i]));
  }

  std::cout << std::endl << std::endl;

  // Keep the newly resampled particles
  particles_.swap(particles);
}

// ----------------------------------------------------------------------------

double MCL::logmvnpdf(const Eigen::VectorXd& x, const Eigen::VectorXd& mu, const Eigen::MatrixXd& Sigma)
{
  // error checking
  if ((x.size() != mu.size()) || (x.size() != Sigma.rows())) {
    ROS_ERROR("[MCL::logmvnpdf] x and mu must be the same size!");
    return 0;
  }
  if (Sigma.rows() != Sigma.cols()) {
    ROS_ERROR("[MCL::logmvnpdf] Sigma must be square!");
    return 0;
  }

  // Dimension of mvn
  int k = Sigma.rows();

  // Kernel of a gaussian
  Eigen::VectorXd residual = x - mu;
  double mahal_sq = residual.transpose() * Sigma.inverse() * residual;

  // Log-likelihood function (natural log of a mvn pdf)
  return -0.5*( std::log(Sigma.determinant()) + mahal_sq + k*std::log(2*M_PI) );
}

// ----------------------------------------------------------------------------

double MCL::mvnpdf(const Eigen::VectorXd& x, const Eigen::VectorXd& mu, const Eigen::MatrixXd& Sigma)
{
  // error checking
  if ((x.size() != mu.size()) || (x.size() != Sigma.rows())) {
    ROS_ERROR("[MCL::mvnpdf] x and mu must be the same size!");
    return 0;
  }
  if (Sigma.rows() != Sigma.cols()) {
    ROS_ERROR("[MCL::mvnpdf] Sigma must be square!");
    return 0;
  }

  // Normalization constant for a multivariate Gaussian pdf
  int k = Sigma.rows();
  double eta = 1.0/std::sqrt( std::pow(2*M_PI,k) * Sigma.determinant() );

  // Kernel of a gaussian
  Eigen::VectorXd residual = x - mu;
  double gaussian = std::exp( -0.5 * residual.transpose() * Sigma.inverse() * residual );

  return eta*gaussian;
}

// ----------------------------------------------------------------------------

void MCL::publish_map()
{
  geometry_msgs::PoseArray map_msg;
  map_msg.header.stamp = ros::Time::now();
  map_msg.header.frame_id = working_frame_;


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
  particles_msg.header.frame_id = working_frame_;

  for (auto& p: particles_) {
    geometry_msgs::Pose particle;

    particle.position.x = p->pos(0);
    particle.position.y = p->pos(1);
    particle.position.z = p->pos(2);
    tf::quaternionEigenToMsg(p->quat, particle.orientation);

    particles_msg.poses.push_back(particle);
  }

  particles_pub_.publish(particles_msg);
}

}