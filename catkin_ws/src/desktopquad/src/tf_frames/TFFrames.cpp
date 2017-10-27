#include "tf_frames/tf_frames.h"

namespace tf_frames {

TFFrames::TFFrames() :
    nh_(ros::NodeHandle())
{
    ros::NodeHandle nh_private("~");

    // Get ROS params
    localize_ = nh_private.param<bool>("localize", false);

    // Set up Publishers and Subscribers based on parameters
    sub_uav_fix_ = nh_.subscribe("uav_gps_fix", 1, &TFFrames::cb_uav_fix, this);
    sub_uav_odom_ = nh_.subscribe("uav_odom", 1, &TFFrames::cb_uav_odom, this);

    // ROS Services
    srv_uavpose_ = nh_private.advertiseService("set_uav_pose", &TFFrames::srv_set_pose, this);
}

// ----------------------------------------------------------------------------

bool TFFrames::srv_set_pose(nasa_s2d::SetUAVPose::Request &req, nasa_s2d::SetUAVPose::Response &res)
{
    pose_override_ = true;

    // Allow the user to set the UAV heading
    // Heading w.r.t ENU inertial frame (i.e., heading is zero at East)
    heading_ = req.heading_enu * (M_PI/180);

    // Provide an offset to the position of the UAV
    // Position is w.r.t ENU inertial frame
    position_offset_ = tf::Vector3(req.offset_enu.x, req.offset_enu.y, req.offset_enu.z);
}

// ----------------------------------------------------------------------------

void TFFrames::cb_uav_odom(const nav_msgs::OdometryPtr& msg)
{
    // Put current pose into tf data structures
    tf::Vector3 origin(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    if (localize_) {

        // set the home position if not set before
        if (!localized_) {
            home_odom_ = msg;
            localized_ = true;
        }

        origin.setX(msg->pose.pose.position.x - home_odom_->pose.pose.position.x);
        origin.setY(msg->pose.pose.position.y - home_odom_->pose.pose.position.y);

        // origin.setX(msg->pose.pose.position.y - home_odom_->pose.pose.position.y);
        // origin.setY(msg->pose.pose.position.x - home_odom_->pose.pose.position.x);
    }

    // Send the transform that connects the fcu to the base_link
    send_transform(origin, quat);
}

// ----------------------------------------------------------------------------

void TFFrames::send_transform(tf::Vector3& origin, tf::Quaternion& attitude)
{
    // Allow the user to override the heading and add a position offset to the tf
    if (pose_override_) {

        //
        // Heading
        //

        // Convert the attitude quaternion to Euler angles
        double phi, theta, psi;
        tf::Matrix3x3(attitude).getRPY(phi, theta, psi);

        // Heading w.r.t ENU inertial frame (i.e., heading is zero at East)
        if (std::abs(heading_) < 2*M_PI)
            psi = heading_;

        // Set the new attitude with specified heading
        attitude.setRPY(phi, theta, psi);

        //
        // Position Offset
        //

        // Origin is w.r.t ENU inertial frame
        origin += position_offset_;
    }

    // Translate and rotate into the body (base_link) frame -- remember that this is all in ENU instead of NED
    tf::Transform transform;
    transform.setOrigin(origin);
    transform.setRotation(attitude);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", "base_link")); // base_link is in ROS Body (REP 103) -- body NWU
}

// ----------------------------------------------------------------------------

void TFFrames::cb_uav_fix(const sensor_msgs::NavSatFix& msg)
{
    // Create an identity transform
    tf::Transform transform;
    transform.setIdentity();

    // Convert lat/lon to UTM
    // http://answers.ros.org/question/50763/need-help-converting-lat-long-coordinates-into-meters/?answer=257140#post-id-257140
    geographic_msgs::GeoPoint geo_pt;
    geo_pt.latitude = msg.latitude;
    geo_pt.longitude = msg.longitude;
    geo_pt.altitude = msg.altitude;
    geodesy::UTMPoint utm_pt(geo_pt);

    // Link the fcu/map origin to the origin of this UTM zone.
    transform.setOrigin(tf::Vector3(-utm_pt.easting, -utm_pt.northing, 0));
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fcu", "utm_origin_" + std::to_string(utm_pt.zone) + utm_pt.band));
}

}