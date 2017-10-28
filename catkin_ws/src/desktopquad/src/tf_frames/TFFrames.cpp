#include "tf_frames/TFFrames.h"

namespace tf_frames {

TFFrames::TFFrames() :
    nh_(ros::NodeHandle())
{
    ros::NodeHandle nh_private("~");

    // Get ROS params
    double bH = nh_.param<double>("platform/base/height", 0);
    double rL = nh_.param<double>("platform/rods/length", 0);
    double tH = nh_.param<double>("platform/top/height", 0);
    double aH = nh_.param<double>("platform/aruco/height", 0);

    // calculate the height of the platform, from top of the base
    // to bottom of the ArUco map on the top -- i.e., the flyable region.
    platform_height_ = rL - bH - tH - aH;
}

// ----------------------------------------------------------------------------

void TFFrames::tick()
{
    //
    // Link platform_base to aruco_map
    //

    double z_ned = -platform_height_;

    // Publish the transform to get from the platform_base (parent) frame to the aruco_map (child) frame
    tf::StampedTransform transform;
    transform.setOrigin(tf::Vector3(0, 0, z_ned));
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "platform_base", "aruco_map"));
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

}