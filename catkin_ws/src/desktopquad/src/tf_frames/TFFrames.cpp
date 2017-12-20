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
    // Create a transform that will be used throughout
    tf::StampedTransform transform;


    //
    // Link platform (NWU) to platform_NED
    //

    transform.setIdentity();
    tf::Quaternion qNWU2NED; qNWU2NED.setRPY(M_PI, 0.0, 0.0);
    transform.setRotation(qNWU2NED);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "platform", "platform_NED"));


    //
    // Link platform_NED to the map (on the top of the DesktopQuad Platform)
    //

    double z_ned = -platform_height_;

    // Publish the transform to get from the platform_NED (parent) frame to the map (child) frame
    transform.setIdentity();
    transform.setOrigin(tf::Vector3(0, 0, z_ned));
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "platform_NED", "map"));


    //
    // Link map to the ArUco Marker Map
    //

    // Publish the transform to get from the map (parent) frame to the aruco (child) frame
    transform.setIdentity();
    // transform.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion qmap2aruco; qmap2aruco.setRPY(0.0, 0.0, M_PI/2);
    transform.setRotation(qmap2aruco);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco"));


    //
    // Link camera to the quad body
    //

    transform.setIdentity();
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0));
    tf::Quaternion q; q.setRPY(M_PI, 0, 0);
    transform.setRotation(q);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "body"));
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

}