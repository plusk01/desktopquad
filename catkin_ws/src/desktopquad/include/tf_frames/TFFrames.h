#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geodesy/utm.h>

#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geographic_msgs/GeoPoint.h>

#include "nasa_s2d/SetUAVPose.h"

namespace tf_frames {

    class TFFrames
    {
    public:
        TFFrames();

    private:
        // Node handles, publishers, subscribers
        ros::NodeHandle nh_;
        ros::Subscriber sub_uav_odom_;
        ros::Subscriber sub_uav_fix_;

        // ROS params
        bool localize_ = false;

        // localization helpers
        bool localized_ = false;
        nav_msgs::OdometryPtr home_odom_;

        // ROS tf listener and broadcaster
        tf::TransformListener tf_listener_;
        tf::TransformBroadcaster tf_br_;

        // ROS service server
        ros::ServiceServer srv_uavpose_;

        // Heading override and position offset
        bool pose_override_ = false;
        double heading_;
        tf::Vector3 position_offset_;

        // Functions
        void cb_uav_odom(const nav_msgs::OdometryPtr& msg);
        void cb_uav_fix(const sensor_msgs::NavSatFix& msg);
        bool srv_set_pose(nasa_s2d::SetUAVPose::Request &req, nasa_s2d::SetUAVPose::Response &res);

        void send_transform(tf::Vector3& origin, tf::Quaternion& attitude);
        
    };

}