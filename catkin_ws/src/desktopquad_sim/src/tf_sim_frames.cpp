#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_sim_frames");
    ros::NodeHandle nh;

    // Create transform listener and broadcaster
    tf::TransformListener listener;
    tf::TransformBroadcaster br;

    while (nh.ok()) {

        // Current time, to keep all tf's in sync
        ros::Time now = ros::Time::now();

        try {
            // Get the most recent ground_truth message from Gazebo/Odometry
            tf::StampedTransform transform;
            listener.waitForTransform("world", "/chiny/ground_truth", now, ros::Duration(1.0));
            listener.lookupTransform("world", "/chiny/ground_truth", now, transform);

            //
            // Link truth to ArUco Marker Map
            //

            tf::Vector3 origin = transform.getOrigin();
            origin.setZ( origin.z()-0.015);
            transform.setOrigin(origin);
            br.sendTransform(tf::StampedTransform(transform, now, "base", "chiny/truth"));

        } catch (tf::TransformException &ex) {
            ROS_ERROR("[tf_sim_frames]: %s", ex.what());
            ros::Duration(1.0).sleep(); // wait 1 second
        }
        
    }

    return 0;
}