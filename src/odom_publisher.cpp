/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "odom_publisher");

  ros::Publisher laser_odom_publisher_;
  ros::Publisher ugv_odom_publisher_;
  ros::NodeHandle node_handle_;

  laser_odom_publisher_=
      node_handle_.advertise<nav_msgs::Odometry>("LaserOdomTopic", 100);
  ugv_odom_publisher_  =
      node_handle_.advertise<nav_msgs::Odometry>("UgvOdomTopic", 100);

  // configuring parameters
  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub;


   //create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(0.1));

  ros::Rate rate(publish_frequency);
  while (node_handle_.ok())
  {
    tf::StampedTransform transform;
    try
    {
     listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

      // construct a pose message
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

      static nav_msgs::Odometry odom;
      static nav_msgs::Odometry last_odom;
      last_odom=odom;
      //odom.header.stamp = 0;
      odom.header.frame_id = "/map";
      odom.pose.pose.position = pose_stamped.pose.position;
      odom.pose.pose.orientation = pose_stamped.pose.orientation;
      odom.header.stamp = ros::Time::now();

      double secs_now = (ros::Time::now()).toSec();
      double secs_last = last_odom.header.stamp.toSec();
      double secs_duration=secs_now-secs_last;
      odom.twist.twist.linear.x=(last_odom.pose.pose.position.x-odom.pose.pose.position.x)/secs_duration;
      odom.twist.twist.linear.y=(last_odom.pose.pose.position.y-odom.pose.pose.position.y)/secs_duration;
      odom.twist.twist.linear.z=(last_odom.pose.pose.position.z-odom.pose.pose.position.z)/secs_duration;
	
  
        p_pub.publish(odom);
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
