#ifndef FLOWER_OBSERVER_H
#define FLOWER_OBSERVER_H

// Include cpp important headers
#include <math.h>
#include <stdio.h>
#include <chrono>
#include <thread>

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2/LinearMath/Quaternion.h>

// Message includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

class FlowerObserver
{
public:
  //! Constructor.
  FlowerObserver(ros::NodeHandle & nh);

  //! Node name
  std::string node_name_;
  int counter_;
  int max_counter_;

  void setModelState();

private:
  //! Node Handle
  ros::NodeHandle & nh_;

  //! Publisher
  ros::Publisher pubPosition;

  //! Client
  ros::ServiceClient clientSetModelState;

  //! Methods
  void discretizeSpace();

  //! Spherical Coordinates.
  std::vector<double> r_, theta_, phi_;

  //! Discretization Limits and Parameters.
  double r_max_, r_min_, r_n_;
  double theta_max_, theta_min_, theta_n_;
  double phi_max_, phi_min_, phi_n_;

  //! Cartesian Coordinates and Orientation
  double x_, y_, z_; // position
  std::vector<double> normal_ = {0,0,1}; // normal vector
  double q1_, q2_, q3_, q4_; // rotation quaternion
};

#endif // FLOWER_OBSERVER_H