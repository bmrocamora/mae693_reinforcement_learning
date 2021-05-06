#include "flower_observer/flower_observer.h"

/*--------------------------------------------------------------------
 * FlowerObserver()
 * Constructor.
 *------------------------------------------------------------------*/

FlowerObserver::FlowerObserver(ros::NodeHandle & nh)
  : nh_(nh)
{
  clientSetModelState = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  pubPosition = nh_.advertise<geometry_msgs::Point>("flower_position", 1);

  node_name_ = "flower_observer";
  if (ros::param::get(node_name_ + "/r_max", r_max_) == false)
  {
    ROS_FATAL("No parameter 'r_max' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/r_min",r_min_)==false)
  {
    ROS_FATAL("No parameter 'r_min' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/r_n",r_n_)==false)
  {
    ROS_FATAL("No parameter 'r_n' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/theta_max",theta_max_)==false)
  {
    ROS_FATAL("No parameter 'theta_max' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/theta_min",theta_min_)==false)
  {
    ROS_FATAL("No parameter 'theta_min' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/theta_n",theta_n_)==false)
  {
    ROS_FATAL("No parameter 'theta_n' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/phi_max",phi_max_)==false)
  {
    ROS_FATAL("No parameter 'phi_max' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/phi_min",phi_min_)==false)
  {
    ROS_FATAL("No parameter 'phi_min' specified");
    ros::shutdown();
    exit(1);
  }
  if(ros::param::get(node_name_+"/phi_n",phi_n_)==false)
  {
    ROS_FATAL("No parameter 'phi_n' specified");
    ros::shutdown();
    exit(1);
  }

  discretizeSpace();
  ROS_INFO("Space was discretized.");

  counter_ = 0;
} // end FlowerObserver()

/*--------------------------------------------------------------------
 * setModelState()
 * Call the Gazebo's setModelState service.
 *------------------------------------------------------------------*/

void FlowerObserver::discretizeSpace()
{
  double dr = (r_max_ - r_min_) / (r_n_-1);  
  double dtheta = (theta_max_ - theta_min_) / (theta_n_-1);  
  double dphi = (phi_max_ - phi_min_) / (phi_n_-1);  

  for (int i=0; i<r_n_; i++)
  {
    for (int j=0; j<theta_n_; j++)
    {
      for (int k=0; k<phi_n_; k++)
      {
        r_.push_back(r_min_+i*dr);
        theta_.push_back(theta_min_+j*dtheta);
        phi_.push_back(phi_min_+k*dphi);

        ROS_INFO_STREAM("r:"<<r_min_+i*dr<<", theta:"<<theta_min_+j*dtheta<<", phi:"<<phi_min_+k*dphi);
      }
    }
  }
  max_counter_ = r_n_ * theta_n_ * phi_n_;
  // ROS_INFO("Space was discretized.");
} // end setModelState()


/*--------------------------------------------------------------------
 * setModelState()
 * Call the Gazebo's setModelState service.
 *------------------------------------------------------------------*/

void FlowerObserver::setModelState()
{
  x_ = r_[counter_] * sin(phi_[counter_]) * cos(theta_[counter_]);
  y_ = r_[counter_] * sin(phi_[counter_]) * sin(theta_[counter_]);
  z_ = r_[counter_] * cos(phi_[counter_]);

  // x_ = 0.25;
  // y_ = 0.25;
  // z_ = 0.3536;

  geometry_msgs::Point camera_position;
  camera_position.x = x_;
  camera_position.y = y_;
  camera_position.z = z_;

  pubPosition.publish(camera_position);

  tf2::Quaternion quat1;
  quat1.setEuler(0, 0, theta_[counter_]-M_PI);
  // quat1.setEuler(0, 0, M_PI_4-M_PI);
  quat1.normalize();
  tf2::Quaternion quat2;
  quat2.setEuler(M_PI_2-phi_[counter_], 0, 0);
  // quat2.setEuler(-(M_PI_4+M_PI_2), 0, 0);
  quat2.normalize();

  tf2::Quaternion quat = quat1 * quat2;

  q1_ = quat.x();
  q2_ = quat.y();
  q3_ = quat.z();
  q4_ = quat.w();

  ROS_INFO_STREAM("Quaternion: " << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w());

  geometry_msgs::Pose camera_pose;
  camera_pose.position.x = x_;
  camera_pose.position.y = y_;
  camera_pose.position.z = z_ + 0.1;
  camera_pose.orientation.x = q1_;
  camera_pose.orientation.y = q2_;
  camera_pose.orientation.z = q3_;
  camera_pose.orientation.w = q4_;

  gazebo_msgs::ModelState camera_model_state;
  camera_model_state.model_name = (std::string) "realsense2_camera";
  camera_model_state.pose = camera_pose;

  gazebo_msgs::SetModelState srv;
  srv.request.model_state = camera_model_state;
  ROS_INFO_STREAM("Camera model state: "<<camera_model_state);

  if(clientSetModelState.call(srv))
  {
      ROS_INFO("Realsense D425 was magically moved with success!");
  }
  else
  {
      ROS_ERROR("Failed to magically move Realsense D425! Error msg:%s",srv.response.status_message.c_str());
  }
} // end setModelState()

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "flower_observer");
  ros::NodeHandle nh("");

  // Tell ROS how fast to run this node.
  ros::Rate r(1);

  // Create object FlowerObserver.
  FlowerObserver flower_observer(nh);

  ros::Duration(5).sleep();

  // Main loop.
  while (ros::ok())
  {
    if (flower_observer.counter_ < flower_observer.max_counter_)
    {
      flower_observer.setModelState();
      flower_observer.counter_++;
      ROS_INFO_STREAM("Progress: " << flower_observer.counter_ << "/" << flower_observer.max_counter_);
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()