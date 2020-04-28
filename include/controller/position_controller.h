#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "rosflight_msgs/Command.h"

namespace controller{
  typedef struct
  {
    double roll;
    double pitch;
    double yaw;
    double F;
  } attitude_input_t;

  class PositionController
  {
    public:
      PositionController(ros::NodeHandle *nh);
    private:
      // ROS
      ros::NodeHandle nh_;
      ros::Subscriber odom_subscriber_;
      ros::Publisher command_publisher_;
      ros::Publisher debug_position_error;

      // Signals
      // State
      Eigen::VectorXd pos_;
      Eigen::Vector3d z_b_; // z-axis in body frame
      // Errors
      Eigen::VectorXd e_pos_;

      // Command
      Eigen::VectorXd pos_d_;

      // Model parameters
      double m_;
      double g_;
      double max_thrust_;

      // Controller
      attitude_input_t input_;

      // Functions
      void init();
      void calculateErrors();
      void computeInput();
      void publishCommand();
      void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

      double saturate(double v, double min, double max);
      void publish_position_error();

      Eigen::Vector3d QuatToEuler(Eigen::Quaterniond q);
      Eigen::Quaterniond EulerToQuat(double yaw, double pitch, double roll);
  };
}
