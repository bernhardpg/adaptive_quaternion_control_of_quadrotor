#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "rosflight_msgs/Command.h"

namespace controller {
  typedef struct
  {
    Eigen::Vector3d tau;
    double F;
  } input_t;

  class AdaptiveController
  {
    public:
      AdaptiveController(ros::NodeHandle *nh);
    private:
      ros::NodeHandle nh_;

      ros::Subscriber odom_subscriber_;
      ros::Publisher command_publisher_;

      // State
      Eigen::Quaterniond q_;
      Eigen::Vector3d w_;
      // Errors
      Eigen::Quaterniond q_e_; // From body to command frame
      Eigen::Vector3d w_bc_; // Between body and command frame, given in body frame
      // Command
      Eigen::Quaterniond q_c_;
      Eigen::Vector3d w_c_; // Given in command frame

      void init();
      void calculateErrors();
      void computeInput();
      void publishCommand();
      void odomCallback(const nav_msgs::OdometryConstPtr &msg);

      double saturate(double v, double min, double max); // TODO move somewhere else

  };
}
