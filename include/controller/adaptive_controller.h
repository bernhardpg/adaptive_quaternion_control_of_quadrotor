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
      // ROS
      ros::NodeHandle nh_;
      ros::Subscriber odom_subscriber_;
      ros::Publisher command_publisher_;

      // Signals
      // State
      Eigen::Quaterniond q_; // Rotation from body to inertial frame
      Eigen::Vector3d w_; // Angular velocity between body and inertial frame
      double height_;
      double height_dot_;
      // Errors
      Eigen::Quaterniond q_e_; // From body to command frame
      Eigen::Vector3d w_bc_; // Between body and command frame, given in body frame
      double e_height_;
      double e_height_dot_;

      // Command
      Eigen::Quaterniond q_c_;
      Eigen::Vector3d w_c_; // Given in command frame
      Eigen::Vector3d w_c_dot_;
      double height_c_;

      // Model parameters
      double m_;
      double g_;
      double max_thrust_;
      Eigen::Matrix3d J_; // Inertia matrix

      // Controller
      input_t input_;
      double k_q_;
      double k_w_;

      // Functions
      void init();
      void calculateErrors();
      void computeInput();
      void publishCommand();
      void odomCallback(const nav_msgs::OdometryConstPtr &msg);

      void heightController();

      double saturate(double v, double min, double max); // TODO move somewhere else
      // TODO move somewhere else
      Eigen::Matrix3d cross_map(Eigen::Vector3d v);
      Eigen::Vector3d vee_map(Eigen::Matrix3d v_hat); // Inverse of cross_map
      Eigen::Vector3d quat_log_v(Eigen::Quaterniond q);
      Eigen::Quaterniond quat_plus_map(Eigen::Quaterniond q);

  };
}
