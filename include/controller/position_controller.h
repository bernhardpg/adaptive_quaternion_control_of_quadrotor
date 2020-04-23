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
      ros::Publisher error_publisher_;

      // Signals
      // State
      double height_;
      double height_dot_;
      Eigen::Vector2d pos_;
      Eigen::Vector2d vel_;
      // Errors
      double e_height_;
      double e_height_dot_;
      Eigen::Vector2d e_pos_;
      Eigen::Vector2d e_vel_;

      // Command
      double height_c_;
      Eigen::Vector2d pos_c_;

      // Model parameters
      double m_;
      double g_;
      double max_thrust_;

      // Controller
      double k_e_;
      double k_e_dot_;
      attitude_input_t input_;

      // Functions
      void init();
      void calculateErrors();
      void computeInput();
      void publishCommand();
      void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

      double saturate(double v, double min, double max);
  };
}
