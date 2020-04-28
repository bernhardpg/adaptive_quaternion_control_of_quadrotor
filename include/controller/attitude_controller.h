#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "rosflight_msgs/Command.h"
#include "rosflight_msgs/Attitude.h"

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
      ros::Publisher command_publisher_;
      ros::Subscriber odom_subscriber_;
      ros::Subscriber attitude_command_subscriber_;

      ros::Publisher attitude_publisher_;
      ros::Publisher attitude_ref_publisher;

      // Signals
      // State
      Eigen::Quaterniond q_; // Rotation from body to inertial frame
      Eigen::Vector3d w_; // Angular velocity between body and inertial frame
      // Errors
      Eigen::Quaterniond q_e_; // From body to command frame
      Eigen::Vector3d w_bc_; // Between body and command frame, given in body frame
      // Command
      Eigen::Quaterniond q_c_;
      Eigen::Vector3d w_c_; // Given in command frame
      Eigen::Vector3d w_c_body_frame;
      Eigen::Vector3d w_c_dot_;
      Eigen::Vector3d w_c_dot_body_frame;

      // Model parameters
      double max_thrust_;
      double max_torque_;
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
      void odomCallback(const rosflight_msgs::Attitude::ConstPtr &msg);
      void commandCallback(const rosflight_msgs::Command::ConstPtr& msg);

      double saturate(double v, double min, double max); // TODO move somewhere else

      // TODO move somewhere else
      Eigen::Matrix3d cross_map(Eigen::Vector3d v);
      Eigen::Vector3d vee_map(Eigen::Matrix3d v_hat); // Inverse of cross_map
      Eigen::Vector3d quat_log_v(Eigen::Quaterniond q);
      Eigen::Quaterniond quat_plus_map(Eigen::Quaterniond q);

      //Eigen::Vector3d QuatToEuler(Eigen::Quaterniond q);
      //Eigen::Quaterniond EulerToQuat(double yaw, double pitch, double roll);

      void publish_attitude_tracking();
  };

  Eigen::Vector3d QuatToEuler(Eigen::Quaterniond q);
  Eigen::Quaterniond EulerToQuat(double yaw, double pitch, double roll);

}
