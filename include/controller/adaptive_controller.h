#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "rosflight_msgs/Command.h"

using namespace Eigen;

namespace controller {
  typedef struct
  {
    Vector3d tau;
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

      void odomCallback(const nav_msgs::OdometryConstPtr &msg);
      void publishCommand();

      double saturate(double v, double min, double max); // TODO move somewhere else

  };
}
