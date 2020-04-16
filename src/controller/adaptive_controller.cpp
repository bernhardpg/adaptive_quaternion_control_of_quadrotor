#include "controller/adaptive_controller.h"

namespace controller {
  AdaptiveController::AdaptiveController(ros::NodeHandle *nh)
    : nh_(*nh)
  {
    AdaptiveController::init();

    // TODO change to subscribe to /attitude published by rosflight
    odom_subscriber_ = nh_.subscribe("/multirotor/truth/NED", 1000,
        &AdaptiveController::odomCallback, this);
    command_publisher_ = nh_.advertise<rosflight_msgs::Command>(
        "/command", 1000);
  }

  void AdaptiveController::init()
  {
    // Initialize variables
    q_ = Eigen::Quaterniond::Identity();
    w_ << 0.0, 0.0, 0.0;
    q_e_ = Eigen::Quaterniond::Identity();
    w_bc_ << 0.0, 0.0, 0.0;

    // Set command signal
    q_c_ = Eigen::Quaterniond::Identity();
    w_bc_ = Eigen::Vector3d::Zero();
  }

  void AdaptiveController::odomCallback(
      const nav_msgs::Odometry::ConstPtr& msg
      )
  {
    // Attitude
    q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);

    // Angular velocity
    w_ << msg->twist.twist.angular.x,
          msg->twist.twist.angular.y,
          msg->twist.twist.angular.z; // TODO should this be negative?

    // Controll loop
    calculateErrors();
    computeInput();

  }

  void AdaptiveController::calculateErrors()
  {
    q_e_ = q_c_ * q_;
    w_bc_ = w_ - q_e_.conjugate()._transformVector(w_c_);

    //std::cout << q_e_.w() << std::endl << q_e_.vec() << std::endl << std::endl;
    //std::cout << w_bc_ << std::endl << std::endl;
  }

  void AdaptiveController::computeInput()
  {
      
  
  
  }

  void AdaptiveController::publishCommand()
  {
    rosflight_msgs::Command command;
    command.header.stamp = ros::Time::now();
    command.ignore = rosflight_msgs::Command::IGNORE_NONE;
    command.mode = rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    
    // F from 0 to 1, scale by max thrust
    command.F = 0;
    command.x = 0;
    command.y = 0;
    command.z = 0;

    command_publisher_.publish(command);
  }

  double AdaptiveController::saturate(double v, double min, double max)
  {
    v = v > max ? max : (
        v < min ? min : v
        );

    return v;
  }

} // namespace controller


// TODO move out of ROS node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "adaptive_controller");
  ros::NodeHandle nh;
  controller::AdaptiveController c = controller::AdaptiveController(&nh);
  ros::spin();

  return 0;
}
