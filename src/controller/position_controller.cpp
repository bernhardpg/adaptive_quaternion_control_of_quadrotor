#include "controller/position_controller.h"

namespace controller {
  PositionController::PositionController(ros::NodeHandle *nh)
    : nh_(*nh)
  {
    PositionController::init();

    // TODO change to subscribe to /attitude published by rosflight
    odom_subscriber_ = nh_.subscribe("/multirotor/truth/NED", 1000,
        &PositionController::odomCallback, this);
    command_publisher_ = nh_.advertise<rosflight_msgs::Command>(
        "/attitude_command", 1000);
  }

  void PositionController::init()
  {
    // Controller params
    k_e_ = 0.5;
    k_e_dot_ = 0.1;

    // Initialize model parameters
    m_ = 2.0; // kg
    g_ = 9.8; // m/s**2
    max_thrust_ = 14.961 * 4; // From gazebo sim, 4 rotor

    // Initialize variables
    // TODO
    
    // Set command signal
    height_c_ = 1.5;

  }

  void PositionController::odomCallback(
      const nav_msgs::Odometry::ConstPtr& msg
      )
  {
    height_ = -msg->pose.pose.position.z;
    height_dot_ = -msg->twist.twist.linear.z;

    // Controll loop
    calculateErrors();
    computeInput();
    publishCommand();
  }

  void PositionController::calculateErrors()
  {
    e_height_  = height_ - height_c_;
    e_height_dot_ = height_dot_;
    std::cout << "e_height_: " << e_height_ << std::endl;
  }

  void PositionController::computeInput()

  {
    input_.F = - 40.0 * e_height_ - 10 * e_height_dot_ + m_ * g_;
    //std::cout << "F: "<< input_.F << std::endl;
    //std::cout << saturate(input_.F / max_thrust_, 0, 1.0) << std::endl;
  }

  void PositionController::publishCommand()
  {
    rosflight_msgs::Command command;
    command.header.stamp = ros::Time::now();
    command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

    // F from 0 to 1, scale by max thrust
    command.F = saturate(input_.F / max_thrust_, 0, 1.0);
    command.x = input_.roll;
    command.y = input_.pitch;
    command.z = input_.yaw;

    command_publisher_.publish(command);
  }

  double PositionController::saturate(double v, double min, double max)
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
  ros::init(argc, argv, "position_controller");
  ros::NodeHandle nh;
  controller::PositionController c = controller::PositionController(&nh);
  ros::spin();

  return 0;
}
