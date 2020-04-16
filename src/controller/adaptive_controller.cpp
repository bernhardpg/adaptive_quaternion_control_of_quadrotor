#include "controller/adaptive_controller.h"

namespace controller {
  AdaptiveController::AdaptiveController(ros::NodeHandle *nh) : nh_(*nh)
  {
    odom_subscriber_ = nh_.subscribe("/multirotor/truth/NED", 1000,
        &AdaptiveController::odomCallback, this);
    command_publisher_ = nh_.advertise<rosflight_msgs::Command>(
        "/command", 1000);

    ROS_INFO("Controller node init complete");
  }

  void AdaptiveController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
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
