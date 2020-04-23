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
    // Controller params
    k_q_ = 0.5;
    k_w_ = 0.1;

    // Initialize model parameters
    m_ = 2.0; // kg
    g_ = 9.8; // m/s**2
    max_thrust_ = 14.961 * 4; // From gazebo sim, 4 rotor
    J_ = Eigen::Matrix3d::Identity();

    // Initialize variables
    q_ = Eigen::Quaterniond::Identity();
    w_ = Eigen::Vector3d::Zero();
    q_e_ = Eigen::Quaterniond::Identity();

    w_bc_ = Eigen::Vector3d::Zero();

    // Set command signal
    q_c_ = Eigen::Quaterniond::Identity();
    w_c_ = Eigen::Vector3d::Zero();
    w_c_dot_ = Eigen::Vector3d::Zero();

    height_c_ = 1.5;

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

    height_ = -msg->pose.pose.position.z;
    height_dot_ = -msg->twist.twist.linear.z;

    //
    // Controll loop
    calculateErrors();
    computeInput();
    heightController();
    publishCommand();
  }

  void AdaptiveController::heightController()
  {
    e_height_  = height_ - height_c_;
    e_height_dot_ = height_dot_;
    std::cout << "e_height_: " << e_height_ << std::endl;

    input_.F = - 40.0 * e_height_ - 10 * e_height_dot_ + m_ * g_;
    //std::cout << "F: "<< input_.F << std::endl;
    //std::cout << saturate(input_.F / max_thrust_, 0, 1.0) << std::endl;
  }

  void AdaptiveController::calculateErrors()
  {
    // Calulate attitude error
  
    q_e_ = q_c_ * q_;
    // Calculate angular velocity error
    w_bc_ = w_ - q_e_.conjugate()._transformVector(w_c_);

    //std::cout << q_e_.w() << std::endl << q_e_.vec() << std::endl << std::endl;
    //std::cout << w_bc_ << std::endl << std::endl;
  }


  void AdaptiveController::computeInput()
  {

    // Baseline controller
    Eigen::Vector3d cancellation_terms = cross_map(w_) * J_ * w_
      + J_ * (w_c_dot_
          + cross_map(w_c_) * w_c_);

    // TODO change q_e_ to plus map
    Eigen::Vector3d feedforward_terms = - k_q_ * quat_log_v(q_e_) - k_w_ * w_bc_;

    input_.tau = cancellation_terms + feedforward_terms;
    std::cout << "input_tau:\n" << input_.tau << std::endl << std::endl;
  }

  
  void AdaptiveController::publishCommand()
  {
    rosflight_msgs::Command command;
    command.header.stamp = ros::Time::now();
    command.ignore = rosflight_msgs::Command::IGNORE_NONE;
    //command.mode = rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    command.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;
    //

    // F from 0 to 1, scale by max thrust
    command.F = saturate(input_.F / max_thrust_, 0, 1.0);
    // TODO input here, but it must be scaled
    command.x = input_.tau(0);
    command.y = input_.tau(1);
    command.z = input_.tau(2);

    command_publisher_.publish(command);
  }

  double AdaptiveController::saturate(double v, double min, double max)
  {
    v = v > max ? max : (
        v < min ? min : v
        );

    return v;
  }

  // Note: also called hat map in litterature
  Eigen::Matrix3d AdaptiveController::cross_map(Eigen::Vector3d v)
  {
    Eigen::Matrix3d v_hat;
    v_hat << 0, -v(2), v(1),
            v(2), 0, -v(0),
          -v(1), v(0), 0;
    return v_hat;
  }

  // Opposite of hat map
  Eigen::Vector3d AdaptiveController::vee_map(Eigen::Matrix3d v_hat)
  {
    Eigen::Vector3d v;
    v << v_hat(2,1), v_hat(0,2), v_hat(0,1);
    k_q_ = 1.0;
    return v;
  }

  // Returns the direct map of the quaternion logarithm to R^3 (instead of R^4)
  Eigen::Vector3d AdaptiveController::quat_log_v(Eigen::Quaterniond q)
  {
    Eigen::AngleAxis<double> aa(q);
    return (aa.angle() / 2) * aa.axis();
  }

  // Returns the short rotation quaternion with angle <= pi
  Eigen::Quaterniond AdaptiveController::quat_plus_map(Eigen::Quaterniond q)
  {
    return q.w() >= 0 ? q : Eigen::Quaterniond(-q.w(), q.x(), q.y(), q.z());
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
