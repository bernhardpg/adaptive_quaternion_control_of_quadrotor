#include "controller/attitude_controller.h"

namespace controller {
  AdaptiveController::AdaptiveController(ros::NodeHandle *nh)
    : nh_(*nh)
  {
    AdaptiveController::init();

    // TODO change to subscribe to /attitude published by rosflight
    odom_subscriber_ = nh_.subscribe("/attitude", 1000,
        &AdaptiveController::odomCallback, this);
    attitude_command_subscriber_ = nh_.subscribe("/attitude_command", 1000,
        &AdaptiveController::commandCallback, this);
    command_publisher_ = nh_.advertise<rosflight_msgs::Command>(
        "/command", 1000);

    attitude_publisher_ = nh_.advertise<rosflight_msgs::Command>(
        "/attitude_euler", 1000);
    attitude_ref_publisher = nh_.advertise<rosflight_msgs::Command>(
        "/attitude_ref_euler", 1000);

    std::cout << "Attitude controller initialized" << std::endl;
  }

  void AdaptiveController::init()
  {
    // Controller params
    k_q_ = 30;
    k_w_ = 7;

    // Initialize model parameters
    double max_rotor_thrust = 14.961;
    double arm_length = 0.2;
    max_thrust_ = max_rotor_thrust * 4; // From orgazebo sim, 4 rotor
    max_torque_ = arm_length * max_rotor_thrust * 2;
    J_ << 0.07, 0, 0,
          0, 0.08, 0,
          0, 0, 0.12; // From .urdf file

    // Initialize variables
    q_ = Eigen::Quaterniond::Identity();
    w_ = Eigen::Vector3d::Zero();
    q_e_ = Eigen::Quaterniond::Identity();
    w_bc_ = Eigen::Vector3d::Zero();
    time_step_ = 0.0;

    // Set initial values for command trajectory
    q_c_ = Eigen::Quaterniond::Identity();
    w_c_ = Eigen::Vector3d::Zero();
    w_c_dot_ = Eigen::Vector3d::Zero();
    w_c_body_frame = Eigen::Vector3d::Zero();
    w_c_dot_body_frame = Eigen::Vector3d::Zero();
  }

  void AdaptiveController::odomCallback(
      const rosflight_msgs::Attitude::ConstPtr &msg
      )
  {
    q_ = Eigen::Quaterniond(msg->attitude.w,
                            msg->attitude.x,
                            msg->attitude.y,
                            msg->attitude.z);

    w_ << msg->angular_velocity.x,
          msg->angular_velocity.y,
          msg->angular_velocity.z;

    // Controll loop
    calculateErrors();
    computeInput();
    publishCommand();
    publish_attitude_tracking();
  }

  void AdaptiveController::commandCallback(
      const rosflight_msgs::Command::ConstPtr& msg
      )
  {
    // Feedforward input thrust
    input_.F = msg->F;

    //double x = 0.15 * sin(ros::Time::now().toSec() * 2 * M_PI * 0.4);
    //double y = 0.3 * sin(ros::Time::now().toSec() * 2 * M_PI * 0.5);
    //q_c_ = EulerToQuat(z, y, x);

    // Save desired angles to reference signal
    q_r_ = controller::EulerToQuat(0, 0, 0);
  }

  void AdaptiveController::generateReferenceSignal()
  {
    double w_0 = 1.0; // Bandwidth
    double D = 2.0; // Damping

    // Difference between reference and command frame
    Eigen::Quaterniond q_rc = q_r_.conjugate() * q_c_;

    // Create quaternion from vector to make math work
    Eigen::Quaterniond w_c_quat;
    w_c_quat.w() = 0;
    w_c_quat.vec() = 0.5 * w_c_;

    // Define derivatives
    Eigen::Quaterniond q_c_dot = q_c_ * w_c_quat;
    w_c_dot_ = - 2 * pow(w_0, 2) * quat_log_v(q_rc)
      - 2 * D * w_0 * w_c_;
    // Note: w_c_dot_ = u_c in the paper

    // Integrate using forward Euler:
    q_c_.w() = q_c_.w() + time_step_ * q_c_dot.w();
    q_c_.vec() = q_c_.vec() + time_step_ * q_c_dot.vec();
    w_c_ = w_c_ + time_step_ * w_c_dot_;

    // Calculate body frame values
    w_c_body_frame = q_e_.conjugate()._transformVector(w_c_);
    w_c_dot_body_frame = q_e_.conjugate()._transformVector(w_c_dot_);
  }

  void AdaptiveController::calculateErrors()
  {
    // Calulate attitude error
    q_e_ = q_c_.conjugate() * q_;
    // Calculate angular velocity error
    w_bc_ = w_ - q_e_.conjugate()._transformVector(w_c_);
  }

  void AdaptiveController::computeInput()
  {
    // Calculates inputs for the NED frame!
    // Baseline controller
    Eigen::Vector3d cancellation_terms = cross_map(w_) * J_ * w_
      + J_ * (w_c_dot_body_frame
          + cross_map(w_) * w_c_body_frame);

    Eigen::Vector3d feedforward_terms = J_ * (- k_q_ * quat_log_v(quat_plus_map(q_e_)) - k_w_ * w_bc_);

    input_.tau = cancellation_terms + feedforward_terms;
  }

  void AdaptiveController::publishCommand()
  {
    rosflight_msgs::Command command;
    command.header.stamp = ros::Time::now();
    //command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command.mode = rosflight_msgs::Command::MODE_PASS_THROUGH;

    command.F = saturate(input_.F / max_thrust_, 0, 1);
    // Defined in NED frame by ROSflight
    command.x = saturate(input_.tau(0) / max_torque_, -1, 1);
    command.y = saturate(input_.tau(1) / max_torque_, -1, 1);
    command.z = saturate(input_.tau(2) / 3, -1, 1); // TODO change with real z max torque

    command_publisher_.publish(command);
  }


  // ***********************
  // Publish for plotting
  // ***********************
  void AdaptiveController::publish_attitude_tracking()
  {
    rosflight_msgs::Command attitude;
    attitude.header.stamp = ros::Time::now();
    Eigen::Vector3d att_vec = controller::QuatToEuler(q_);
    attitude.x = att_vec(0);
    attitude.y = att_vec(1);
    attitude.z = att_vec(2);
    attitude_publisher_.publish(attitude);

    rosflight_msgs::Command attitude_c;
    attitude_c.header.stamp = ros::Time::now();
    Eigen::Vector3d att_vec_c = controller::QuatToEuler(q_c_);
    attitude_c.x = att_vec_c(0);
    attitude_c.y = att_vec_c(1);
    attitude_c.z = att_vec_c(2);
    attitude_ref_publisher.publish(attitude_c);
  }


  // *********
  // Utilities
  // *********
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

  Eigen::Quaterniond EulerToQuat(double yaw, double pitch, double roll)
    // yaw (Z), pitch (Y), roll (X)
  {
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
  }

  Eigen::Vector3d QuatToEuler(Eigen::Quaterniond q) {
    Eigen::Vector3d euler;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        euler(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return euler;
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
