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
    position_error_publisher = nh_.advertise<rosflight_msgs::Command>(
        "/position_error", 1000);
    position_publisher = nh_.advertise<rosflight_msgs::Command>(
        "/position", 1000);
    position_ref_publisher = nh_.advertise<rosflight_msgs::Command>(
        "/position_ref", 1000);

    std::cout << "Position controller initialized" << std::endl;

  }


  void PositionController::init()
  {
    // Initialize model parameters
    m_ = 2.0; // kg
    g_ = 9.8; // m/s**2
    max_thrust_ = 14.961 * 4; // From gazebo sim, 4 rotor
    //std::cout << "F: " << input_.F << std::endl;

    // Initialize variables
    input_.F = 0;

    pos_ = Eigen::VectorXd::Zero(6);
    e_pos_ = Eigen::VectorXd::Zero(6);
    pos_d_ = Eigen::VectorXd::Zero(6);

    pos_d_(0) = 0.0;
    pos_d_(1) = 0.0;
    pos_d_(2) = -3.0; // set height
  }

  void PositionController::publish_position_tracking()
  {
    rosflight_msgs::Command pos_error;
    pos_error.header.stamp = ros::Time::now();
    pos_error.x = e_pos_(0);
    pos_error.y = e_pos_(1);
    pos_error.z = e_pos_(2);
    position_error_publisher.publish(pos_error);

    rosflight_msgs::Command pos;
    pos.header.stamp = ros::Time::now();
    pos.x = pos_(0);
    pos.y = pos_(1);
    pos.z = pos_(2);
    position_publisher.publish(pos);

    rosflight_msgs::Command pos_d;
    pos_d.header.stamp = ros::Time::now();
    pos_d.x = pos_d_(0);
    pos_d.y = pos_d_(1);
    pos_d.z = pos_d_(2);
    position_ref_publisher.publish(pos_d);
  }

  void PositionController::odomCallback(
      const nav_msgs::Odometry::ConstPtr& msg
      )
  {
    // NOTE NED frame!
    pos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;

    // TODO this should come from estimator (works now because zb is equal in NED frame and body frame)
    // Attitude
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                  msg->pose.pose.orientation.x,
                  msg->pose.pose.orientation.y,
                  msg->pose.pose.orientation.z);
    z_b_ = q.toRotationMatrix()(Eigen::all, Eigen::last);
    //std::cout << "z vector:\n "position_error_publisher << z_b_ << std::endl;

    // Controll loop
    calculateErrors();
    computeInput();
    publishCommand();
    publish_position_tracking();
  }

  void PositionController::calculateErrors()
  {
    e_pos_ = pos_ - pos_d_;
    //std::cout << "e_pos: \n" << e_pos_ << std::endl << std::endl;
  }

  void PositionController::computeInput()
  {
    // TODO what about mixer??
    // TODO clean up
    Eigen::MatrixXd K_(3,6);
    K_ <<


3.162277660168362481e-02, 0.000000000000000000e+00, 0.000000000000000000e+00, 2.534670653228285331e-01, 0.000000000000000000e+00, 0.000000000000000000e+00,
0.000000000000000000e+00, 3.162277660168379134e-02, 3.453002916881814603e-14, 0.000000000000000000e+00, 2.516854250912579638e-01, 4.192748472657690160e-15,
0.000000000000000000e+00, -1.942469094671972213e-17, 3.162277660168368421e+00, 0.000000000000000000e+00, 4.192748472657690160e-15, 1.031137989409451450e+01;


//3.162277660168367532e+00, 0.000000000000000000e+00, 0.000000000000000000e+00, 5.596834401725383046e+00, 0.000000000000000000e+00, 0.000000000000000000e+00,
//0.000000000000000000e+00, 3.162277660168351545e+00, -1.140793766416117155e-13, 0.000000000000000000e+00, 5.596834401725378605e+00, -1.202692469726148055e-14,
//0.000000000000000000e+00, -6.468126082651022810e-14, 9.999999999999516831e+00, 0.000000000000000000e+00, -1.202692469726148055e-14, 1.095445115010327619e+01;

    Eigen::Vector3d u_pos = - K_ * (e_pos_);
    Eigen::Vector3d u_pd = u_pos - Eigen::Vector3d(0,0, 1.7 * g_); // TODO factor two here works very well, but why?

    Eigen::Quaterniond q_d;
    q_d.w() = (-z_b_.dot(u_pd) + u_pd.norm());
    q_d.vec() = -z_b_.cross(u_pd);
    q_d.normalize();

    double F_th = u_pd.norm() * m_; // Note: F always applied in body z-axis
    Eigen::Vector3d attitude_d = QuatToEuler(q_d);
    input_.F = F_th;
    input_.roll = attitude_d(0);
    input_.pitch= attitude_d(1);
    input_.yaw = attitude_d(2);
  }

  void PositionController::publishCommand()
  {
    rosflight_msgs::Command command;
    command.header.stamp = ros::Time::now();
    command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

    // Scale inputs in attitude controller
    command.F = input_.F;
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


  Eigen::Quaterniond PositionController::EulerToQuat(double yaw, double pitch, double roll)
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

  Eigen::Vector3d PositionController::QuatToEuler(Eigen::Quaterniond q) {
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
  ros::init(argc, argv, "position_controller");
  ros::NodeHandle nh;
  controller::PositionController c = controller::PositionController(&nh);
  ros::spin();

  return 0;
}
