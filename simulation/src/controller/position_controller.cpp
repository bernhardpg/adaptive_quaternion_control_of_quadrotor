#include "controller/position_controller.h"

namespace controller {
  PositionController::PositionController()
  {
    PositionController::init();
  }


  void PositionController::init()
  {
    // Initialize model parameters
    m_ = 2.856; // kg
    g_ = 9.8; // m/s**2

		// Initialize variables
		pos_ << 0, 0, 0;
		vel_ << 0, 0, 0;
		e_pos_ << 0, 0, 0;
		e_vel_ << 0, 0, 0;

		// Setpoint
		pos_d_ << 0, 0, -3.0;
		vel_d_ << 0, 0, 0;

		// Controller
		K_.resize(3,6);
    K_ << 3.1622, 0, 0, 2.7063, 0, 0,
					0, 3.1622, 0, 0, 2.7063, 0,
					0, 0, 3.1622, 0, 0, 2.7063;

  }

  void PositionController::controllerCallback(
			Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Quaterniond q
			)
  {
    // NED frame
		pos_ = pos;
		vel_ = vel;
		q_ = q;
    thrust_direction_ = -q_.toRotationMatrix()(Eigen::all, Eigen::last);

    // Controll loop
    calculateErrors();
    computeInput();
  }

  void PositionController::calculateErrors()
  {
    e_pos_ = pos_ - pos_d_;
    e_vel_ = vel_ - vel_d_;
  }

  void PositionController::computeInput()
  {
		Eigen::VectorXd tot_e(6);
		tot_e << e_pos_, e_vel_;

    Eigen::Vector3d u_pos = - K_ * (tot_e);
    Eigen::Vector3d u_pd = u_pos - Eigen::Vector3d(0,0,g_);

    Eigen::Quaterniond q_d;
    q_d.w() = (thrust_direction_.dot(u_pd) + u_pd.norm());
    q_d.vec() = thrust_direction_.cross(u_pd);
    q_d.normalize();

    double F_th = u_pd.norm() * m_; // Note: F always applied in body z-axis
    Eigen::Vector3d attitude_d = QuatToEuler(q_d);
  }
} // namespace controller

