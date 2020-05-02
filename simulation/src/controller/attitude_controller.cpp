#include "controller/attitude_controller.h"

namespace controller {
  AdaptiveController::AdaptiveController()
  {
    init();
  }

  void AdaptiveController::init()
  {
		// *******
    // Model parameters
		// *******
		Eigen::Matrix3d est_errors;
		est_errors << 0.008, 0,  0,
									0, 0.005, 0,
									0, 0,  0.01;

    J_nominal_ << 0.07, 0, 0,
									0, 0.08, 0,
									0, 0, 0.12; // From .urdf file
		// Augment inertia matrix to not be perfectly identified
		J_nominal_ += est_errors;

		// *******
		// Trajectory generator params
		// *******
    step_size_ = pow(10, -3); // From simulation

    cmd_w_0_ = 30.0; // Bandwidth
    cmd_damping_ = 1.0; // Damping

		// *******
    // Controller params
		// *******
    k_q_ = 1.0;
    k_w_ = 1.0;

		// *******
		// Adaptive controller
		// *******
		enable_adaptive_controller_ = false;
		k_e_ = 25;

		// Adaptive gains
		adaptive_gain_Theta_ = 1 * Eigen::Vector3d(1,1,1).asDiagonal();
		adaptive_gain_Lambda_ = 1 * Eigen::Vector3d(1,1,1).asDiagonal();
		adaptive_gain_tau_ = 600;

		// Nominal values
		Theta_nominal_ = Eigen::Matrix3d::Zero();
		Theta_nominal_(0,0) = (J_nominal_(2,2) - J_nominal_(1,1)) / J_nominal_(0,0);
		Theta_nominal_(1,1) = (J_nominal_(0,0) - J_nominal_(2,2)) / J_nominal_(1,1);
		Theta_nominal_(2,2) = (J_nominal_(1,1) - J_nominal_(0,0)) / J_nominal_(2,2);

		// Initialize adaptive parameters
		Lambda_hat_ = Eigen::Matrix3d(J_nominal_.inverse());
		Theta_hat_ = Eigen::Matrix3d(Theta_nominal_);
		tau_dist_hat_ << 0, 0, 0;

		// Initialize signals
		P_ = Eigen::Matrix3d::Identity();
		Phi_ << 0, 0, 0;

		w_adaptive_model_ << 0, 0, 0;
		e_adaptive_model_ << 0, 0, 0;

		// *******
    // Initialize variables
		// *******
    q_ = Eigen::Quaterniond::Identity();
    w_ = Eigen::Vector3d::Zero();
    q_e_ = Eigen::Quaterniond::Identity();
    w_bc_ = Eigen::Vector3d::Zero();

    q_c_ = Eigen::Quaterniond::Identity();
    w_c_ = Eigen::Vector3d::Zero();
    w_c_dot_ = Eigen::Vector3d::Zero();
    w_c_body_frame_ = Eigen::Vector3d::Zero();
    w_c_dot_body_frame_ = Eigen::Vector3d::Zero();

    q_r_ = EulerToQuat(0,0,0);
  }

  void AdaptiveController::controllerCallback(Eigen::Quaterniond q, Eigen::Vector3d w, double t)
  {
		t_ = t;
    q_ = Eigen::Quaterniond(q);
    w_ = Eigen::Vector3d(w);

    // Controll loop
    generateCommandSignal();
    calculateErrors();
    calculateBaselineInput();
		calculateAdaptiveReferenceErrors();
		calculateAdaptiveParameters();
		calculateAdaptiveInput();
		calculateTotalInputTorques();
  }

  void AdaptiveController::generateCommandSignal()
  {

    // Difference between reference and command frame
    Eigen::Quaterniond q_rc = q_r_.conjugate() * q_c_;

    // Create quaternion from vector to make math work
    Eigen::Quaterniond w_c_quat;
    w_c_quat.w() = 0;
    w_c_quat.vec() = 0.5 * w_c_;

    // Define derivatives
    Eigen::Quaterniond q_c_dot = q_c_ * w_c_quat;
    w_c_dot_ = - 2 * pow(cmd_w_0_, 2) * quat_log_v(quat_plus_map(q_rc))
      - 2 * cmd_damping_ * cmd_w_0_ * w_c_;
    // Note: w_c_dot_ = u_c in the paper

    // Integrate using forward Euler:
    q_c_.w() = q_c_.w() + step_size_ * q_c_dot.w();
    q_c_.vec() = q_c_.vec() + step_size_ * q_c_dot.vec();
    w_c_ = w_c_ + step_size_ * w_c_dot_;

    // Calculate body frame values
    w_c_body_frame_ = q_e_.conjugate()._transformVector(w_c_);
    w_c_dot_body_frame_ = q_e_.conjugate()._transformVector(w_c_dot_);
  }

  void AdaptiveController::calculateErrors()
  {
    // Calulate attitude error
    q_e_ = q_c_.conjugate() * q_;
    // Calculate angular velocity error
    w_bc_ = w_ - q_e_.conjugate()._transformVector(w_c_);
  }

	void AdaptiveController::calculateAdaptiveReferenceErrors()
	{
		// Define signals
		Eigen::Matrix3d A_m;
		Eigen::Vector3d r;

		A_m = - k_w_ * Eigen::Matrix3d::Identity() - cross_map(w_c_body_frame_);
		r = w_c_dot_body_frame_
			- k_q_ * quat_log_v(quat_plus_map(q_e_))
			+ k_w_ * w_c_body_frame_;

		// Calculate derivative
		Eigen::Vector3d w_adaptive_model_dot;
		w_adaptive_model_dot = A_m * w_adaptive_model_ + r - k_e_ * e_adaptive_model_;
	
		// Forward euler
		w_adaptive_model_ = w_adaptive_model_ + step_size_ * w_adaptive_model_dot;

		// Calculate adaptive model error
		e_adaptive_model_ = w_adaptive_model_ - w_;
	}


	void AdaptiveController::calculateAdaptiveParameters()
	{
		// Regressor signal
		Phi_ << - w_(2) * w_(1),
						- w_(0) * w_(2),
						- w_(0) * w_(1);

		// Calculate derivatives
		Eigen::Vector3d tau_dist_hat_dot;
		Eigen::Matrix3d Theta_hat_dot;
		Eigen::Matrix3d Lambda_hat_dot;

		tau_dist_hat_dot = - adaptive_gain_tau_ * P_ * e_adaptive_model_;
		Lambda_hat_dot = - adaptive_gain_Lambda_ * P_ * e_adaptive_model_ *	total_input_torques_.transpose();
		Theta_hat_dot = - adaptive_gain_Theta_ * P_ * e_adaptive_model_ * Phi_.transpose();

		// Forward euler
		tau_dist_hat_ = tau_dist_hat_ + step_size_ * tau_dist_hat_dot;
		Lambda_hat_ = Lambda_hat_ + step_size_ * Lambda_hat_dot;
		Theta_hat_ = Theta_hat_ + step_size_ * Theta_hat_dot;
	}

	void AdaptiveController::calculateAdaptiveInput()
	{
		tau_adaptive_ = Lambda_hat_.inverse()
			* (- (Theta_hat_ - Theta_nominal_) * Phi_ - tau_dist_hat_)
			- (Eigen::Matrix3d::Identity() - Lambda_hat_.inverse() * J_nominal_.inverse())
			* tau_baseline_;
	}

  void AdaptiveController::calculateBaselineInput()
  {
    // Calculates inputs for the NED frame!
    // Baseline controller
		// TODO fix these names
    Eigen::Vector3d cancellation_terms = cross_map(w_) * J_nominal_ * w_
      + J_nominal_ * (w_c_dot_body_frame_
          + cross_map(w_) * w_c_body_frame_);

    Eigen::Vector3d feedforward_terms = J_nominal_ * (- k_q_ * quat_log_v(quat_plus_map(q_e_)) - k_w_ * w_bc_);

    tau_baseline_ = cancellation_terms + feedforward_terms;
  }

	void AdaptiveController::calculateTotalInputTorques()
	{
		total_input_torques_ = tau_baseline_;
		if (enable_adaptive_controller_)
			total_input_torques_ += tau_adaptive_;
	}

	// *******
	// Setters
	// *******

	void AdaptiveController::setRefSignal(Eigen::Quaterniond q_ref)
	{
		q_r_ = q_ref;
	}

	void AdaptiveController::setAdaptive(bool enable_adaptive_controller)
	{
		enable_adaptive_controller_ = enable_adaptive_controller;
	}

	// *******
	// Getters
	// *******

	Eigen::Vector3d AdaptiveController::getAdaptiveModelAngVel()
	{
		return w_adaptive_model_;
	}

	Eigen::Vector3d AdaptiveController::getAdaptiveModelError()
	{
		return e_adaptive_model_;
	}

	Eigen::Quaterniond AdaptiveController::getAttCmdSignal()
	{
		return q_c_;
	}

	Eigen::Matrix3d AdaptiveController::getThetaHat()
	{
		return Theta_hat_;
	}

	Eigen::Matrix3d AdaptiveController::getLambdaHat()
	{
		return Lambda_hat_;
	}

	Eigen::Vector3d AdaptiveController::getTauDistHat()
	{
		return tau_dist_hat_;
	}

	Eigen::Vector3d AdaptiveController::getBaselineInput()
	{
		return	tau_baseline_;
	}

	Eigen::Vector3d AdaptiveController::getAdaptiveInput()
	{
		return	tau_adaptive_;
	}

	Eigen::Vector3d AdaptiveController::getInputTorques()
	{
		return	total_input_torques_;
	}

} // namespace controller
