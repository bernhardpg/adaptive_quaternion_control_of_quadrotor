#pragma once

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "tools/quat.h"

namespace controller {
  class AdaptiveController
  {
    public:
      AdaptiveController();

			void controllerCallback(Eigen::Quaterniond q, Eigen::Vector3d w, double t);

			Eigen::Quaterniond getAttCmdSignal();
			Eigen::Vector3d getAdaptiveModelAngVel();
			Eigen::Vector3d getAdaptiveModelError();
			Eigen::Vector3d getBaselineInput();
			Eigen::Vector3d getAdaptiveInput();
			Eigen::Vector3d getInputTorques();
			Eigen::Matrix3d getThetaHat();
			Eigen::Matrix3d getLambdaHat();
			Eigen::Vector3d getTauDistHat();

			void setAdaptive(bool enable_adaptive_controller);
			void setRefSignal(Eigen::Quaterniond q_ref);

    private:
			double t_;

      // *******
      // Signals
      // *******
      // State
      Eigen::Quaterniond q_; // Rotation from body to inertial frame
      Eigen::Vector3d w_; // Angular velocity between body and inertial frame

      // Errors
      Eigen::Quaterniond q_e_; // From body to command frame
      Eigen::Vector3d w_bc_; // Between body and command frame, given in body frame
      // Reference
      // (only needs to be piecewise continuous)
      Eigen::Quaterniond q_r_;
      Eigen::Vector3d att_ref_euler_; // radians

      // Command
      // (generated from the reference signal)
      Eigen::Quaterniond q_c_;
      Eigen::Vector3d w_c_; // Given in command frame
      Eigen::Vector3d w_c_dot_;
      Eigen::Vector3d w_c_body_frame_;
      Eigen::Vector3d w_c_dot_body_frame_;

      // *******
      // Model parameters
      // *******
      Eigen::Matrix3d J_nominal_; // Inertia matrix

      // *******
      // Controller
      // *******
			Eigen::Vector3d tau_baseline_; // Control input from baseline controller
			Eigen::Vector3d tau_adaptive_; // Control input from adaptive controller
      double k_q_;
      double k_w_;
			Eigen::Vector3d total_input_torques_; // Control input from adaptive controller

			// *******
			// Adaptive controller
			// *******
			double k_e_; // Gain on error feedback in closed loop ref model
			bool enable_adaptive_controller_;

			Eigen::Matrix3d adaptive_gain_Theta_;
			Eigen::Matrix3d adaptive_gain_Lambda_;
			double adaptive_gain_tau_;

			Eigen::Matrix3d Lambda_hat_; // Control effectiveness
			Eigen::Matrix3d Theta_hat_; // Adaptive parameters
			Eigen::Vector3d tau_dist_hat_; // Angular acceleration disturbance

			Eigen::Vector3d Phi_; // Regressor: known vector of basis functions
			Eigen::Matrix3d Theta_nominal_;
			Eigen::Matrix3d P_;

			Eigen::Vector3d w_adaptive_model_;
			Eigen::Vector3d e_adaptive_model_;

      // *******
			// Trajectory generator
      // *******
      double time_step_;
      double cmd_w_0_; // Bandwidth
      double cmd_damping_;

      // *******
      // Functions
      // *******
      void init();
      void generateCommandSignal();
      void calculateErrors();
			void calculateAdaptiveParameters();
			void calculateAdaptiveReferenceErrors();
			void calculateAdaptiveInput();
      void calculateBaselineInput();
			void calculateTotalInputTorques();

      double saturate(double v, double min, double max); // TODO move somewhere else
  };


}
