#pragma once

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "tools/quat.h"

namespace controller {
  typedef struct
  {
    Eigen::Vector3d tau;
    double F;
  } input_t;

  class AdaptiveController
  {
    public:
      AdaptiveController();

			void controllerCallback(Eigen::Quaterniond q, Eigen::Vector3d w, double t);
			Eigen::Vector3d getInputTorques();
			Eigen::Quaterniond getAttCmdSignal();
			Eigen::Vector3d getAdaptiveModelAngVel();
			Eigen::Vector3d getAdaptiveModelError();
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
      input_t input_;
      double k_q_;
      double k_w_;

			// *******
			// Adaptive controller
			// *******
			Eigen::DiagonalMatrix<double,3,3> Lambda_hat_; // Control effectiveness
			Eigen::DiagonalMatrix<double,3,3> Theta_hat_; // Adaptive parameters
			Eigen::Vector3d tau_dist_hat_; // Angular acceleration disturbance

			Eigen::Vector3d Phi_; // Known vector of basis functions
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
			void calculateAdaptiveParameters();
      void calculateErrors();
      void computeInput();

      double saturate(double v, double min, double max); // TODO move somewhere else
  };


}
