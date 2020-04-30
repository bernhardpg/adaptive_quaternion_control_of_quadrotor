#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

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
      Eigen::Vector3d w_c_body_frame;
      Eigen::Vector3d w_c_dot_body_frame;

      // *******
      // Model parameters
      // *******
      double max_thrust_;
      double max_torque_;
      Eigen::Matrix3d J_; // Inertia matrix

      // *******
      // Controller
      // *******
      input_t input_;
      double k_q_;
      double k_w_;
      double time_step_;
      bool stabilize_curr_pos_;

      // *******
      // Functions
      // *******
      void init();
      void refSignalCallback();
      void generateCommandSignal();
			void controllerCallback(Eigen::Quaterniond q, Eigen::Vector3d w, double t);
      void calculateErrors();
      void computeInput();
      void publishCommand();

      double saturate(double v, double min, double max); // TODO move somewhere else

      // TODO move somewhere else
      Eigen::Matrix3d cross_map(Eigen::Vector3d v);
      Eigen::Vector3d vee_map(Eigen::Matrix3d v_hat); // Inverse of cross_map
      Eigen::Vector3d quat_log_v(Eigen::Quaterniond q);
      Eigen::Quaterniond quat_plus_map(Eigen::Quaterniond q);
  };

  Eigen::Vector3d QuatToEuler(Eigen::Quaterniond q);
  Eigen::Quaterniond EulerToQuat(double yaw, double pitch, double roll);

}
