#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "tools/quat.h"

namespace controller{
  class PositionController
  {
    public:
      PositionController();

			void controllerCallback(
					Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Quaterniond q
					);

			Eigen::Quaterniond getInputAttitude();
			double getInputThrust();

			void setRefSignal(Eigen::Vector3d pos_ref);
			void setRefSignal(Eigen::Vector3d pos_ref, Eigen::Vector3d vel_ref);

    private:
			// *******
      // Signals
			// *******

      // State
      Eigen::Vector3d pos_;
      Eigen::Vector3d vel_;
			Eigen::Quaterniond q_;
      Eigen::Vector3d thrust_direction_;

      // Errors
      Eigen::Vector3d e_pos_;
      Eigen::Vector3d e_vel_;

      // Command
      Eigen::Vector3d pos_ref_;
      Eigen::Vector3d vel_ref_;

			// *********
			// Controller
			// *********
			Eigen::MatrixXd K_;
			double F_thrust_;
			Eigen::Quaterniond q_desired_;

			// *******
      // Model parameters
			// *******

      double m_;
      double g_;
      double max_thrust_;

			// *******
      // Functions
			// *******

      void init();
      void calculateErrors();
      void computeInput();

      double saturate(double v, double min, double max);
  };
}
