#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>

#include "tools/quat.h"

void export_position(
		Eigen::VectorX<Eigen::Vector3d> poss,
		Eigen::VectorX<Eigen::Vector3d> ref_poss,
		std::vector<double> ts
		);

void export_attitude(
		Eigen::VectorX<Eigen::Quaterniond> qs,
		Eigen::VectorX<Eigen::Vector3d> refs,
		Eigen::VectorX<Eigen::Quaterniond> cmds,
		std::vector<double> ts
		);

void export_input_torques(
		Eigen::VectorX<Eigen::Vector3d> input_torques,
		Eigen::Vector3d limits,
		std::vector<double> ts
		);

void export_adaptive_params(
		Eigen::VectorX<Eigen::Matrix3d> Theta_hat,
		Eigen::VectorX<Eigen::Matrix3d> Lambda_hat,
		Eigen::VectorX<Eigen::Vector3d> tau_dist_hat,
		std::vector<double> ts
		);
