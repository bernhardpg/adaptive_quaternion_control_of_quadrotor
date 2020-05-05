#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "plot/matplotlibcpp.h"
#include "tools/quat.h"

namespace plt = matplotlibcpp;

void plot_position3d(
		Eigen::VectorX<Eigen::Vector3d> poss
		);

void plot_position3d(
		Eigen::VectorX<Eigen::Vector3d> poss,
		Eigen::VectorX<Eigen::Vector3d> ref_poss
		);

void plot_position(
		Eigen::VectorX<Eigen::Vector3d> poss,
		std::vector<double> ts
		);

void plot_position(
		Eigen::VectorX<Eigen::Vector3d> poss,
		Eigen::VectorX<Eigen::Vector3d> ref_poss,
		std::vector<double> ts
		);

void plot_adaptive_params(
		Eigen::VectorX<Eigen::Matrix3d> Theta_hat,
		Eigen::VectorX<Eigen::Matrix3d> Lambda_hat,
		Eigen::VectorX<Eigen::Vector3d> tau_dist_hat,
		std::vector<double> ts
		);

void plot_input_torques(
		Eigen::VectorX<Eigen::Vector3d> input_torques,
		Eigen::Vector3d limits,
		std::vector<double> ts
		);

void plot_input_torques(
		Eigen::VectorX<Eigen::Vector3d> baseline_input_torques,
		Eigen::VectorX<Eigen::Vector3d> adaptive_input_torques,
		std::vector<double> ts
		);

void plot_adaptive_ref_model(
		Eigen::VectorX<Eigen::Vector3d> ws,
		Eigen::VectorX<Eigen::Vector3d> ws_adaptive_model,
		std::vector<double> ts
		);

void plot_attitude(
		Eigen::VectorX<Eigen::Quaterniond> qs,
		Eigen::VectorX<Eigen::Vector3d> refs,
		Eigen::VectorX<Eigen::Quaterniond> cmds,
		std::vector<double> ts
		);

void plot_cmd(
		Eigen::VectorX<Eigen::Quaterniond> cmds,
		std::vector<double> ts
		);
