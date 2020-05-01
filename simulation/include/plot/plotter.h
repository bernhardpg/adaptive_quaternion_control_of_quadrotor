#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "plot/matplotlibcpp.h"
#include "tools/quat.h"

namespace plt = matplotlibcpp;

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
