#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "plot/matplotlibcpp.h"

namespace plt = matplotlibcpp;

void plot_state(Eigen::VectorX<Eigen::Quaterniond> qs, Eigen::VectorX<Eigen::Vector3d> ws);
