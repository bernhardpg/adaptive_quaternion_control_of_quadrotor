#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller/attitude_controller.h"
#include "controller/position_controller.h"
#include "plot/plotter.h"
#include "tools/quat.h"
#include "simulate/signal_generators.h"

void simulate();
Eigen::Vector3d nedToEnu(Eigen::Vector3d);
