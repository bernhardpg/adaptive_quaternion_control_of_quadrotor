#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller/attitude_controller.h"
#include "plot/plotter.h"

void simulate();
Eigen::Vector3d get_ref_signal(double t);
