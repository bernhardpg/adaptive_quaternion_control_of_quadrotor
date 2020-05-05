#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::VectorX<Eigen::Vector3d> getRefTrajCircular(int N);
Eigen::Vector3d getRefSignalSquare(double t);
