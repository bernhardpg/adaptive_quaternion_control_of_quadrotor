#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>

Eigen::Vector3d QuatToEuler(Eigen::Quaterniond q);
Eigen::Quaterniond EulerToQuat(double yaw, double pitch, double roll);
Eigen::Quaterniond EulerToQuat(Eigen::Vector3d euler);

Eigen::Matrix3d cross_map(Eigen::Vector3d v);
Eigen::Vector3d vee_map(Eigen::Matrix3d v_hat); // Inverse of cross_map
Eigen::Vector3d quat_log_v(Eigen::Quaterniond q);
Eigen::Quaterniond quat_plus_map(Eigen::Quaterniond q);


