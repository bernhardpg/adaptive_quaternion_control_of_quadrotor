#include "tools/quat.h"

// Note: also called hat map in litterature
Eigen::Matrix3d cross_map(Eigen::Vector3d v)
{
	Eigen::Matrix3d v_hat;
	v_hat << 0, -v(2), v(1),
					v(2), 0, -v(0),
				-v(1), v(0), 0;
	return v_hat;
}

// Opposite of hat map
Eigen::Vector3d vee_map(Eigen::Matrix3d v_hat)
{
	Eigen::Vector3d v;
	v << v_hat(2,1), v_hat(0,2), v_hat(0,1);
	return v;
}

// Returns the direct map of the quaternion logarithm to R^3 (instead of R^4)
Eigen::Vector3d quat_log_v(Eigen::Quaterniond q)
{
	Eigen::AngleAxis<double> aa(q);
	return (aa.angle() / 2) * aa.axis();
}

// Returns the short rotation quaternion with angle <= pi
Eigen::Quaterniond quat_plus_map(Eigen::Quaterniond q)
{
	return q.w() >= 0 ? q : Eigen::Quaterniond(-q.w(), q.x(), q.y(), q.z());
}

// TODO Change order ehre! wtf
Eigen::Quaterniond EulerToQuat(double yaw, double pitch, double roll)
	// yaw (Z), pitch (Y), roll (X)
{
	// Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	Eigen::Quaterniond q;
	q.w() = cr * cp * cy + sr * sp * sy;
	q.x() = sr * cp * cy - cr * sp * sy;
	q.y() = cr * sp * cy + sr * cp * sy;
	q.z() = cr * cp * sy - sr * sp * cy;

	return q;
}

Eigen::Vector3d QuatToEuler(Eigen::Quaterniond q) {
	Eigen::Vector3d euler;

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
	euler(0) = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
	if (std::abs(sinp) >= 1)
			euler(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
			euler(1) = std::asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
	euler(2) = std::atan2(siny_cosp, cosy_cosp);

	return euler;
}
