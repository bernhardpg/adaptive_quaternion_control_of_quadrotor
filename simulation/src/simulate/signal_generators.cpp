#include "simulate/signal_generators.h"

Eigen::VectorX<Eigen::Vector3d> getRefTrajCircular(int N)
{
	Eigen::VectorX<Eigen::Vector3d> ref_traj(N);

	double th = 0;
	double radius = 4;
	for (int i = 0; i < N; ++i)
	{
		th = (double)i * (M_PI / (double)N);
		Eigen::Vector3d r_t;
		r_t << radius * cos(th),
					 radius * sin(th),
					 - (i * 2) / (double) N;

		ref_traj(i) = r_t;
	}

	return ref_traj;
}

Eigen::Vector3d getRefSignalSquare(double t)
{
	Eigen::Vector3d ref;
	if (t < 3)
		ref << 0, 0, 0;
	else if (t < 4)
		ref << 0.3, 0.3, 0;
	else if (t < 5)
		ref << 0, 0, 0;
	else if (t < 6)
		ref << -0.3, -0.3, 0;
	else if (t < 7)
		ref << 0,0,0;
	else if (t < 8)
		ref << 0.1,0.1,0;
	else if (t < 9.5)
		ref << 0.35,0.32,0;
	else if (t < 10.0)
		ref << -0.3, 0,0;
	else if (t < 12)
		ref << -0.3,0,0;
	else if (t < 13)
		ref << 0.3,0.2,0;
	else if (t < 13.5)
		ref << 0,0,0;
	else if (t < 15.0)
		ref << 0.15,0.15,0;
	else if (t < 16.5)
		ref << -0.3,-0.2,0;
	else if (t < 18.5)
		ref << 0.2,0,0;
	else if (t < 19)
		ref << 0,0,0;
	else if (t < 3 + 20)
		ref << 0, 0, 0;
	else if (t < 4 + 20)
		ref << 0.3, 0.3, 0;
	else if (t < 5 + 20)
		ref << 0, 0, 0;
	else if (t < 6 + 20)
		ref << -0.3, -0.3, 0;
	else if (t < 7 + 20)
		ref << 0,0,0;
	else if (t < 8 + 20)
		ref << 0.1,0.1,0;
	else if (t < 9.5 + 20)
		ref << 0.35,0.32,0;
	else if (t < 10.0 + 20)
		ref << -0.3, 0,0;
	else if (t < 12 + 20)
		ref << -0.3,0,0;
	else if (t < 13 + 20)
		ref << 0.3,0.2,0;
	else if (t < 13.5 + 20)
		ref << 0,0,0;
	else if (t < 15.0 + 20)
		ref << 0.15,0.15,0;
	else if (t < 16.5 + 20)
		ref << -0.3,-0.2,0;
	else if (t < 18.5 + 20)
		ref << 0.2,0,0;
	else if (t < 19 + 20)
		ref << 0,0,0;

	return ref;
}
