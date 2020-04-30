#include "dynamics/dynamics.h"
//#include "controller/attitude_controller.h"
#include "plot/plotter.h"

void simulate()
{
	double h = pow(10, -3);
	int T = 100; // s, end time
	int N = T / h; // Number of time steps

	Eigen::Matrix3d J;
	J << 0.07, 0, 0,
				0, 0.08, 0,
				0, 0, 0.12; // From .urdf file

	// Initial values
	Eigen::Quaterniond q(1,0,0,0);
	Eigen::Quaterniond q_dot;
	Eigen::Vector3d w(0,0,0);
	Eigen::Vector3d w_dot;

	Eigen::Vector3d tau_ext(0.5,0,0);

	// Store values
	Eigen::VectorX<Eigen::Quaterniond> qs(N);
	Eigen::VectorX<Eigen::Vector3d> ws(N);
	std::vector<double> ts;

	// ************
	// Simulate dynamics
	// ************
	double t = 0;

	for (int i = 0; i < N; ++i)
	{
		t += h;
		ts.push_back(t);

		//Calculate derivatives
		Eigen::Quaterniond w_quat;
		w_quat.w() = 0;
		w_quat.vec() = 0.5 * w;

		q_dot = q * w_quat;
		w_dot = J.inverse() * (- w.cross(J * w) + tau_ext);

		q.w() = q.w() + h * q_dot.w();
		q.vec() = q.vec() + h * q_dot.vec();
		w = w + h * w_dot;

		qs(i) = q;
		ws(i) = w;
	}


	plot_state(qs, ws, ts);

}
