#include "dynamics/dynamics.h"

Eigen::Vector3d get_ref_signal(double t)
{
	Eigen::Vector3d ref;

	if (t < 3)
		ref << 0, 0, 0;
	else if (t < 4)
		ref << 0.3, 0, 0;
	else if (t < 8)
		ref << 0, 0, 0;
	else if (t < 10)
		ref << -0.3, 0, 0;
	else if (t < 15)
		ref << 0,0,0;
	else if (t < 16)
		ref << 0,0.3,0;
	else
		ref << 0,0,0;

	return ref;
}

void simulate()
{
	double h = pow(10, -3);
	int T = 20; // s, end time
	int N = T / h; // Number of time steps

	Eigen::Matrix3d J;
	J << 0.07, 0, 0,
				0, 0.08, 0,
				0, 0, 0.12; // From .urdf file

	// Initial values
	Eigen::Quaterniond q = EulerToQuat(0, 0, 0);
	Eigen::Quaterniond q_dot;
	Eigen::Vector3d w(0,0,0);
	Eigen::Vector3d w_dot;

	Eigen::Vector3d tau_ext(0,0,0);

	// Store values
	Eigen::VectorX<Eigen::Quaterniond> qs(N);
	Eigen::VectorX<Eigen::Vector3d> ws(N);
	std::vector<double> ts;

	Eigen::VectorX<Eigen::Quaterniond> cmds(N);
	Eigen::VectorX<Eigen::Vector3d> refs(N);

	controller::AdaptiveController controller;

	// ************
	// Simulate dynamics
	// ************
	double t = 0;

	for (int i = 0; i < N; ++i)
	{
		t += h;
		ts.push_back(t);

		// Send current reference signal to controller
		Eigen::Vector3d ref = get_ref_signal(t);
		controller.setRefSignal(EulerToQuat(ref));

		// Calculate control input
		controller.controllerCallback(q, w, t);
		tau_ext = controller.getInputTorques();

		// Calculate derivatives
		Eigen::Quaterniond w_quat;
		w_quat.w() = 0;
		w_quat.vec() = 0.5 * w;

		q_dot = q * w_quat;
		w_dot = J.inverse() * (- w.cross(J * w) + tau_ext);

		// Integrate using forward euler
		q.w() = q.w() + h * q_dot.w();
		q.vec() = q.vec() + h * q_dot.vec();
		w = w + h * w_dot;

		qs(i) = q;
		ws(i) = w;
		cmds(i) = controller.getCmdSignal();
		refs(i) = ref;
	}

	plot_attitude(qs, refs, ts);
	//plot_cmd(cmds, refs, ts);
}
