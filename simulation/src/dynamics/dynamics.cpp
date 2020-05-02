#include "dynamics/dynamics.h"

Eigen::Vector3d get_ref_signal(double t)
{
	Eigen::Vector3d ref;

	/*
	if (t < 2)
		ref << 0, 0, 0;
	else if (t < 17.5)
		ref << 0.3, 0.2, 0;
	else if(t < 19)
		ref << 0, 0, 0;
	else if(t < 21)
		ref << 0.3, 0, 0;*/

	/*
	else if (t < 5)
		ref << 0, 0, 0;
	else if (t < 7)
		ref << 0.2, -0.3, 0;
	else if (t < 10)
		ref << 0, 0, 0;

	else if (t < 2.5 + 10)
		ref << 0.3, 0.2, 0;
	else if (t < 5 + 10)
		ref << 0, 0, 0;
	else if (t < 7 + 10)
		ref << 0.2, -0.3, 0;
	else if (t < 10 + 10)
		ref << 0, 0, 0;
		*/

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



void simulate()
{
	double h = pow(10, -3);
	int T = 50; // s, end time
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

	Eigen::VectorX<Eigen::Vector3d> ws_adaptive_model(N);

	Eigen::VectorX<Eigen::Vector3d> baseline_input_torques(N);
	Eigen::VectorX<Eigen::Vector3d> adaptive_input_torques(N);

	Eigen::VectorX<Eigen::Matrix3d> Theta_hats(N);
	Eigen::VectorX<Eigen::Matrix3d> Lambda_hats(N);
	Eigen::VectorX<Eigen::Vector3d> tau_dist_hats(N);

	controller::AdaptiveController controller;

	// ************
	// Simulate dynamics
	// ************
	double t = 0;

	for (int i = 0; i < N; ++i)
	{
		t += h;
		ts.push_back(t);

		// **********
		// Controller
		// **********

		// Send current reference signal to controller
		Eigen::Vector3d ref = get_ref_signal(t);
		controller.setRefSignal(EulerToQuat(ref));

		// Enable adaptive controller after 10 seconds
		if (t > 15)
			controller.setAdaptive(true);

		// Calculate control input
		controller.controllerCallback(q, w, t);
		tau_ext = controller.getInputTorques();

		// ********
		// Dynamics
		// ********

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

		// ************
		// Store values
		// ************

		qs(i) = q;
		ws(i) = w;
		ws_adaptive_model(i) = controller.getAdaptiveModelAngVel();
		cmds(i) = controller.getAttCmdSignal();
		refs(i) = ref;
		baseline_input_torques(i) = controller.getBaselineInput();
		adaptive_input_torques(i) = controller.getAdaptiveInput();
		Theta_hats(i) = controller.getThetaHat();
		Lambda_hats(i) = controller.getLambdaHat();
		tau_dist_hats(i) = controller.getTauDistHat();
	}

	std::cout << "Plotting" << std::endl;
	//plot_input_torques(baseline_input_torques, adaptive_input_torques, ts);
	//plot_adaptive_ref_model(ws, ws_adaptive_model, ts);
	plot_attitude(qs, refs, cmds, ts);
	//plot_adaptive_params(Theta_hats, Lambda_hats, tau_dist_hats, ts);
	//plot_cmd(cmds, refs, ts);
}
