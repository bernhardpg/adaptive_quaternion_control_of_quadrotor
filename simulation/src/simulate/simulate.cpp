#include "simulate/simulate.h"

void simulate()
{
	// Simulation parameters
	double step_size = pow(10, -3);
	int T = 50; // s, end time
	int N = T / step_size; // Number of time steps

	double arm_length = 0.2;
	double max_rotor_thrust = 1.4 * 9.81; // N
	double max_thrust = max_rotor_thrust * 4;
	double max_torque = arm_length * max_rotor_thrust * 2;

	// Model parameters
	Eigen::Matrix3d J;
	J << 0.07, 0, 0,
				0, 0.08, 0,
				0, 0, 0.12; // From .urdf file
	double m = 2.856;
	Eigen::Vector3d gravity(0,0,9.81);

	// Variables
	Eigen::Quaterniond q = EulerToQuat(0, 0, 0);
	Eigen::Quaterniond q_dot;
	Eigen::Vector3d w(0,0,0);
	Eigen::Vector3d w_dot;
	Eigen::Vector3d pos(0,0,0);
	Eigen::Vector3d pos_dot(0,0,0);
	Eigen::Vector3d pos_ddot(0,0,0);

	Eigen::Quaterniond ref_attitude = EulerToQuat(0, 0, 0);
	Eigen::Vector3d F_thrust(0,0,0);
	Eigen::Vector3d tau_ext(0,0,0);
	Eigen::Vector3d tau_c(0,0,0);
	Eigen::Vector3d tau_dist(0,0,0);

	Eigen::VectorX<Eigen::Vector3d> ref_traj(N);

	// Store values
	std::vector<double> ts;
	Eigen::VectorX<Eigen::Quaterniond> qs(N);
	Eigen::VectorX<Eigen::Vector3d> ws(N);
	Eigen::VectorX<Eigen::Vector3d> poss(N);
	Eigen::VectorX<Eigen::Vector3d> ref_poss(N);
	Eigen::VectorX<Eigen::Vector3d> pos_dots(N);
	Eigen::VectorX<Eigen::Quaterniond> cmds(N);
	Eigen::VectorX<Eigen::Vector3d> ref_attitudes(N);
	Eigen::VectorX<Eigen::Vector3d> ws_adaptive_model(N);
	Eigen::VectorX<Eigen::Vector3d> input_torques(N);
	Eigen::VectorX<Eigen::Vector3d> baseline_input_torques(N);
	Eigen::VectorX<Eigen::Vector3d> adaptive_input_torques(N);
	Eigen::VectorX<Eigen::Matrix3d> Theta_hats(N);
	Eigen::VectorX<Eigen::Matrix3d> Lambda_hats(N);
	Eigen::VectorX<Eigen::Vector3d> tau_dist_hats(N);

	// ************
	// Simulate dynamics
	// ************
	
	double t = 0;

	controller::AdaptiveController attitude_controller;
	controller::PositionController position_controller;

	position_controller.setRefSignal(Eigen::Vector3d(0,0,-1));

	std::string test_mode = "att_square_weight";
	// Test modes:
	//	att_square
	//	att_square_weight
	//  pos_tracking
	//  pos_tracking_weight

	if (test_mode == "pos_tracking" || test_mode == "pos_tracking_weight")
	{
		ref_traj = getRefTrajCircular(N);
		attitude_controller.setAdaptive(true);
	}
	else if (test_mode == "att_square")
	{
		attitude_controller.setAdaptive(false);
	}
	else if (test_mode == "att_square_weight")
	{
		attitude_controller.setAdaptive(false);
	}


	for (int i = 0; i < N; ++i)
	{
		t += step_size;
		ts.push_back(t);

		// **********
		// Controller
		// **********

		if (test_mode == "att_square" || test_mode == "att_square_weight")
		{
			// Enable adaptive controller after 10 seconds
			if (t > 15)
				attitude_controller.setAdaptive(true);

			// Send current reference signal to controller
			ref_attitude = EulerToQuat(getRefSignalSquare(t));
			attitude_controller.setRefSignal(ref_attitude);
		}
		if (test_mode == "att_square_weight")
		{
			double roll = QuatToEuler(q)(0);
			tau_dist << 0.20 * 9.81 * arm_length * cos(roll), 0, 0;
		}
		if (test_mode == "pos_tracking" || test_mode == "pos_tracking_weight")
		{
			position_controller.setRefSignal(ref_traj[i]);

			if (test_mode == "pos_tracking_weight")
			{
				if (t > 25)
				{
					double roll = QuatToEuler(q)(0);
					tau_dist << 0.3 * 9.81 * arm_length * cos(roll), 0, 0;
				}
			}
		}

		// Calculate control input
		position_controller.controllerCallback(pos, pos_dot, q);
		F_thrust(2) = -saturate(position_controller.getInputThrust(), 0, max_thrust);

		if (test_mode == "pos_tracking" || test_mode == "pos_tracking_weight")
		{
			ref_attitude = position_controller.getInputAttitude();
			attitude_controller.setRefSignal(ref_attitude);
		}

		attitude_controller.controllerCallback(q, w, t);
		tau_c = attitude_controller.getInputTorques();

		tau_c << saturate(tau_c(0), -max_torque, max_torque),
						 saturate(tau_c(1), -max_torque, max_torque),
						 saturate(tau_c(2), -max_torque, max_torque);

		// ********
		// Dynamics : Everything is in NED frame
		// ********
		tau_ext = tau_c + tau_dist;

		// Attitude
		// Calculate derivatives
		Eigen::Quaterniond w_quat;
		w_quat.w() = 0;
		w_quat.vec() = 0.5 * w;

		q_dot = q * w_quat;
		w_dot = J.inverse() * (- w.cross(J * w) + tau_ext);

		// Integrate using forward euler
		q.w() = q.w() + step_size * q_dot.w();
		q.vec() = q.vec() + step_size * q_dot.vec();
		w = w + step_size * w_dot;

		// Position
		// Calculate derivatives
		pos_dot = pos_dot;
		pos_ddot = q._transformVector(F_thrust / m) + gravity;

		// Integrate using forward euler
		pos = pos + step_size * pos_dot;
		pos_dot = pos_dot + step_size * pos_ddot;

		// ************
		// Store values
		// ************

		qs(i) = q;
		ws(i) = w;
		poss(i) = nedToEnu(pos); // Convert to ENU for plotting
		ref_poss(i) = nedToEnu(ref_traj(i)); // Convert to ENU for plotting
		pos_dots(i) = nedToEnu(pos_dot);
		ws_adaptive_model(i) = attitude_controller.getAdaptiveModelAngVel();
		cmds(i) = attitude_controller.getAttCmdSignal();
		ref_attitudes(i) = QuatToEuler(ref_attitude);
		input_torques(i) = tau_c;
		baseline_input_torques(i) = attitude_controller.getBaselineInput();
		adaptive_input_torques(i) = attitude_controller.getAdaptiveInput();
		Theta_hats(i) = attitude_controller.getThetaHat();
		Lambda_hats(i) = attitude_controller.getLambdaHat();
		tau_dist_hats(i) = attitude_controller.getTauDistHat();
	}

	std::cout << "Plotting" << std::endl;
	plot_attitude(qs, ref_attitudes, cmds, ts);
	plot_position(poss, ref_poss, ts);
	plot_position3d(poss, ref_poss);
	//plot_input_torques(baseline_input_torques, adaptive_input_torques, ts);
	plot_input_torques(input_torques, Eigen::Vector3d(max_torque, max_torque, max_torque), ts);
	//plot_adaptive_ref_model(ws, ws_adaptive_model, ts);
	plot_adaptive_params(Theta_hats, Lambda_hats, tau_dist_hats, ts);
	//plot_cmd(cmds, ref_attitudes, ts);
}
