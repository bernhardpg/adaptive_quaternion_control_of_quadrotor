#include "plot/plotter.h"

void plot_position3d(
		Eigen::VectorX<Eigen::Vector3d> poss
		)
{
	std::vector<double> pos_xs, pos_ys, pos_zs;

	for (int i = 0; i < poss.size(); ++i)
	{
		pos_xs.push_back(poss(i)(0));
		pos_ys.push_back(poss(i)(1));
		pos_zs.push_back(poss(i)(2));
	}

	plt::plot3(pos_xs, pos_ys, pos_zs);
	plt::title("3D-position");
	plt::show();
}

void plot_position3d(
		Eigen::VectorX<Eigen::Vector3d> poss,
		Eigen::VectorX<Eigen::Vector3d> ref_poss
		)
{
	std::vector<double> pos_xs, pos_ys, pos_zs;
	std::vector<double> ref_pos_xs, ref_pos_ys, ref_pos_zs;

	for (int i = 0; i < poss.size(); ++i)
	{
		pos_xs.push_back(poss(i)(0));
		pos_ys.push_back(poss(i)(1));
		pos_zs.push_back(poss(i)(2));

		ref_pos_xs.push_back(ref_poss(i)(0));
		ref_pos_ys.push_back(ref_poss(i)(1));
		ref_pos_zs.push_back(ref_poss(i)(2));
	}

	plt::plot3(ref_pos_xs, ref_pos_ys, ref_pos_zs);
	plt::plot3(pos_xs, pos_ys, pos_zs);
	plt::title("3D-position");
	plt::show();
}

void plot_position(
		Eigen::VectorX<Eigen::Vector3d> poss,
		Eigen::VectorX<Eigen::Vector3d> ref_poss,
		std::vector<double> ts
		)
{
	std::vector<double> pos_xs, pos_ys, pos_zs;
	std::vector<double> ref_pos_xs, ref_pos_ys, ref_pos_zs;

	for (int i = 0; i < ts.size(); ++i)
	{
		pos_xs.push_back(poss(i)(0));
		pos_ys.push_back(poss(i)(1));
		pos_zs.push_back(poss(i)(2));

		ref_pos_xs.push_back(ref_poss(i)(0));
		ref_pos_ys.push_back(ref_poss(i)(1));
		ref_pos_zs.push_back(ref_poss(i)(2));
	}

	plt::plot(ts, pos_xs);
	plt::plot(ts, ref_pos_xs);
	plt::title("x-pos");
	plt::show();

	plt::plot(ts, pos_ys);
	plt::plot(ts, ref_pos_ys);
	plt::title("y-pos");
	plt::show();

	plt::plot(ts, pos_zs);
	plt::plot(ts, ref_pos_zs);
	plt::title("z-pos");
	plt::show();
}

void plot_position(
		Eigen::VectorX<Eigen::Vector3d> poss,
		std::vector<double> ts
		)
{
	std::vector<double> pos_xs, pos_ys, pos_zs;

	for (int i = 0; i < ts.size(); ++i)
	{
		pos_xs.push_back(poss(i)(0));
		pos_ys.push_back(poss(i)(1));
		pos_zs.push_back(poss(i)(2));
	}

	plt::plot(ts, pos_xs);
	plt::title("x-pos");
	plt::show();

	plt::plot(ts, pos_ys);
	plt::title("y-pos");
	plt::show();

	plt::plot(ts, pos_zs);
	plt::title("z-pos");
	plt::show();
}

void plot_adaptive_params(
		Eigen::VectorX<Eigen::Matrix3d> Theta_hat,
		Eigen::VectorX<Eigen::Matrix3d> Lambda_hat,
		Eigen::VectorX<Eigen::Vector3d> tau_dist_hat,
		std::vector<double> ts
		)
{
	std::vector<double> Theta_hat_11, Theta_hat_12, Theta_hat_13,
											Theta_hat_21, Theta_hat_22, Theta_hat_23,
											Theta_hat_31, Theta_hat_32, Theta_hat_33;

	std::vector<double> Lambda_hat_11, Lambda_hat_12, Lambda_hat_13,
											Lambda_hat_21, Lambda_hat_22, Lambda_hat_23,
											Lambda_hat_31, Lambda_hat_32, Lambda_hat_33;

	std::vector<double> tau_dist_x, tau_dist_y, tau_dist_z;

	for (int i = 0; i < ts.size(); ++i)
	{
		Theta_hat_11.push_back(Theta_hat(i)(0,0));
		Theta_hat_12.push_back(Theta_hat(i)(0,1));
		Theta_hat_13.push_back(Theta_hat(i)(0,2));

		Theta_hat_21.push_back(Theta_hat(i)(1,0));
		Theta_hat_22.push_back(Theta_hat(i)(1,1));
		Theta_hat_23.push_back(Theta_hat(i)(1,2));

		Theta_hat_31.push_back(Theta_hat(i)(2,0));
		Theta_hat_32.push_back(Theta_hat(i)(2,1));
		Theta_hat_33.push_back(Theta_hat(i)(2,2));

		Lambda_hat_11.push_back(Lambda_hat(i)(0,0));
		Lambda_hat_12.push_back(Lambda_hat(i)(0,1));
		Lambda_hat_13.push_back(Lambda_hat(i)(0,2));

		Lambda_hat_21.push_back(Lambda_hat(i)(1,0));
		Lambda_hat_22.push_back(Lambda_hat(i)(1,1));
		Lambda_hat_23.push_back(Lambda_hat(i)(1,2));

		Lambda_hat_31.push_back(Lambda_hat(i)(2,0));
		Lambda_hat_32.push_back(Lambda_hat(i)(2,1));
		Lambda_hat_33.push_back(Lambda_hat(i)(2,2));

		tau_dist_x.push_back(tau_dist_hat(i)(0));
		tau_dist_y.push_back(tau_dist_hat(i)(1));
		tau_dist_z.push_back(tau_dist_hat(i)(2));
	}

	plt::plot(ts, Theta_hat_11);
	plt::plot(ts, Theta_hat_12);
	plt::plot(ts, Theta_hat_13);
	plt::plot(ts, Theta_hat_21);
	plt::plot(ts, Theta_hat_22);
	plt::plot(ts, Theta_hat_23);
	plt::plot(ts, Theta_hat_31);
	plt::plot(ts, Theta_hat_32);
	plt::plot(ts, Theta_hat_33);
	plt::title("Theta_hat");
	plt::show();

	plt::plot(ts, Lambda_hat_11);
	plt::plot(ts, Lambda_hat_12);
	plt::plot(ts, Lambda_hat_13);
	plt::plot(ts, Lambda_hat_21);
	plt::plot(ts, Lambda_hat_22);
	plt::plot(ts, Lambda_hat_23);
	plt::plot(ts, Lambda_hat_31);
	plt::plot(ts, Lambda_hat_32);
	plt::plot(ts, Lambda_hat_33);
	plt::title("Lambda_hat");
	plt::show();

	plt::plot(ts, tau_dist_x);
	plt::plot(ts, tau_dist_y);
	plt::plot(ts, tau_dist_z);
	plt::title("Tau_dist");
	plt::show();
}


void plot_input_torques(
		Eigen::VectorX<Eigen::Vector3d> baseline_input_torques,
		Eigen::VectorX<Eigen::Vector3d> adaptive_input_torques,
		std::vector<double> ts
		)
{
	std::vector<double> baseline_tau_x, baseline_tau_y, baseline_tau_z;
	std::vector<double> adaptive_tau_x, adaptive_tau_y, adaptive_tau_z;
	for (int i = 0; i < baseline_input_torques.size(); ++i)
	{
		baseline_tau_x.push_back(baseline_input_torques(i)(0));
		baseline_tau_y.push_back(baseline_input_torques(i)(1));
		baseline_tau_z.push_back(baseline_input_torques(i)(2));

		adaptive_tau_x.push_back(adaptive_input_torques(i)(0));
		adaptive_tau_y.push_back(adaptive_input_torques(i)(1));
		adaptive_tau_z.push_back(adaptive_input_torques(i)(2));
	}

//	plt::plot(ts, baseline_tau_x, "b--");
	plt::plot(ts, adaptive_tau_x);
	plt::title("tau_x");
	plt::show();

//	plt::plot(ts, baseline_tau_y, "b--");
	plt::plot(ts, adaptive_tau_y);
	plt::title("tau_y");
	plt::show();

//	plt::plot(ts, baseline_tau_y, "b--");
	plt::plot(ts, adaptive_tau_y);
	plt::title("tau_y");
	plt::show();
}

void plot_adaptive_ref_model(
		Eigen::VectorX<Eigen::Vector3d> ws,
		Eigen::VectorX<Eigen::Vector3d> ws_adaptive_model,
		std::vector<double> ts
		)
{
	std::vector<double> w_x, w_y, w_z;
	std::vector<double> w_x_adaptive_model, w_y_adaptive_model, w_z_adaptive_model;

	for (int i = 0; i < ws.size(); ++i)
	{
		w_x.push_back(ws(i)(0));
		w_y.push_back(ws(i)(1));
		w_z.push_back(ws(i)(2));

		w_x_adaptive_model.push_back(ws_adaptive_model(i)(0));
		w_y_adaptive_model.push_back(ws_adaptive_model(i)(1));
		w_z_adaptive_model.push_back(ws_adaptive_model(i)(2));
	}

	plt::plot(ts, w_x);
	plt::plot(ts, w_x_adaptive_model, "r--");
	plt::title("w_x");
	plt::show();

	plt::plot(ts, w_y);
	plt::plot(ts, w_y_adaptive_model, "r--");
	plt::title("w_y");
	plt::show();

	plt::plot(ts, w_z);
	plt::plot(ts, w_z_adaptive_model, "r--");
	plt::title("w_z");
	plt::show();
}


void plot_attitude(
		Eigen::VectorX<Eigen::Quaterniond> qs,
		Eigen::VectorX<Eigen::Vector3d> refs,
		Eigen::VectorX<Eigen::Quaterniond> cmds,
		std::vector<double> ts
		)
{
	std::vector<double> roll, pitch, yaw;
	std::vector<double> ref_roll, ref_pitch, ref_yaw;
	std::vector<double> cmd_roll, cmd_pitch, cmd_yaw;

	// Convert quaternions to euler angles
	for (int i = 0; i < qs.size(); ++i)
	{
		Eigen::Vector3d euler = QuatToEuler(qs(i));
		roll.push_back(euler(0));
		pitch.push_back(euler(1));
		yaw.push_back(euler(2));

		Eigen::Vector3d cmd_euler = QuatToEuler(cmds(i));
		cmd_roll.push_back(cmd_euler(0));
		cmd_pitch.push_back(cmd_euler(1));
		cmd_yaw.push_back(cmd_euler(2));

		ref_roll.push_back(refs(i)(0));
		ref_pitch.push_back(refs(i)(1));
		ref_yaw.push_back(refs(i)(2));
	}

	plt::plot(ts, roll);
	plt::plot(ts, ref_roll);
	plt::plot(ts, cmd_roll, "b--");
	plt::title("Roll");
	plt::show();

	plt::plot(ts, pitch);
	plt::plot(ts, ref_pitch);
	plt::plot(ts, cmd_pitch, "b--");
	plt::title("Pitch");
	plt::show();

	plt::plot(ts, yaw);
	plt::plot(ts, ref_yaw);
	plt::plot(ts, cmd_yaw, "b--");
	plt::title("Yaw");
	plt::ylim(-0.4, 0.4);
	plt::show();

}

void plot_cmd(
		Eigen::VectorX<Eigen::Quaterniond> cmds,
		Eigen::VectorX<Eigen::Vector3d> refs,
		std::vector<double> ts
		)
{
	std::vector<double> roll, pitch, yaw;
	std::vector<double> ref_roll, ref_pitch, ref_yaw;

	// Convert quaternions to euler angles
	for (int i = 0; i < cmds.size(); ++i)
	{
		Eigen::Vector3d euler = QuatToEuler(cmds(i));
		roll.push_back(euler(0));
		pitch.push_back(euler(1));
		yaw.push_back(euler(2));

		ref_roll.push_back(refs(i)(0));
		ref_pitch.push_back(refs(i)(1));
		ref_yaw.push_back(refs(i)(2));
	}

	plt::plot(ts, roll);
	plt::plot(ts, ref_roll);
	plt::title("Roll command");
	plt::show();

	plt::plot(ts, pitch);
	plt::plot(ts, ref_pitch);
	plt::title("Pitch command");
	plt::show();

	plt::plot(ts, yaw);
	plt::plot(ts, ref_yaw);
	plt::title("Yaw command");
	plt::show();
}

