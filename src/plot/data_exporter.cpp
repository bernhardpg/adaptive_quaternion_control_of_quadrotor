#include "plot/data_exporter.h"

void export_position(
		Eigen::VectorX<Eigen::Vector3d> poss,
		Eigen::VectorX<Eigen::Vector3d> ref_poss,
		std::vector<double> ts
		)
{
	std::ofstream pos_file;
	pos_file.open ("output_data/position.csv");
	pos_file << "t,pos_x,pos_x_ref,pos_y,pos_y_ref,pos_z,pos_z_ref\n";

	for (int i = 0; i < ts.size(); ++i)
	{
		pos_file << ts[i] << ",";
		pos_file << poss(i)(0) << "," << ref_poss(i)(0) << ",";
		pos_file << poss(i)(1) << "," << ref_poss(i)(1) << ",";
		pos_file << poss(i)(2) << "," << ref_poss(i)(2);
		pos_file << "\n";
	}
	pos_file.close();
}

void export_attitude(
		Eigen::VectorX<Eigen::Quaterniond> qs,
		Eigen::VectorX<Eigen::Vector3d> refs,
		Eigen::VectorX<Eigen::Quaterniond> cmds,
		std::vector<double> ts
		)
{
	std::ofstream att_file;
	att_file.open ("output_data/attitude.csv");
	att_file << "t,"
					 << "roll,ref_roll,cmd_roll,"
					 << "pitch,ref_pitch,cmd_pitch";

	for (int i = 0; i < qs.size(); ++i)
	{
		// Convert quaternions to euler angles
		Eigen::Vector3d euler = QuatToEuler(qs(i));
		Eigen::Vector3d cmd_euler = QuatToEuler(cmds(i));

		att_file << ts[i] << ",";
		att_file << euler(0) << "," << refs(i)(0) << "," << cmd_euler(0) << ",";
		att_file << euler(1) << "," << refs(i)(1) << "," << cmd_euler(1) << ",";
		att_file << euler(2) << "," << refs(i)(2) << "," << cmd_euler(2);
		att_file << "\n";
	}

	att_file.close();
}

void export_input_torques(
		Eigen::VectorX<Eigen::Vector3d> input_torques,
		Eigen::Vector3d limits,
		std::vector<double> ts
		)
{
	std::ofstream input_file;
	input_file.open ("output_data/inputs.csv");
	input_file << "t,"
						 << "tau_x,limit_x,tau_y,limit_y";

	for (int i = 0; i < input_torques.size(); ++i)
	{
		input_file << ts[i] << ",";
		input_file << input_torques(i)(0) << "," << limits(0) << ",";
		input_file << input_torques(i)(1) << "," << limits(1);
		input_file << "\n";
	}
	input_file.close();

}


void export_adaptive_params(
		Eigen::VectorX<Eigen::Matrix3d> Theta_hat,
		Eigen::VectorX<Eigen::Matrix3d> Lambda_hat,
		Eigen::VectorX<Eigen::Vector3d> tau_dist_hat,
		std::vector<double> ts
		)
{
	std::ofstream adaptive_params_file;
	adaptive_params_file.open ("output_data/adaptive_params.csv");
	adaptive_params_file << "t,"
						 << "tau_dist_x,tau_dist_y,tau_dist_z";

	for (int i = 0; i < ts.size(); ++i)
	{
		adaptive_params_file << ts[i] << ",";
		adaptive_params_file << tau_dist_hat(i)(0) << ",";
		adaptive_params_file << tau_dist_hat(i)(1) << ",";
		adaptive_params_file << tau_dist_hat(i)(2) << "\n";
	}
	adaptive_params_file.close();
}


/*

void plot_input_torques(
		Eigen::VectorX<Eigen::Vector3d> input_torques,
		Eigen::Vector3d limits,
		std::vector<double> ts
		)
{
	std::vector<double> tau_x, tau_y, tau_z;
	std::vector<double> limit_x, limit_y, limit_z;
	std::vector<double> neg_limit_x, neg_limit_y, neg_limit_z;
	for (int i = 0; i < input_torques.size(); ++i)
	{
		tau_x.push_back(input_torques(i)(0));
		tau_y.push_back(input_torques(i)(1));
		tau_z.push_back(input_torques(i)(2));

		limit_x.push_back(limits(0));
		neg_limit_x.push_back(-limits(0));
		limit_y.push_back(limits(1));
		neg_limit_y.push_back(-limits(1));
	}

	plt::plot(ts, tau_x);
	plt::plot(ts, limit_x, "b--");
	plt::plot(ts, neg_limit_x, "b--");
	plt::title("tau_x");
	plt::show();

	plt::plot(ts, tau_y);
	plt::plot(ts, limit_y, "b--");
	plt::plot(ts, neg_limit_y, "b--");
	plt::title("tau_y");
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

	plt::plot(ts, baseline_tau_x, "b--");
	plt::plot(ts, adaptive_tau_x);
	plt::title("tau_x");
	plt::show();

	plt::plot(ts, baseline_tau_y, "b--");
	plt::plot(ts, adaptive_tau_y);
	plt::title("tau_y");
	plt::show();

	plt::plot(ts, baseline_tau_y, "b--");
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

*/
