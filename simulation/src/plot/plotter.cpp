#include "plot/plotter.h"

void plot_adaptive_model(
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

/*

void plot_traj(
		trajopt::MISOSProblem *traj, int num_traj_segments, Eigen::VectorX<double> init_pos, Eigen::VectorX<double> final_pos
		)
{
	int tf = num_traj_segments;

	// Plot trajectory
	const double delta_t = 0.01;
	int N = (int)(tf / delta_t);

	std::vector<double> x;
	std::vector<double> y;

	for (int i = 0; i < N; ++i)
	{
		double t = 0.0 + delta_t * i;
		x.push_back(traj->eval(t)(0));
		y.push_back(traj->eval(t)(1));
	}

	// Plot segment start and ends
	std::vector<double> sample_times_x;
	std::vector<double> sample_times_y;
	for (int t = 0; t <= num_traj_segments; ++t)
	{
			sample_times_x.push_back(traj->eval(t)(0));
			sample_times_y.push_back(traj->eval(t)(1));
	}

	typedef	std::unordered_map<std::string, std::string> string_map;

	// Plot init and final pos
	std::vector<double> init_x;
	std::vector<double> init_y;
	init_x.push_back(init_pos(0));
	init_x.push_back(final_pos(0));
	init_y.push_back(init_pos(1));
	init_y.push_back(final_pos(1));

	plt::scatter(init_x, init_y, 50, string_map({{"color","blue"}}));
	plt::scatter(sample_times_x, sample_times_y, 20, string_map({{"color","red"}}));
	
	plt::plot(x, y);

	plt::show();
}

*/
