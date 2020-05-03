#include "simulate/controller_drake.h"

void ControllerDrake::DoCalcVectorOutput(
				const drake::systems::Context<double>& context,
				const Eigen::VectorBlock<const drake::VectorX<double>>& input,
				const Eigen::VectorBlock<const drake::VectorX<double>>& unused,
				Eigen::VectorBlock<drake::VectorX<double>>* output
		) const
{ 
	*output = Eigen::VectorXd::Zero(4);

	Eigen::Vector3d pos_ = Eigen::VectorXd(input.segment(0,3));
	Eigen::Vector3d pos_dot_ = Eigen::VectorXd(input.segment(3,3));
	Eigen::Vector3d att_euler = Eigen::VectorXd(input.segment(6,3));
	Eigen::Vector3d w_ = Eigen::VectorXd(input.segment(9,3));

	std::cout << att_euler << std::endl << std::endl;

	/*
	pos_ = input.template block<3,1>(0);
	pos_dot_ = input.template block<3,1>(3);
	auto att_euler = input.template block<3,1>(6);
	w_ = input.template block<3,1>(9);
	*/
}
