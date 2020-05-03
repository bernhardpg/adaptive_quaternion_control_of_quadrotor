#pragma once

#include <iostream>
#include <drake/systems/framework/vector_system.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller/attitude_controller.h"
#include "controller/position_controller.h"

class ControllerDrake: public drake::systems::VectorSystem<double>
{
	public:
		ControllerDrake() : drake::systems::VectorSystem<double>(12,4) {};

	private:
	//	Eigen::Quaterniond q_;
	//	Eigen::Vector3d pos_;
	//	Eigen::Vector3d w_;
	//	Eigen::Vector3d pos_dot_;

		controller::AdaptiveController attitude_controller_;
		controller::PositionController position_controller_;

		void DoCalcVectorOutput	(
				const drake::systems::Context<double>& context,
				const Eigen::VectorBlock<const drake::VectorX<double>>& input,
				const Eigen::VectorBlock<const drake::VectorX<double>>& state,
				Eigen::VectorBlock<drake::VectorX<double>>* output
		) const override;
};
