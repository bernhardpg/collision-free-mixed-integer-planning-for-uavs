#pragma once

#include <drake/systems/framework/vector_system.h>
#include "drake/common/symbolic.h"
#include <drake/common/symbolic_expression.h>
#include <Eigen/Core>

namespace controller
{
	class ControllerTVLQR : public drake::systems::VectorSystem<double>
	{
		public:
			ControllerTVLQR() : drake::systems::VectorSystem<double>(12,4) {};

		private:
			void DoCalcVectorOutput	(
					const drake::systems::Context<double>& context,
					const Eigen::VectorBlock<const drake::VectorX<double>>& input,
					const Eigen::VectorBlock<const drake::VectorX<double>>& state,
					Eigen::VectorBlock<drake::VectorX<double>>* output
			) const override;
	};

	class ControllerConstructor
	{
		public:
			ControllerConstructor();

		private:
			drake::symbolic::Expression x_;
			drake::symbolic::Expression y_;
			drake::symbolic::Expression z_;

			drake::symbolic::Expression phi_;
			drake::symbolic::Expression th_;
			drake::symbolic::Expression psi_;

			drake::symbolic::Expression u_th_;
			drake::symbolic::Expression u_x_;
			drake::symbolic::Expression u_y_;
			drake::symbolic::Expression u_z_;

			double g_ = 9.81;
			double m_ = 2.856;
			Eigen::Matrix3d inertia_;

			Eigen::Vector3<drake::symbolic::Expression> get_rDDt();
	};

} // namespace controller
