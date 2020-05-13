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
			const drake::symbolic::Variable x_;
			const drake::symbolic::Variable y_;
			const drake::symbolic::Variable z_;

			const drake::symbolic::Variable phi_;
			const drake::symbolic::Variable th_;
			const drake::symbolic::Variable psi_;

			const drake::symbolic::Variable u_th_;
			const drake::symbolic::Variable u_x_;
			const drake::symbolic::Variable u_y_;
			const drake::symbolic::Variable u_z_;

			const double g_ = 9.81;
			const double m_ = 2.856;
			Eigen::Matrix3d inertia_; // TODO make const

			Eigen::Vector3<drake::symbolic::Expression> get_rDDt();
	};

} // namespace controller
