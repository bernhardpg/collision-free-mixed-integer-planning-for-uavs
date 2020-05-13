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

	void construct_controller_tvlqr();

} // namespace controller
