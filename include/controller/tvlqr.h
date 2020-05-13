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
			ControllerConstructor(double m, Eigen::Matrix3d inertia);

		private:
			const drake::symbolic::Variable x_;
			const drake::symbolic::Variable y_;
			const drake::symbolic::Variable z_;

			const drake::symbolic::Variable phi_; // roll
			const drake::symbolic::Variable th_;  // pitch
			const drake::symbolic::Variable psi_; // yaw

			const drake::symbolic::Variable xDt_;
			const drake::symbolic::Variable yDt_;
			const drake::symbolic::Variable zDt_;

			const drake::symbolic::Variable phiDt_;
			const drake::symbolic::Variable thDt_;
			const drake::symbolic::Variable psiDt_;

			const drake::symbolic::Variable u_th_;
			const drake::symbolic::Variable u_x_;
			const drake::symbolic::Variable u_y_;
			const drake::symbolic::Variable u_z_;

			const double g_;
			const double m_;
			const Eigen::Matrix3d inertia_;

			Eigen::VectorX<drake::symbolic::Variable> state_;
			Eigen::VectorX<drake::symbolic::Variable> input_;
			Eigen::VectorX<drake::symbolic::Expression> stateDt_;
			Eigen::MatrixX<drake::symbolic::Expression> A_;
			Eigen::MatrixX<drake::symbolic::Expression> B_;

			Eigen::Vector3<drake::symbolic::Expression> get_rDDt();
			Eigen::Vector3<drake::symbolic::Expression> get_wDDt();
	};

} // namespace controller
