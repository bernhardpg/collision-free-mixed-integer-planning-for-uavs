#pragma once

#include <memory>
#include <Eigen/Core>
#include <drake/systems/framework/vector_system.h>
#include <drake/common/symbolic.h>
#include <drake/common/symbolic_expression.h>

namespace controller
{
	class DrakeControllerTVLQR : public drake::systems::VectorSystem<double>
	{
		public:
			DrakeControllerTVLQR(
					const Eigen::VectorX<Eigen::MatrixXd> As,
					const Eigen::VectorX<Eigen::MatrixXd> Bs,
					const Eigen::VectorX<Eigen::MatrixXd> Ss,
					const Eigen::MatrixXd Q,
					const Eigen::MatrixXd R,
					const double dt
					);

		private:
			void DoCalcVectorOutput	(
					const drake::systems::Context<double>& context,
					const Eigen::VectorBlock<const drake::VectorX<double>>& input,
					const Eigen::VectorBlock<const drake::VectorX<double>>& state,
					Eigen::VectorBlock<drake::VectorX<double>>* output
			) const override;

			const Eigen::VectorX<Eigen::MatrixXd> As_;
			const Eigen::VectorX<Eigen::MatrixXd> Bs_;
			const Eigen::VectorX<Eigen::MatrixXd> Ss_;
			const Eigen::MatrixXd Q_;
			const Eigen::MatrixXd R_;
			double dt_;
	};


	class ControllerTVLQR
	{
		public:
			ControllerTVLQR(double m, Eigen::Matrix3d inertia);
			std::unique_ptr<DrakeControllerTVLQR> construct_drake_controller(
					double start_time, double end_time, double dt
					);

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

			int N_;
			double start_time_;
			double end_time_;
			double dt_;
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

			Eigen::MatrixXd eval_A(drake::symbolic::Environment curr_state);
			Eigen::MatrixXd eval_B(drake::symbolic::Environment curr_state);
	};

} // namespace controller
