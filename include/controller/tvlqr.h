#pragma once

#include <memory>
#include <cmath>
#include <Eigen/Core>
#include <drake/systems/framework/vector_system.h>
#include <drake/common/symbolic.h>
#include <drake/common/symbolic_expression.h>
#include <drake/math/continuous_algebraic_riccati_equation.h>

#include <drake/math/rotation_matrix.h>
#include <drake/math/roll_pitch_yaw.h>

#include "trajopt/MISOSProblem.h"

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
					double hover_thrust,
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
			Eigen::MatrixXd S_inf_;
			const Eigen::VectorXd feed_forward_;
			const double N_;
			const double dt_;
	};


	class ControllerTVLQR
	{
		public:
			ControllerTVLQR(
					double m,
					double arm_length,
					Eigen::Matrix3d inertia,
					double k_f,
					double k_m
					);

			std::unique_ptr<DrakeControllerTVLQR> construct_drake_controller(
					double start_time,
					double end_time,
					double dt,
					trajopt::MISOSProblem* traj
					// TODO more elegant way of passing this?
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

			const drake::symbolic::Variable u0_; // Propeller force 0
			const drake::symbolic::Variable u1_; // Propeller force 1
			const drake::symbolic::Variable u2_; // Propeller force 2
			const drake::symbolic::Variable u3_; // Propeller force 3

			int N_;
			double start_time_;
			double end_time_;
			double dt_;
			const double g_;
			const double m_;
			const double arm_length_;
			const double k_f_;
			const double k_m_;
			const Eigen::Matrix3d inertia_;

			Eigen::Matrix4d forces_to_inputs_matrix_;
			Eigen::Matrix4d inputs_to_forces_matrix_;

			Eigen::VectorX<drake::symbolic::Variable> state_;
			Eigen::VectorX<drake::symbolic::Variable> input_;
			Eigen::VectorX<drake::symbolic::Expression> stateDt_;
			Eigen::MatrixX<drake::symbolic::Expression> A_;
			Eigen::MatrixX<drake::symbolic::Expression> B_;

			Eigen::Vector3<drake::symbolic::Expression> get_rDDt();
			Eigen::Vector3<drake::symbolic::Expression> get_wDt();

			// *****
			// Differential flatness
			// *****
			// Helper functions to calculate full state trajectory from flat outputs
			double get_u_thrust_from_traj(Eigen::Vector3d a);
			Eigen::Vector3d get_rpy_from_traj(
					Eigen::Vector3d r,
					Eigen::Vector3d a,
					double yaw
					);
			Eigen::Vector3d get_w_from_traj(
					Eigen::Vector3d rpy,
					Eigen::Vector3d a_Dt,
					double yaw_Dt,
					double u_thrust
					);
			Eigen::Vector3d get_wDt_from_traj(
					Eigen::Vector3d rpy,
					Eigen::Vector3d w,
					Eigen::Vector3d a_Dt,
					Eigen::Vector3d a_DDt,
					double u_thrust
					);
			Eigen::Vector3d get_u_torques_from_traj(
					Eigen::Vector3d w,
					Eigen::Vector3d w_Dt
					);


			Eigen::MatrixXd eval_A(drake::symbolic::Environment curr_state);
			Eigen::MatrixXd eval_B(drake::symbolic::Environment curr_state);
	};

} // namespace controller
