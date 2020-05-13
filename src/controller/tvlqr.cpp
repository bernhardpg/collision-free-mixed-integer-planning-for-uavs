#include "controller/tvlqr.h"

using drake::symbolic::cos;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace controller
{

	DrakeControllerTVLQR::DrakeControllerTVLQR(
			const Eigen::VectorX<Eigen::MatrixXd> As,
			const Eigen::VectorX<Eigen::MatrixXd> Bs,
			const Eigen::VectorX<Eigen::MatrixXd> Ss,
			const Eigen::MatrixXd Q,
			const Eigen::MatrixXd R,
			const double dt
			)
		: drake::systems::VectorSystem<double>(12,4),
		As_(As),
		Bs_(Bs),
		Ss_(Ss),
		Q_(Q),
		R_(R),
		dt_(dt)
	{}

	void DrakeControllerTVLQR::DoCalcVectorOutput	(
			const drake::systems::Context<double>& context,
			const Eigen::VectorBlock<const drake::VectorX<double>>& input,
			const Eigen::VectorBlock<const drake::VectorX<double>>& state,
			Eigen::VectorBlock<drake::VectorX<double>>* output
	) const
	{
		double t = context.get_time();
		*output = Eigen::VectorXd::Zero(4);

		int i = t / dt_;
		std::cout << i << std::endl;
	
		/*auto B = Bs_(i);
		auto S = Ss_(i);

		Eigen::MatrixXd K = - R_.inverse() * B.transpose() * S;*/

	}

	/*Eigen::Vector3d pos_ = Eigen::VectorXd(input.segment(0,3));
	Eigen::Vector3d pos_dot_ = Eigen::VectorXd(input.segment(3,3));
	Eigen::Vector3d att_euler = Eigen::VectorXd(input.segment(6,3));
	Eigen::Vector3d w_ = Eigen::VectorXd(input.segment(9,3));

	std::cout << att_euler << std::endl << std::endlm*/

	/*
	pos_ = input.template block<3,1>(0);
	pos_dot_ = input.template block<3,1>(3);
	auto att_euler = input.template block<3,1>(6);
	w_ = input.template block<3,1>(9);
	*/
	

	ControllerTVLQR::ControllerTVLQR(double m, Eigen::Matrix3d inertia)
		:
			g_(9.81),
			m_(m),
			inertia_(inertia),

			x_(Variable("x")),
			y_(Variable("y")),
			z_(Variable("z")),

			xDt_(Variable("xDt")),
			yDt_(Variable("yDt")),
			zDt_(Variable("zDt")),

		  phi_(Variable("phi")),
			th_(Variable("th")),
			psi_(Variable("psi")),

		  phiDt_(Variable("phiDt")),
			thDt_(Variable("thDt")),
			psiDt_(Variable("psiDt")),

			u_th_(Variable("u_th")),
			u_x_(Variable("u_x")), 
			u_y_(Variable("u_y")), 
			u_z_(Variable("u_z"))
	{
		// Set state vector
		state_.resize(12);
		state_ << x_, y_, z_,
							phi_, th_, psi_,
							xDt_, yDt_, zDt_,
							phiDt_, thDt_, psiDt_;

		input_.resize(4);
		input_ << u_th_,
							u_x_,
							u_y_,
							u_z_;

		// Calculate derivatives
		Eigen::Vector3<Expression> rDt(xDt_, yDt_, zDt_);			 // Position Dt
		Eigen::Vector3<Expression> wDt(phiDt_, thDt_, psiDt_); // Angular velocity Dt
		Eigen::Vector3<Expression> rDDt = get_rDDt();	
		Eigen::Vector3<Expression> wDDt = get_wDDt();	

		stateDt_.resize(12);
		stateDt_ << rDt,
								wDt,
								rDDt,
								wDDt;

		// Calculate linear system
		A_ = drake::symbolic::Jacobian(stateDt_, state_);
		B_ = drake::symbolic::Jacobian(stateDt_, input_);
	}

	Eigen::MatrixXd ControllerTVLQR::eval_A(drake::symbolic::Environment curr_state)
	{
		return drake::symbolic::Evaluate(A_, curr_state);
	}

	Eigen::MatrixXd ControllerTVLQR::eval_B(drake::symbolic::Environment curr_state)
	{
		return drake::symbolic::Evaluate(B_, curr_state);
	}

	std::unique_ptr<DrakeControllerTVLQR> ControllerTVLQR::construct_drake_controller(
			double start_time, double end_time, double dt
			)
	{
		drake::symbolic::Environment curr_state{{x_,     0},
																						{y_,     0},
																						{z_,     0},
																						{phi_,   0},
																						{th_,    0},
																						{psi_,   0},
																						{xDt_,   0},
																						{yDt_,   0},
																						{zDt_,   0},
																						{phiDt_, 0},
																						{thDt_,  0},
																						{psiDt_, 0},
																						{u_th_,  0},
																						{u_x_,   0},
																						{u_y_,   0},
																						{u_z_,   0}};

		dt_ = dt;
		start_time_ = start_time;
		end_time_ = end_time;
		N_ = (end_time_ - start_time_) / dt_;

		Eigen::VectorX<Eigen::MatrixXd> As(N_);
		Eigen::VectorX<Eigen::MatrixXd> Bs(N_);
		Eigen::VectorX<Eigen::MatrixXd> Ss(N_);

		Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
		Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);

		Eigen::MatrixXd Q_f = Eigen::MatrixXd::Identity(12, 12);
		Ss(N_ - 1) = Q_f;
		As(N_ - 1) = eval_A(curr_state);
		Bs(N_ - 1) = eval_B(curr_state);
		
		// Note: Integrating backwards
		for (int i = N_ - 1; i > 0; --i)
		{
			// TODO update state from trajectory

			// Differential Ricatti Equation
			auto neg_SDt =
				Ss(i) * As(i) + As(i).transpose() * Ss(i)
				- Ss(i).transpose() * Bs(i) * R.inverse() * Bs(i).transpose() * Ss(i)
				+ Q;

			// Forward Euler to integrate S backwards
			Ss(i - 1) = Ss(i) + dt_ * neg_SDt;

			// Evaluate linearization for next time step
			As(i - 1) = eval_A(curr_state);
			Bs(i - 1) = eval_B(curr_state);
		}

		return std::make_unique<DrakeControllerTVLQR>(
				As, Bs, Ss, Q, R, dt_
				);
	}

	Eigen::Vector3<drake::symbolic::Expression> ControllerTVLQR::get_rDDt()
	{
		// Calculate rotation matrices
		Eigen::Matrix3<Expression> R_z;
		Eigen::Matrix3<Expression> R_y;
		Eigen::Matrix3<Expression> R_x;

		R_z << cos(psi_), -sin(psi_), 0,
					 sin(psi_),  cos(psi_), 0,
			        	  0,         0, 1;

		R_y << cos(th_), 0, sin(th_),
					       0, 1,       0,
					-sin(th_), 0, cos(th_);

		R_x << 1,        0,         0,
			     0, cos(phi_), -sin(phi_),
					 0, sin(phi_),  cos(phi_);

		// R_NB is rotation from Body frame B to Newtonian frame N
		Eigen::Matrix3<drake::symbolic::Expression> R_NB = R_z * R_y * R_x;
		
		// Positional dynamics
		Eigen::Vector3<drake::symbolic::Expression> rDDt =
			Eigen::Vector3d(0,0,-g_) + (u_th_ / m_) * R_NB * Eigen::Vector3d(0,0,1);
		
		return rDDt;
	} 

	Eigen::Vector3<drake::symbolic::Expression> ControllerTVLQR::get_wDDt()
	{
		Eigen::Vector3<Expression> rpy(phiDt_, thDt_, psiDt_);
		Eigen::Vector3<Expression> tau_c(u_x_, u_y_, u_z_);
		Eigen::Vector3<Expression> wDDt =
			inertia_.inverse() * (rpy.cross(inertia_ * rpy)) + tau_c;

		return wDDt;
	}
} // namespace controller
