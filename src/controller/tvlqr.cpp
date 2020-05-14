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
			double hover_thrust,
			const double dt
			)
		: drake::systems::VectorSystem<double>(12,4),
		As_(As),
		Bs_(Bs),
		Ss_(Ss),
		Q_(Q),
		R_(R),
		N_(As.size()),
		S_inf_(drake::math::ContinuousAlgebraicRiccatiEquation(As_(0), Bs_(0), Q_, R_)),
		feed_forward_(Eigen::VectorXd::Constant(4, hover_thrust/4)),
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

		Eigen::VectorXd x_d(12);
		x_d << 0, 0, 1.0, 0, 0, 0,
					 0, 0,   0, 0, 0, 0;
	
		if (false)
		{
		  int i = t / dt_;
			auto B = Bs_(i);
			auto S = Ss_(i);

			//std::cout << B << std::endl << std::endl;
			//std::cout << S << std::endl << std::endl;

			//Eigen::MatrixXd K = - R_.inverse() * B.transpose() * S;
			//*output = K * input;
			//std::cout << input << std::endl << std::endl;
		}
		else
		{
			Eigen::MatrixXd K = R_.inverse() * Bs_(0).transpose() * S_inf_;
			*output = - K * (input - x_d) + feed_forward_;
			//std::cout << "error:\n" << input - x_d << std::endl << std::endl;
			//std::cout << "-K*x\n" << *output << std::endl << std::endl;
		}
	}

	ControllerTVLQR::ControllerTVLQR(
					double m, double arm_length, Eigen::Matrix3d inertia, double k_f, double k_m
			)
		:
			g_(9.81),
			m_(m),
			arm_length_(arm_length),
			k_f_(k_f),
			k_m_(k_m),
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

			u0_(Variable("u0")),
			u1_(Variable("u1")), 
			u2_(Variable("u2")), 
			u3_(Variable("u3"))
	{
		// Set state vector
		state_.resize(12);
		state_ << x_, y_, z_,
							phi_, th_, psi_,
							xDt_, yDt_, zDt_,
							phiDt_, thDt_, psiDt_;

		input_.resize(4);
		input_ << u0_,
						  u1_,
						  u2_,
						  u3_;


		// Calculate derivatives
		Eigen::Vector3<Expression> rDt(xDt_, yDt_, zDt_);			 // Position Dt
		Eigen::Vector3<Expression> w(phiDt_, thDt_, psiDt_); // Angular velocity Dt
		Eigen::Vector3<Expression> rDDt = get_rDDt();	
		Eigen::Vector3<Expression> wDt = get_wDt();	

		stateDt_.resize(12);
		stateDt_ << rDt,
								w,
								rDDt,
								wDt;

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

	double ControllerTVLQR::get_u_thrust_from_traj(Eigen::Vector3d a)
	{
		Eigen::Vector3d t(a(0), a(1), a(2) + g_);
		
		return m_ * t.norm();
	}

	Eigen::Vector3d ControllerTVLQR::get_rpy_from_traj(
			Eigen::Vector3d r,
			Eigen::Vector3d a, 
			double yaw
			)
	{
		Eigen::Vector3d t(a(0), a(1), a(2) + g_);
		Eigen::Vector3d z_b = t.normalized();

		Eigen::Vector3d x_c(cos(yaw), sin(yaw), 0);
		Eigen::Vector3d y_b = (z_b.cross(x_c)).normalized();
		Eigen::Vector3d x_b = y_b.cross(z_b);

		drake::math::RotationMatrix<double> R_NB =
			drake::math::RotationMatrix<double>::MakeFromOrthonormalColumns(x_b, y_b, z_b);
		
		drake::math::RollPitchYaw rpy(R_NB);
		return Eigen::Vector3d(rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle());
	}

	Eigen::Vector3d ControllerTVLQR::get_w_from_traj(
			Eigen::Vector3d rpy,
			Eigen::Vector3d a_Dt,
			double yaw_Dt,
			double u_thrust
			)
	{
		Eigen::Matrix3d R_NB = 
			drake::math::RotationMatrix(drake::math::RollPitchYaw(rpy))
			.matrix();
		Eigen::Vector3d x_b = R_NB(Eigen::all, 0);
		Eigen::Vector3d y_b = R_NB(Eigen::all, 1);
		Eigen::Vector3d z_b = R_NB(Eigen::all, 2);

		Eigen::Vector3d h_w = (m_ / u_thrust) * (a_Dt - (z_b.dot(a_Dt) * z_b));

		double p = - h_w.dot(y_b);
		double q = h_w.dot(x_b);
		double r = yaw_Dt * Eigen::Vector3d(0,0,1).dot(z_b);

		return Eigen::Vector3d(p, q, r);
	}

	// Angular acceleration
	Eigen::Vector3d ControllerTVLQR::get_wDt_from_traj(
			Eigen::Vector3d rpy,
			Eigen::Vector3d w,
			Eigen::Vector3d a_Dt,
			Eigen::Vector3d a_DDt,
			double u_thrust
			)
	{
		Eigen::Matrix3d R_NB = 
			drake::math::RotationMatrix(drake::math::RollPitchYaw(rpy))
			.matrix();
		Eigen::Vector3d x_b = R_NB(Eigen::all, 0);
		Eigen::Vector3d y_b = R_NB(Eigen::all, 1);
		Eigen::Vector3d z_b = R_NB(Eigen::all, 2);

		Eigen::Vector3d temp = w.cross(w.cross(z_b));

		Eigen::Vector3d h_alpha = 
			(m_ / u_thrust) * (a_DDt - (z_b.dot(a_DDt)) * z_b)
			+ (-temp + z_b.dot(temp) * z_b)
			- (2 / u_thrust) * z_b.dot(m_ * a_Dt) * w.cross(z_b);

		double p_Dt = - h_alpha.dot(y_b);
		double q_Dt = h_alpha.dot(x_b);

		// TODO implement yaw if needed
		return Eigen::Vector3d(p_Dt, q_Dt, 0);
	}

	std::unique_ptr<DrakeControllerTVLQR> ControllerTVLQR::construct_drake_controller(
			double start_time,
			double end_time,
			double dt,
			trajopt::MISOSProblem* traj_obj
			)
	{
		double hover_thrust = m_ * g_;
 
		drake::symbolic::Environment curr_state{{x_,     0},
																						{y_,     0},
																						{z_,     1},
																						{phi_,   0},
																						{th_,    0},
																						{psi_,   0},
																						{xDt_,   0},
																						{yDt_,   0},
																						{zDt_,   0},
																						{phiDt_, 0},
																						{thDt_,  0},
																						{psiDt_, 0},
																						{u0_,    hover_thrust/4},
																						{u1_,    hover_thrust/4},
																						{u2_,    hover_thrust/4},
																						{u3_,    hover_thrust/4}};

		dt_ = dt;
		start_time_ = start_time;
		end_time_ = end_time;
		N_ = (end_time_ - start_time_) / dt_;

		// Calculate full state trajectory from flat outputs
		// Based on paper by Mellinger, Kumar (2011):
		// "Minimum snap trajectory generation and control for quadrotors"
		double t = 0;
		for (int i = 0; i < N_; ++i)
		{
			t += dt;
			if (t < 8) // TODO generalize
			{
				double yaw = 0;
				Eigen::Vector3d r = traj_obj->eval(t);
				Eigen::Vector3d r_Dt = traj_obj->eval_derivative(t,1);
				Eigen::Vector3d a = traj_obj->eval_derivative(t,2);
				Eigen::Vector3d a_Dt = traj_obj->eval_derivative(t,3);
				Eigen::Vector3d a_DDt = traj_obj->eval_derivative(t,4);

				double u_thrust = get_u_thrust_from_traj(a);
				Eigen::Vector3d rpy = get_rpy_from_traj(r, a, yaw);
				Eigen::Vector3d w = get_w_from_traj(rpy, a_Dt, yaw, u_thrust);
				Eigen::Vector3d wDt = get_wDt_from_traj(rpy, w, a_Dt, a_DDt, u_thrust);
				
				// TODO translate inputs to inputs
			}
		}

		Eigen::VectorX<Eigen::MatrixXd> As(N_);
		Eigen::VectorX<Eigen::MatrixXd> Bs(N_);
		Eigen::VectorX<Eigen::MatrixXd> Ss(N_);

		Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12);
		Q(0,0) = 30;
		Q(1,1) = 30;
		Q(2,2) = 30;
		Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);

		Eigen::MatrixXd Q_f = Eigen::MatrixXd::Identity(12, 12);
		Ss(N_ - 1) = Q_f;
		As(N_ - 1) = eval_A(curr_state);
		Bs(N_ - 1) = eval_B(curr_state);

		//std::cout << "a:\n" << as(n_-1) << std::endl << std::endl;
		//std::cout << "b:\n" << bs(n_-1) << std::endl << std::endl;
		
		// Note: Integrating backwards
		for (int i = N_ - 1; i > 0; --i)
		{
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
				As, Bs, Ss, Q, R, hover_thrust, dt_
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
			        	   0,          0, 1;

		R_y << cos(th_), 0, sin(th_),
					        0, 1,        0,
					-sin(th_), 0, cos(th_);

		R_x << 1,        0,         0,
			     0, cos(phi_), -sin(phi_),
					 0, sin(phi_),  cos(phi_);

		// R_NB is rotation from Body frame B to Newtonian frame N
		Eigen::Matrix3<drake::symbolic::Expression> R_NB = R_z * R_y * R_x;
		
		drake::symbolic::Expression u_th = u0_ + u1_ + u2_ + u3_;

		// Positional dynamics
		Eigen::Vector3<drake::symbolic::Expression> rDDt =
			Eigen::Vector3d(0,0,-g_) + (u_th / m_) * R_NB * Eigen::Vector3d(0,0,1);
		
		return rDDt;
	} 

	Eigen::Vector3<drake::symbolic::Expression> ControllerTVLQR::get_wDt()
	{
		Eigen::Vector3<Expression> rpy(phiDt_, thDt_, psiDt_);
		Eigen::MatrixXd forces_to_torques(3,4);

		forces_to_torques << 0, k_f_ * arm_length_, 0, - k_f_ * arm_length_,
												 - k_f_ * arm_length_, 0, k_f_ * arm_length_, 0,
												 k_m_, - k_m_, k_m_, - k_m_;

		Eigen::Vector3<Expression> tau_c = forces_to_torques * input_;


		Eigen::Vector3<Expression> wDt =
			inertia_.inverse() * (rpy.cross(inertia_ * rpy)) + tau_c;

		return wDt;
	}
} // namespace controller
