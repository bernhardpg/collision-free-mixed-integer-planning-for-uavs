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
			const Eigen::VectorX<Eigen::VectorXd> full_state_d,
			const double hover_thrust,
			const double end_time,
			const double dt
			)
		: drake::systems::VectorSystem<double>(12,4),
		As_(As),
		Bs_(Bs),
		Ss_(Ss),
		Q_(Q),
		R_(R),
		N_(As.size()),
		A_inf_(As(Eigen::last)),
		B_inf_(Bs(Eigen::last)),
		S_inf_(Ss(Eigen::last)),
		full_state_d_(full_state_d),
		hover_thrust_(hover_thrust),
		end_time_(end_time),
		dt_(dt)
	{
		auto pos_d = full_state_d(Eigen::last)(Eigen::seq(0,2));
		x_d_inf_.resize(12);
		x_d_inf_ << pos_d,
								0, 0, 0,
								0, 0, 0,
								0, 0, 0;
		u_inf_ = Eigen::Vector4d(hover_thrust_/4, hover_thrust_/4, hover_thrust_/4, hover_thrust_/4);
	}

	void DrakeControllerTVLQR::DoCalcVectorOutput	(
			const drake::systems::Context<double>& context,
			const Eigen::VectorBlock<const drake::VectorX<double>>& input,
			const Eigen::VectorBlock<const drake::VectorX<double>>& state,
			Eigen::VectorBlock<drake::VectorX<double>>* output
	) const
	{
		double t = context.get_time();

		if (t < end_time_)
		{
		  int i = t / dt_;
			auto B = Bs_(i);
			auto S = Ss_(i);
			Eigen::VectorXd x_d = full_state_d_(i)(Eigen::seq(0,11));
			Eigen::Vector4d u = full_state_d_(i)(Eigen::seq(12,15));

			Eigen::MatrixXd K = R_.inverse() * B.transpose() * S;
			*output = - K * (input - x_d) + u;

			//std::cout << B << std::endl << std::endl;
			//std::cout << S << std::endl << std::endl;
			//std::cout << input << std::endl << std::endl;
		}
		else
		{
			Eigen::MatrixXd K = R_.inverse() * B_inf_.transpose() * S_inf_;
			*output = - K * (input - x_d_inf_) + u_inf_;
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
		// ***********
		// Initialize variables
		// ***********

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


		// ***********
		// Initialize input mapping matrices
		// ***********

		forces_to_inputs_matrix_ << k_f_, k_f_, k_f_, k_f_,
																0, k_f_ * arm_length_, 0, - k_f_ * arm_length_,
															 - k_f_ * arm_length_, 0, k_f_ * arm_length_, 0,
															 k_m_, - k_m_, k_m_, - k_m_;
		inputs_to_forces_matrix_ = forces_to_inputs_matrix_.inverse();

		// *******
		// Obtain linearization
		// *******

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

		// ********
		// Controller weights
		// ********

		Q_ = Eigen::MatrixXd::Identity(12, 12);
		R_ = Eigen::MatrixXd::Identity(4, 4);
		Q_(0,0) = 50;
		Q_(1,1) = 50;
		Q_(2,2) = 50;
	}

	// ********
	// Linearization
	// ********

	Eigen::MatrixXd ControllerTVLQR::eval_A(drake::symbolic::Environment curr_state)
	{
		return drake::symbolic::Evaluate(A_, curr_state);
	}

	Eigen::MatrixXd ControllerTVLQR::eval_B(drake::symbolic::Environment curr_state)
	{
		return drake::symbolic::Evaluate(B_, curr_state);
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

		auto forces_to_torques_matrix = forces_to_inputs_matrix_(Eigen::seq(1,3), Eigen::all);
		Eigen::Vector3<Expression> tau_c = forces_to_torques_matrix * input_;

		Eigen::Vector3<Expression> wDt =
			inertia_.inverse() * (rpy.cross(inertia_ * rpy)) + tau_c;

		return wDt;
	}

	// *********
	// Differential flatness helper functions
	// *********

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

	Eigen::Vector3d ControllerTVLQR::get_u_torques_from_traj(
			Eigen::Vector3d w,
			Eigen::Vector3d w_Dt
			)
	{
		Eigen::Vector3d u_torques = inertia_ * w_Dt	 + w.cross(inertia_ * w);
		return u_torques;
	}

	// Calculate full state trajectory from flat outputs
	// Based on paper by Mellinger, Kumar (2011):
	// "Minimum snap trajectory generation and control for quadrotors"
	Eigen::VectorXd ControllerTVLQR::get_state_traj_from_flat_outputs(trajopt::MISOSProblem *traj_obj, double t)
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
		Eigen::Vector3d w_Dt = get_wDt_from_traj(rpy, w, a_Dt, a_DDt, u_thrust);
		Eigen::Vector3d u_torques = get_u_torques_from_traj(w, w_Dt);

		Eigen::Vector4d u =
			inputs_to_forces_matrix_ 
			* (Eigen::Vector4d() << u_thrust, u_torques).finished();
		
		Eigen::VectorXd full_state_traj(16);
		full_state_traj << r, rpy, r_Dt, w, u;

		return full_state_traj;
	}

	drake::symbolic::Environment ControllerTVLQR::make_drake_env_state(Eigen::VectorXd state)
	{
		drake::symbolic::Environment state_drake_env{
			{x_,     state(0)},
			{y_,     state(1)},
			{z_,     state(2)},
			{phi_,   state(3)},
			{th_,    state(4)},
			{psi_,   state(5)},
			{xDt_,   state(6)},
			{yDt_,   state(7)},
			{zDt_,   state(8)},
			{phiDt_, state(9)},
			{thDt_,  state(10)},
			{psiDt_, state(11)},
			{u0_,    state(12)},
			{u1_,    state(13)},
			{u2_,    state(14)},
			{u3_,    state(15)}
		};
		return state_drake_env;
	}

	// *************
	// Controller constructor
	// *************

	std::unique_ptr<DrakeControllerTVLQR> ControllerTVLQR::construct_drake_controller(
			double dt,
			trajopt::MISOSProblem* traj_obj
			)
	{
		double hover_thrust = m_ * g_;

		dt_ = dt;
		end_time_ = traj_obj->get_end_time();
		N_ = end_time_ / dt_;

		// *******
		// Calculate full state trajectory from flat outpus
		// *******

		Eigen::VectorX<Eigen::VectorXd> full_state_traj(N_);
		double t = 0;
		for (int i = 0; i < N_; ++i)
		{
			t += dt;
			full_state_traj(i) = get_state_traj_from_flat_outputs(traj_obj, t);
		}

		// ******
		// Calculate linearizations A, B
		// ******

		Eigen::VectorX<Eigen::MatrixXd> As(N_);
		Eigen::VectorX<Eigen::MatrixXd> Bs(N_);
	
		t = 0;
		drake::symbolic::Environment curr_state;
		for (int i = 0; i < N_; ++i)
		{
			t += dt;
			curr_state = make_drake_env_state(full_state_traj(i));
			As(i) = eval_A(curr_state);
			Bs(i) = eval_B(curr_state);
		}

		// ********
    // Calculate cost-to-go S
		// ********

		Eigen::VectorX<Eigen::MatrixXd> Ss(N_);
		auto S_inf =
			drake::math::ContinuousAlgebraicRiccatiEquation(As(Eigen::last), Bs(Eigen::last), Q_, R_);
		Ss(N_ - 1) = S_inf;

		// Note: Integrating backwards
		for (int i = N_ - 1; i > 0; --i)
		{
			// Differential Ricatti Equation
			auto neg_SDt =
				Ss(i) * As(i) + As(i).transpose() * Ss(i)
				- Ss(i).transpose() * Bs(i) * R_.inverse() * Bs(i).transpose() * Ss(i)
				+ Q_;

			// Forward Euler to integrate S backwards
			Ss(i - 1) = Ss(i) + dt_ * neg_SDt;
		}

		return std::make_unique<DrakeControllerTVLQR>(
				As, Bs, Ss, Q_, R_, full_state_traj, hover_thrust, end_time_, dt_
				);
	}

} // namespace controller
