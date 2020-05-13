#include "controller/tvlqr.h"

using drake::symbolic::cos;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace controller
{
	ControllerConstructor::ControllerConstructor(double m, Eigen::Matrix3d inertia)
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

		drake::symbolic::Environment at_origin {{x_,     0},
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

		std::cout << "A: " << A_.rows() << " x " << A_.cols() << std::endl;
		std::cout << drake::symbolic::Evaluate(A_, at_origin) << std::endl;

		std::cout << "B: " << B_.rows() << " x " << B_.cols() << std::endl;
		std::cout << drake::symbolic::Evaluate(B_, at_origin) << std::endl;
	}

	Eigen::Vector3<drake::symbolic::Expression> ControllerConstructor::get_rDDt()
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

	Eigen::Vector3<drake::symbolic::Expression> ControllerConstructor::get_wDDt()
	{
		Eigen::Vector3<Expression> rpy(phiDt_, thDt_, psiDt_);
		Eigen::Vector3<Expression> tau_c(u_x_, u_y_, u_z_);
		Eigen::Vector3<Expression> wDDt =
			inertia_.inverse() * (rpy.cross(inertia_ * rpy)) + tau_c;

		return wDDt;
	}
} // namespace controller
