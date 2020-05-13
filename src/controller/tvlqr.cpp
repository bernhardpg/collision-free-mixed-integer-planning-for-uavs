#include "controller/tvlqr.h"

using drake::symbolic::cos;
using drake::symbolic::Expression;
using drake::symbolic::Variable;

namespace controller
{
	ControllerConstructor::ControllerConstructor()
		:
			x_(Variable("x")),
			y_(Variable("y")),
			z_(Variable("z")),
		  phi_(Variable("phi")),
			th_(Variable("th")),
			psi_(Variable("psi")),
			u_th_(Variable("u_th")),
			u_x_(Variable("u_x")), 
			u_y_(Variable("u_y")), 
			u_z_(Variable("u_z"))
	{
		inertia_ << 0.07, 0, 0,
							 0, 0.08, 0,
							 0, 0, 0.12; // From .urdf file

		Eigen::VectorX<Variable> state_(6);
		state_ << x_, y_, z_,
							phi_, th_, psi_;

		auto rDDt = get_rDDt();	

		auto j = drake::symbolic::Jacobian(rDDt, state_);

		std::cout << j.rows() << " x " << j.cols() << std::endl;
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

		// R_NB is rotation from Body frame to Newtonian frame
		Eigen::Matrix3<drake::symbolic::Expression> R_NB = R_z * R_y * R_x;
		
		// Positional dynamics
		Eigen::Vector3<drake::symbolic::Expression> rDDt =
			Eigen::Vector3d(0,0,-g_) + (u_th_ / m_) * R_NB * Eigen::Vector3d(0,0,1);
		
		return rDDt;
	} 
} // namespace controller
