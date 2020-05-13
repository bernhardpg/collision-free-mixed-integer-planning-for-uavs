#include "controller/tvlqr.h"

using drake::symbolic::cos;

namespace controller
{
	ControllerConstructor::ControllerConstructor() // TODO add g_ etc here
	{
		inertia_ << 0.07, 0, 0,
							 0, 0.08, 0,
							 0, 0, 0.12; // From .urdf file

		auto rDDt = get_rDDt();	
	}

	Eigen::Vector3<drake::symbolic::Expression> ControllerConstructor::get_rDDt()
	{
		// Calculate rotation matrices
		Eigen::Matrix3<drake::symbolic::Expression> R_z;
		Eigen::Matrix3<drake::symbolic::Expression> R_y;
		Eigen::Matrix3<drake::symbolic::Expression> R_x;

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
