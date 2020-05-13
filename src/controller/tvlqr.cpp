#include "controller/tvlqr.h"

using drake::symbolic::cos;

namespace controller
{
	void construct_controller_tvlqr()
	{
		drake::symbolic::Variable x("x");
		drake::symbolic::Variable y("x");
		drake::symbolic::Variable z("x");

		drake::symbolic::Variable phi("phi");
		drake::symbolic::Variable th("theta");
		drake::symbolic::Variable psi("psi");

		drake::symbolic::Variable u1("u1");

		double g = 9.81;
		double m = 2.856;

		Eigen::Matrix3d inertia;
		inertia << 0.07, 0, 0,
							 0, 0.08, 0,
							 0, 0, 0.12; // From .urdf file

		// Calculate rotation matrices
		Eigen::Matrix3<drake::symbolic::Expression> R_z;
		Eigen::Matrix3<drake::symbolic::Expression> R_y;
		Eigen::Matrix3<drake::symbolic::Expression> R_x;

		R_z << cos(psi), -sin(psi), 0,
					 sin(psi),  cos(psi), 0,
			        	  0,         0, 1;

		R_y << cos(th), 0, sin(th),
					       0, 1,       0,
					-sin(th), 0, cos(th);

		R_x << 1,        0,         0,
			     0, cos(phi), -sin(phi),
					 0, sin(phi),  cos(phi);

		// R_NB is rotation from Body frame to Newtonian frame
		Eigen::Matrix3<drake::symbolic::Expression> R_NB = R_z * R_y * R_x;
		

		// Positional dynamics
		Eigen::Vector3<drake::symbolic::Expression> rDDt = Eigen::Vector3d(0,0,-g) + (u1 / m) * R_NB * Eigen::Vector3d(0,0,1);
		
		
		std::cout << rDDt << std::endl;
	} 
} // namespace controller
