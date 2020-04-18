#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <iostream>
#include <Eigen/Dense>
#include "iris/iris.h"

namespace trajopt
{
	typedef Eigen::MatrixX<drake::symbolic::Expression> coeff_matrix_t;

	class MISOSProblem
	{
		public:
			MISOSProblem(
					const int num_traj_segments,
					const int num_vars,
					const int degree,
					const int continuity_degree
					);

		private:
			const int num_vars_;
			const int degree_;
			const int continuity_degree_;
			const int num_traj_segments_;

			drake::symbolic::Variable t_;
			std::vector<coeff_matrix_t> coeffs_;
			std::vector<std::vector<coeff_matrix_t>> coeffs_d;
			drake::solvers::MathematicalProgramResult result_;
			Eigen::VectorX<drake::symbolic::Expression> m_; // Vector of monomial basis functions


			drake::solvers::MathematicalProgram prog_;
	};
	int factorial(int n);
	}
