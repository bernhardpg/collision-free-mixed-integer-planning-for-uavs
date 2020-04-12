#pragma once

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "plot/matplotlibcpp.h"

namespace trajopt
{

	class PPTrajectory
	{
		public:
			PPTrajectory(
					Eigen::VectorXd sample_times,
					const int num_vars,
					const int degree,
					const int continuity_degree
					);

			void generate();
			Eigen::VectorX<drake::symbolic::Expression> eval_symbolic(
					const double t
					);
			Eigen::VectorX<drake::symbolic::Expression> eval_symbolic(
					const double t, const int derivative_order
					);
			Eigen::VectorXd eval(const double t);
			Eigen::VectorXd eval(const double t, const int derivative_order);

			void add_constraint(
					const double t, const int derivative_order, Eigen::VectorXd lb
					);
			void add_constraint(
					const double t, const int derivative_order, Eigen::VectorXd lb, Eigen::VectorXd ub
					);

		private:
			Eigen::VectorXd sample_times_;
			const int num_vars_;
			const int degree_;
			const int continuity_degree_;
			drake::solvers::MathematicalProgramResult result_;
			std::vector<drake::solvers::MatrixXDecisionVariable> coeffs_;

			drake::solvers::MathematicalProgram prog_;
	};

	void program_init();
	int factorial(int n);

}
