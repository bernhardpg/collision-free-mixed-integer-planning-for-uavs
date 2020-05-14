#pragma once

#include <drake/common/symbolic.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/mosek_solver.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <iostream>
#include <Eigen/Core>


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
					const int continuity_degree,
					Eigen::VectorX<double> init_cond,
					Eigen::VectorX<double> final_cond
					);

			void add_region_constraint(
					int region_number, int segment_number
					)
			{
				add_region_constraint(region_number, segment_number, false);
			};
			void add_region_constraint(
					int region_number, int segment_number, bool always_enforce
					);
			void add_safe_region_assignments(
					Eigen::MatrixX<int>
					);
			void add_convex_regions(
					std::vector<Eigen::MatrixX<double>> As,
					std::vector<Eigen::VectorX<double>> bs
					);
			void create_region_binary_variables();
			void generate();
			Eigen::MatrixX<int> get_region_assignments();
			Eigen::VectorX<double> eval(double t);
		private:
			const int num_vars_;
			const int degree_;
			const int continuity_degree_;
			const int num_traj_segments_;
			int num_regions_;
			const double vehicle_radius_;
			const double big_M_ = 10; // TODO just set arbitrary: set better?

			std::vector<Eigen::MatrixX<double>> regions_A_;
			std::vector<Eigen::VectorX<double>> regions_b_;

			Eigen::VectorX<drake::symbolic::Expression> m_;
			// Vector of monomial basis functions

			drake::symbolic::Variable t_;
			std::vector<coeff_matrix_t> coeffs_;
			Eigen::MatrixX<drake::symbolic::Expression> H_;
			std::vector<std::vector<coeff_matrix_t>> coeffs_d_;
			drake::solvers::MathematicalProgram prog_;

			drake::solvers::MathematicalProgramResult result_;
			Eigen::MatrixX<drake::symbolic::Polynomial> polynomials_;

			Eigen::VectorX<drake::symbolic::Expression> get_coefficients_in_t(drake::symbolic::Polynomial p);

			void generate_polynomials();
	};
	int factorial(int n);
	}
