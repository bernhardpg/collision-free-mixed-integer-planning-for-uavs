#include "trajectory_optimization.h"
#include <drake/solvers/mosek_solver.h>


namespace trajopt
{

MISOSProblem::MISOSProblem(
		int num_traj_segments,
		int num_vars,
		int degree,
		int continuity_degree,
		Eigen::VectorX<double> init_cond,
		Eigen::VectorX<double> final_cond
		) :
	num_traj_segments_(num_traj_segments),
	num_vars_(num_vars),
	degree_(degree),
	continuity_degree_(continuity_degree),
	vehicle_radius_(0.2)
{
	assert(continuity_degree_ <= degree_);

	t_ = prog_.NewIndeterminates(1, 1, "t")(0,0);
	m_.resize(degree_ + 1); // Monomial basis

	for (int d = 0; d < degree + 1; ++d)
	{
		m_(d) = drake::symbolic::Monomial(t_, d).ToExpression();
	}

	// Add coefficients
	for (int j = 0;	j < num_traj_segments_; ++j)
	{
		// Add d + 1 coefficients for each variable as decision variables
		coeff_matrix_t c = prog_.NewContinuousVariables(
				num_vars, degree + 1, "C");
		coeffs_.push_back(c);

		// Calculate resulting coefficients for each derivative degree
		std::vector<coeff_matrix_t> coeffs_dj;
		for (int der_degree = 1; der_degree < degree_ + 1; ++der_degree)
		{
			// Calculate symbolic polynomials
			auto p = c * m_; // vector of same size as problem
			// Calculate coefficients of derivatives
			auto px = drake::symbolic::Jacobian(p, {t_});

			coeff_matrix_t c_d(num_vars_, degree_ + 1);
			for (int i = 0; i < num_vars_; ++i)
			{
				auto coeff_map = drake::symbolic::Polynomial(px(i), {t_})
					.monomial_to_coefficient_map();

				for (int mm_deg = 0; mm_deg < degree_ + 1 - der_degree; ++mm_deg)
				{
					c_d(i, mm_deg) = coeff_map[drake::symbolic::Monomial(m_(mm_deg))];
				}
			}
			coeffs_dj.push_back(c_d);
			c = c_d;
		}
		coeffs_d_.push_back(coeffs_dj);
	}

	// Get symbolic expressions for monomial values at start and end
	Eigen::VectorX<drake::symbolic::Expression> m_value_t0(degree_ + 1);
	Eigen::VectorX<drake::symbolic::Expression> m_value_t1(degree_ + 1);
	drake::symbolic::Environment t0 {{t_, 0.0}};
	drake::symbolic::Environment t1 {{t_, 1.0}};
	for (int d = 0; d < degree_ + 1; ++d)
	{
		m_value_t0(d) = m_(d).Evaluate(t0);
		m_value_t1(d) = m_(d).Evaluate(t1);
	}

	// Enforce continuity up to required continuity degree
	// For each polynomial segment
	for (int j = 0; j < num_traj_segments_ - 1; ++j)
	{
		auto left_val = coeffs_[j] * m_value_t1;
		auto right_val = coeffs_[j + 1] * m_value_t0;
		prog_.AddLinearConstraint(left_val.array() == right_val.array());

		// For each derivative degree up to required continuity degree
		for (int d = 1; d < continuity_degree_ + 1; ++d)
		{
			auto left_val = coeffs_d_[j][d - 1] * m_value_t1;
			auto right_val = coeffs_d_[j + 1][d - 1] * m_value_t0;
			prog_.AddLinearConstraint(left_val.array() == right_val.array());
		}
	}

	// Add initial and final conditions
	prog_.AddLinearConstraint((coeffs_[0] * m_value_t0).array() == init_cond.array());

	prog_.AddLinearConstraint(
			(coeffs_[num_traj_segments_ - 1] * m_value_t1).array() == final_cond.array());

	// TODO change
	// Force first derivatives at initial and final position to be 0
	for (int d = 1; d <= 1; ++d)
	{
		prog_.AddLinearConstraint(
				(coeffs_d_[0][d - 1] * m_value_t0).array() == Eigen::VectorXd::Zero(num_vars_).array()
			);
		prog_.AddLinearConstraint(
				(coeffs_d_[num_traj_segments_ - 1][d - 1] * m_value_t1).array()
				== Eigen::VectorXd::Zero(num_vars_).array()
			);
	}

	// Add cost to minimize highest derivative order coefficients
	// Reformulate cost to be linear for correct SDP problem formulation
	// (Cost is actually quadratic)
	auto a = prog_.NewContinuousVariables(num_traj_segments, "a");
	for (int j = 0; j < num_traj_segments_; ++j)
	{
		prog_.AddLinearCost(a(j));
		// Remember that highest order derivative only has one coefficient
		auto quadratic_form = coeffs_d_[j][degree_ - 1](Eigen::all, 0)
			.dot(coeffs_d_[j][degree_ - 1](Eigen::all, 0));
		prog_.AddLorentzConeConstraint(a(j), quadratic_form);
	}
}

// Returns coefficients in t
Eigen::VectorX<drake::symbolic::Expression> MISOSProblem::get_coefficients(drake::symbolic::Polynomial p)
{
	Eigen::VectorX<drake::symbolic::Expression> c(degree_ + 1);
	auto coeff_map = p.monomial_to_coefficient_map();

	// mm = monomial
	for (int mm_deg = 0; mm_deg < degree_ + 1; ++mm_deg)
	{
		c(mm_deg) = coeff_map[drake::symbolic::Monomial(m_(mm_deg))];
	}

	return c;
}

void MISOSProblem::add_convex_regions(
		std::vector<Eigen::MatrixX<double>> As, std::vector<Eigen::VectorX<double>> bs
		)
{
	num_regions_ = As.size();
	regions_A_ = As;
	regions_b_ = bs;
}

// Will create a binary decision variable for each combination of region and segment
void MISOSProblem::create_region_binary_variables()
{
	H_ = prog_.NewBinaryVariables(num_regions_, num_traj_segments_, "H");
	// Ensure that each traj segment is strictly within one region
	for (int j = 0; j < num_traj_segments_; ++j)
	{
		prog_.AddLinearConstraint(H_(Eigen::all, j).sum() == 1);
	}

	for (int j = 0; j < num_traj_segments_; ++j)
	{
		for (int r = 0; r < num_regions_; ++r)
		{
			add_region_constraint(r,j);
		}
	}
}

void MISOSProblem::add_region_constraint(
		int region_number, int segment_number, bool always_enforce
		)
{
	for (int i = 0; i < regions_A_[region_number].rows(); ++i)
	{
		auto ai_transpose = regions_A_[region_number](i, Eigen::all);
		auto bi = regions_b_[region_number](i);

		drake::symbolic::Polynomial q;
		if (always_enforce)
		{
			// Force constaint to always be true
			q = drake::symbolic::Polynomial(
				bi - vehicle_radius_ - ai_transpose * coeffs_[segment_number] * m_,
				{t_}
				);
		}
		else
		{
			// Use binary decision variable and big M
			// to only enforce constraints when binary decision variable is 1
			q = drake::symbolic::Polynomial(
					big_M_ * (1 - H_(region_number, segment_number)) +
					bi - vehicle_radius_ - ai_transpose * coeffs_[segment_number] * m_,
					{t_}
					);
		}

		drake::symbolic::Polynomial sigma_1;
		drake::symbolic::Polynomial sigma_2;

		// Add second order cone constraint for polynomials of degree 3
		if (degree_ == 3)
		{
			coeff_matrix_t sigma_coeffs = prog_.NewContinuousVariables(2, 3, "Beta");

			sigma_1  = drake::symbolic::Polynomial(
					sigma_coeffs(0, Eigen::all).dot(m_(Eigen::seq(0,2))),
					{t_}
					);
			sigma_2  = drake::symbolic::Polynomial(
					sigma_coeffs(1, Eigen::all).dot(m_(Eigen::seq(0,2))),
					{t_}
					);

			prog_.AddRotatedLorentzConeConstraint(
					sigma_coeffs(0,0),
					sigma_coeffs(0,2),
					0.25 * sigma_coeffs(0,1) * sigma_coeffs(0,1)
					);

			prog_.AddRotatedLorentzConeConstraint(
					sigma_coeffs(1,0),
					sigma_coeffs(1,2),
					0.25 * sigma_coeffs(1,1) * sigma_coeffs(1,1)
					);
		}
		// Add SOS constraints for all other degrees
		// NOTE: Mosek does currently not support MISDP problems,
		// which MI with SOS constraints will be translated to.
		else
		{
			sigma_1 = prog_.NewSosPolynomial(
					{t_}, degree_ - 1
					).first;
			sigma_2 = prog_.NewSosPolynomial(
					{t_}, degree_ - 1
					).first;
		}

		// Add constraints: q(t) = t * sigma1(t) + (1 - t) * sigma2(t)
		// by setting coefficiants equal
		prog_.AddLinearConstraint(
				get_coefficients(q).array() ==
				get_coefficients(t_ * sigma_1 + sigma_2 - t_ * sigma_2).array()
				);
	}
}

void MISOSProblem::generate()
{
	result_ = Solve(prog_);
	//assert(result_.is_success());
	std::cout << "Solver id: " << result_.get_solver_id() << std::endl;
	std::cout << "Found solution: " << result_.is_success() << std::endl;
	std::cout << "Solution result: " << result_.get_solution_result() << std::endl;
	auto details = result_.get_solver_details<drake::solvers::MosekSolver>();
	std::cout << "Solver details: rescode: \n" << details.rescode << std::endl;
	std::cout << "Solver details: solution_status: \n" << details.solution_status << std::endl;

	auto inf_const = drake::solvers::GetInfeasibleConstraints(prog_, result_);
	std::cout << "Num inf consts: " << inf_const.size() << std::endl;
	for (auto i : inf_const)
	{
		std::cout << "Infeasible constraint: " << i << std::endl;
	}

	polynomials_.resize(num_vars_, num_traj_segments_);
	for (int j = 0; j < num_traj_segments_; ++j)
	{
		Eigen::VectorX<drake::symbolic::Expression> expr = result_
			.GetSolution(coeffs_[j]) * m_;

		for (int i = 0; i < num_vars_; ++i)
		{
			polynomials_(i,j) = drake::symbolic::Polynomial(expr[i]);
		}
	}
}

Eigen::VectorX<double> MISOSProblem::eval(double t)
{
	int traj_index = 0;
	while ((double) traj_index + 1 < t) ++traj_index;

	double t_rel = t - (double) traj_index;

	Eigen::VectorX<double> val(num_vars_);
	drake::symbolic::Environment at_t {{t_, t_rel}};
	for (int i = 0; i < num_vars_; ++i)
	{
		val(i) = polynomials_(i, traj_index).Evaluate(at_t);
	}

	return val;
}

} // Namespace trajopt
