#include "trajectory_optimization.h"


namespace trajopt
{

MISOSProblem::MISOSProblem(
		int num_traj_segments,
		int num_vars,
		int degree,
		int continuity_degree
		) :
	num_traj_segments_(num_traj_segments),
	num_vars_(num_vars),
	degree_(degree),
	continuity_degree_(continuity_degree)
{
	t_ = prog_.NewIndeterminates(1, 1, "t")(0,0);
	m_.resize(degree_ + 1);

	for (int d = 0; d < degree + 1; ++d)
	{
		m_(d) = drake::symbolic::Monomial(t_, d).ToExpression();
	}

	// One polynomial for each time slice
	for (int j = 0;	j < num_traj_segments_; ++j)
	{
		// d + 1 coefficients for each variable
		coeff_matrix_t c = prog_.NewContinuousVariables(
				num_vars, degree + 1, "C");
		coeffs_.push_back(c);

		// Add coefficients for each derivative degree
		std::vector<coeff_matrix_t> coeffs_dj;
		for (int der_degree = 1; der_degree < continuity_degree_ + 1; ++der_degree)
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
		}
		coeffs_d.push_back(coeffs_dj);
	}

	// Plan
	// 1. Add m list of monomials
	// 2. Calculate polynomial from this
	// 3. Calculate derivatives of the polynomial
	// 4. Get the coefficients of the derivatives
	// 5. Do this for every derivative order and for every traj segment
	// then
	// 6. Constrain initial and final value
	// 7. Enforce continuity on the derivatives of the polynomial


	/*
	// Add continuity constraints
	for (int s = 0; s < num_traj_segments_ - 1; ++s)
	{
		double time_rel = sample_times[s + 1] - sample_times[s];
		auto curr_coeffs = coeffs_[s];

		for (int var = 0; var < num_vars; ++var)
		{
			// For every derivative degree we enforce continuity on
			for (int derivative_deg = 0; derivative_deg < continuity_degree + 1; ++derivative_deg)
			{
				drake::symbolic::Expression left_val;
				for (int deg = derivative_deg; deg < degree_ + 1; ++deg)
				{
					// Calculate (d/dt)**derivatice_deg of P_j(1)
					left_val += (curr_coeffs(var, deg) * pow(time_rel, deg - derivative_deg)
						* factorial(deg) / factorial(deg - derivative_deg));
				}
				// Calculate (d/dt)**derivatice_deg of P_j+1(0)
				// (Note: only the constant term remains)
				drake::symbolic::Expression right_val = (coeffs_[s + 1](var, derivative_deg)
					* factorial(derivative_deg));
				prog_.AddLinearConstraint(left_val == right_val);
			}
		}

		// Add cost to minimize highest order terms
		for (int s = 0; s < num_traj_segments_ - 1; ++s)
		{
			auto Q = Eigen::MatrixXd::Identity(num_vars_, num_vars_);
			auto b = Eigen::VectorXd::Zero(num_vars_);
			prog_.AddQuadraticCost(Q, b, coeffs_[s](Eigen::all, Eigen::last));
		}
	}
	*/

}


int factorial(int n)
{
	return (n==1 || n==0) ? 1: n * factorial(n - 1);
}

} // Namespace trajopt
