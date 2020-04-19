#include "trajectory_optimization.h"


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

	// TODO add derivative initial conditions too?
	/*// Add initial and final conditions on derivatives
	for (int d = 1; d < init_cond.cols(); ++d)
	{
		prog_.AddLinearConstraint((coeffs_d_[0][d - 1] * m_value_t0).array() == init_cond(Eigen::all, 0).array());
	}
	for (int d = 1; d < final_cond.cols(); ++d)
	{
		prog_.AddLinearConstraint((coeffs_d_[num_traj_segments_ - 1][d - 1] * m_value_t0).array() == final_cond(Eigen::all, 0).array());
	}
	*/

	// TODO change
	// Force first derivatives at initial and final position to be 0
	for (int d = 1; d <= 1; ++d)
	{
		prog_.AddLinearConstraint(
				(coeffs_d_[0][d - 1] * m_value_t0).array() == Eigen::Vector2d(0,0).array()
			);
		prog_.AddLinearConstraint(
				(coeffs_d_[num_traj_segments_ - 1][d - 1] * m_value_t1).array()
				== Eigen::Vector2d(0,0).array()
			);
	}

	// Add cost to minimize highest derivative order coefficients
	for (int j = 0; j < num_traj_segments_; ++j)
	{ 
		// Remember that highest order derivative only has one coefficient
		auto quadratic_form = coeffs_d_[j][degree_ - 1](Eigen::all, 0)
			.dot(coeffs_d_[j][degree_ - 1](Eigen::all, 0));
		prog_.AddQuadraticCost(quadratic_form);
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

void MISOSProblem::add_region_constraint(Eigen::MatrixXd A, Eigen::VectorXd b, int segment_number)
{
	for (int i = 0; i < A.rows(); ++i)
	{
		auto ai_transpose = A(i, Eigen::all);
		auto bi = b(i);
		drake::symbolic::Polynomial q(
				bi - vehicle_radius_ - ai_transpose * coeffs_[segment_number] * m_, {t_}
				);

		drake::symbolic::Polynomial sigma_1 = prog_.NewSosPolynomial({t_}, degree_ - 1).first;
		drake::symbolic::Polynomial sigma_2 = prog_.NewSosPolynomial({t_}, degree_ - 1).first;

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
