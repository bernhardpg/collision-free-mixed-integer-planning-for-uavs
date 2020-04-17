#include "trajectory_optimization.h"


namespace trajopt
{
PPTrajectory::PPTrajectory(
		Eigen::VectorXd sample_times,
		int num_vars, 
		int degree,
		int continuity_degree
		) : 
	sample_times_(sample_times),
	num_vars_(num_vars),
	degree_(degree),
	continuity_degree_(continuity_degree)
{
	// One polynomial for each time slice
	for (int i = 0;	i < sample_times.size(); ++i)
	{
		// d + 1 coefficients for each variable
		coeffs_.push_back(prog_.NewContinuousVariables(num_vars, degree + 1, "C"));
	}

	// Add continuity constraints
	for (int s = 0; s < sample_times.size() - 1; ++s)
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
		for (int s = 0; s < sample_times_.size() - 1; ++s)
		{
			auto Q = Eigen::MatrixXd::Identity(num_vars_, num_vars_);
			auto b = Eigen::VectorXd::Zero(num_vars_);
			prog_.AddQuadraticCost(Q, b, coeffs_[s](Eigen::all, Eigen::last));
		}
	}
}

Eigen::VectorX<drake::symbolic::Expression> PPTrajectory::eval_symbolic(const double t)
{
	return PPTrajectory::eval(t, 0);
}


Eigen::VectorX<drake::symbolic::Expression> PPTrajectory::eval_symbolic(
		const double t, const int derivative_order
		)
{
	if (derivative_order > degree_)
	{
		auto zero_vec = Eigen::VectorX<drake::symbolic::Expression>::Zero(num_vars_);
		return zero_vec;
	}

	// Find the right polynomial segment
	int s = 0;
	while (s < sample_times_.size() - 1 && t >= sample_times_[s + 1]) ++s;

	const double time_rel = t - sample_times_[s];

	auto coeffs = coeffs_[s];
	
	const int derivative_deg = derivative_order;
	Eigen::VectorX<drake::symbolic::Expression> val(num_vars_);

	for (int var = 0; var < num_vars_; ++var)
	{
		drake::symbolic::Expression curr_val;
		for (int deg = derivative_deg; deg < degree_ + 1; ++deg)
		{
			curr_val += (coeffs(var, deg) * pow(time_rel, deg - derivative_deg)
				* factorial(deg) / factorial(deg - derivative_deg));
		}
		val(var) = curr_val;
	}
	return val;
}

Eigen::VectorXd PPTrajectory::eval(const double t)
{
	return eval(t, 0);
}

// TODO rewrite this somehow?
Eigen::VectorXd PPTrajectory::eval(const double t, const int derivative_order)
{
	if (derivative_order > degree_)
	{
		auto zero_vec = Eigen::VectorXd::Zero(num_vars_);
		return zero_vec;
	}

	// Find the right polynomial segment
	int s = 0;
	while (s < sample_times_.size() - 1 && t >= sample_times_[s + 1]) ++s;

	const double time_rel = t - sample_times_[s];

	Eigen::MatrixXd coeffs = result_.GetSolution(coeffs_[s]);
	
	const int derivative_deg = derivative_order;
	Eigen::VectorXd val(num_vars_);

	for (int var = 0; var < num_vars_; ++var)
	{
		double curr_val = 0;
		for (int deg = derivative_deg; deg < degree_ + 1; ++deg)
		{
			curr_val += (coeffs(var, deg) * pow(time_rel, deg - derivative_deg))
				* factorial(deg) / factorial(deg - derivative_deg);
		}
		val(var) = curr_val;
	}
	return val;
}

void PPTrajectory::add_constraint(
		const double t, const int derivative_order, Eigen::VectorXd lb
		)
{
	add_constraint(t, derivative_order, lb, lb);
}

void PPTrajectory::add_constraint(
		const double t, const int derivative_order, Eigen::VectorXd lb, Eigen::VectorXd ub
		)
{
	assert(derivative_order <= degree_);
	Eigen::VectorX<drake::symbolic::Expression> val = eval_symbolic(t, derivative_order);
	prog_.AddLinearConstraint(val, lb, ub);
}

void PPTrajectory::generate()
{
	result_ = Solve(prog_);
	//assert(result_.is_success());
	std::cout << "Solver id: " << result_.get_solver_id() << std::endl;
	std::cout << "Found solution: " << result_.is_success() << std::endl;
	std::cout << "Solution result: " << result_.get_solution_result() << std::endl;
}

void test_iris()
{
	std::cout << "Testing IRIS" << std::endl;

	// Create bounding box
	Eigen::MatrixXd A(4,2);
	A << -1, 0,
				0, -1,
				1, 0,
				0, 1;
	Eigen::VectorXd b(4);
	b << 0, 0, 5, 5;
	iris::Polyhedron bounds(A,b);

	iris::IRISProblem problem(2);
	problem.setBounds(bounds);
	problem.setSeedPoint(Eigen::Vector2d(0.1, 0.1));

	std::vector<Eigen::MatrixXd> obstacles;
	Eigen::MatrixXd obs(4,2);
	obs << 2, 0,
				 2, 2,
				 3, 2,
				 3, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);
	obs << -1, 0,
				 -1, 2,
				  0, 2,
					0.2, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

  iris::IRISOptions options;
  iris::IRISRegion region = inflate_region(problem, options);

	iris::Polyhedron solution = region.getPolyhedron();
	
	auto points = solution.generatorPoints();
	plot_convex_hull(points);
	plot_obstacles(obstacles);
}


void test_polynomial_trajectory()
{
	const int tf = 3;
	auto sample_times = Eigen::VectorXd::LinSpaced(6, 0, tf);
	const int num_vars = 2;
	const int degree = 5;
	const int continuity_degree = 4;

	PPTrajectory traj(sample_times, num_vars, degree, continuity_degree);
	
	Eigen::Matrix<double, 5, 2> points;
	points << 0.0, 0.0,
						1.0, 2.0,
						5.0, 1.5,
						3.0, 5.0,
						6.0, 3.0;
		
	traj.add_constraint(0.0, 0, points(0, Eigen::all));
	traj.add_constraint(0.0, 1, points(0, Eigen::all));
	traj.add_constraint(0.0, 2, points(0, Eigen::all));
	traj.add_constraint(1.0, 0, points(1, Eigen::all));
	traj.add_constraint(1.5, 0, points(2, Eigen::all));
	traj.add_constraint(2.5, 0, points(3, Eigen::all));
	traj.add_constraint(tf, 0, points(4, Eigen::all));
	traj.add_constraint(tf, 1, points(0, Eigen::all));
	traj.add_constraint(tf, 2, points(0, Eigen::all));
	
	traj.generate();

	const double delta_t = 0.01;
	int N = (int)(tf / delta_t);
	
	std::vector<double> x;
	std::vector<double> y;

	for (int i = 0; i < N; ++i)
	{
		double t = 0.0 + delta_t * i;
		x.push_back(traj.eval(t)(0));
		y.push_back(traj.eval(t)(1));
	}

	auto x_vec = points(Eigen::all, 0);
	auto y_vec = points(Eigen::all, 1);
	std::vector<double> points_x_coord(x_vec.data(), x_vec.data() + x_vec.rows());
	std::vector<double> points_y_coord(y_vec.data(), y_vec.data() + y_vec.rows());

	typedef	std::unordered_map<std::string, std::string> string_map;
	plt::scatter(points_x_coord, points_y_coord, 20, string_map({{"color","red"}}));

	std::vector<double> sample_times_x;
	std::vector<double> sample_times_y;
	for (int i = 0; i < sample_times.size(); ++i)
	{
			sample_times_x.push_back(traj.eval(sample_times[i])(0));
			sample_times_y.push_back(traj.eval(sample_times[i])(1));
	}

	plt::scatter(sample_times_x, sample_times_y, 10);

	plt::plot(x, y);

	plt::show();


	/*
	drake::solvers::MathematicalProgram	prog;

	Eigen::Matrix2d A;
	A << 1, 2, 3, 4;
	Eigen::Vector2d b;
	b << 10, 10;

	std::cout << A(Eigen::all,Eigen::last) << std::endl;

	auto state = prog.NewContinuousVariables(2, "state");
	prog.AddLinearConstraint((A * state).array() >= b.array());
	prog.AddCost(state.transpose() * state);

	drake::solvers::MathematicalProgramResult result = Solve(prog);
	assert(result.is_success());
	std::cout << "Result: " << std::endl << result.GetSolution() << std::endl;
	*/



	return;
}

int factorial(int n)
{
	return (n==1 || n==0) ? 1: n * factorial(n - 1);
}


} // Namespace trajopt
