#include "test/tests.h"

void test_trajectory_socp_fix_mi_variables()
{
	// Create bounding box
	Eigen::MatrixXd A_bounds(4,2);
	A_bounds << -1, 0,
				0, -1,
				1, 0,
				0, 1;
	Eigen::VectorXd b_bounds(4);
	b_bounds << 2, 0, 5, 5;
	iris::Polyhedron bounds(A_bounds,b_bounds);

	// Construct 2D test environment
	iris::IRISProblem problem(2);
	problem.setBounds(bounds);

	std::vector<Eigen::MatrixXd> obstacles;
	Eigen::MatrixXd obs(4,2);

	obs << 4, 0,
				 5, 2,
				 4, 2,
				 5, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	obs << -1, 0,
				 -1, 2,
				  0, 2,
					0.2, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	obs << 2, 0,
				 2, 4,
				 2.2, 4,
				 2.2, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	obs << 3,3,
				 5,3,
				 5,4,
				 3,4;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	/*
	obs << 0.5,3,
				 2,3,
				 2,4,
				 0.5,4;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);
	*/

	// Get convex regions

  iris::IRISOptions options;
	std::vector<Eigen::Vector2d> seed_points;
	seed_points.push_back(Eigen::Vector2d(1,1));
	seed_points.push_back(Eigen::Vector2d(2.1,4.5));
	seed_points.push_back(Eigen::Vector2d(2.5,3.5));
	seed_points.push_back(Eigen::Vector2d(4.5,2.5));
	//seed_points.push_back(Eigen::Vector2d(0.5,3.5));

	std::vector<iris::Polyhedron> convex_polygons;

	for (int i = 0; i < seed_points.size(); ++i)
	{
		problem.setSeedPoint(seed_points[i]);
		iris::IRISRegion region = inflate_region(problem, options);
		convex_polygons.push_back(region.getPolyhedron());
	}

	std::vector<Eigen::MatrixXd> As;
	std::vector<Eigen::VectorXd> bs;
	for (int i = 0; i < convex_polygons.size(); ++i)
	{
		// Matrix containing one convex region
		As.push_back(convex_polygons[i].getA());
		bs.push_back(convex_polygons[i].getB());
	}

	// Plot convex regions
	for (int i = 0; i < convex_polygons.size(); ++i)
	{
		auto points = convex_polygons[i].generatorPoints();
		plot_2d_convex_hull(points);
	}

	plot_2d_obstacles(obstacles);

	// Create trajectory with degree 3 for SOCP constraints
	int num_vars = 2;
	int num_traj_segments = 8;
	int degree = 3;
	int cont_degree = 2;
	Eigen::VectorX<double> init_pos(num_vars);
	init_pos << 1.0, 1.0;

	Eigen::VectorX<double> final_pos(num_vars);
	final_pos << 4.5, 2.5;

	auto traj_3rd_deg = trajopt::MISOSProblem(num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos);
	traj_3rd_deg.add_convex_regions(As, bs);
	traj_3rd_deg.create_region_binary_variables();
	traj_3rd_deg.generate();
	Eigen::MatrixX<int> safe_region_assignments = traj_3rd_deg.get_region_assignments();
	std::cout << "Found 3rd order" << std::endl;

	// Create trajectory with degree 5 with fixed region constraints
	degree = 5;
	cont_degree = 4;

	auto traj = trajopt::MISOSProblem(num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos);
	traj.add_convex_regions(As, bs);
	traj.add_safe_region_assignments(safe_region_assignments);
	traj.generate();
	plot_traj(&traj, num_traj_segments, init_pos, final_pos);
	std::cout << "Found 5th order" << std::endl;
}


void test_trajopt()
{
	int num_vars = 2;
	int num_traj_segments = 3;
	int degree = 5;
	int cont_degree = 5;
	Eigen::VectorX<double> init_pos(num_vars);
	init_pos << 5.0, 0.0;

	Eigen::VectorX<double> final_pos(num_vars);
	final_pos << 1.0, 15.0;

	auto traj = trajopt::MISOSProblem(num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos);

	traj.generate();
	plot_traj(&traj, num_traj_segments, init_pos, final_pos);
}


void test_iris3d()
{
	// Create bounding box
	Eigen::MatrixXd A_bounds(6,3);
	A_bounds << -1, 0, 0,
							0, -1, 0,
							0, 0, -1,
							1, 0, 0,
							0, 1, 0,
							0, 0, 1;

	Eigen::VectorXd b_bounds(6);
	b_bounds << 0, 0, 0, 5, 5, 2;
	iris::Polyhedron bounds(A_bounds,b_bounds);

	// Construct 2D test environment
	iris::IRISProblem problem(3);
	problem.setBounds(bounds);


	std::vector<Eigen::MatrixXd> obstacles;
	Eigen::MatrixXd obs(8,3);
	obs << 4, 0, 0,
				 5, 2, 0, 
				 4, 2, 0,
				 5, 0, 0,
				 5, 0, 1,
				 4, 2, 1,
				 5, 2, 1, 
				 4, 0, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);
	obs << -1, 0, 0,
				 -1, 2, 0,
				  0, 2, 0,
					0.2, 0, 0,
					0.2, 0, 1,
				  0, 2, 1,
				 -1, 2, 1,
				 -1, 0, 1;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	obs << 2, 0, 0,
				 2, 4, 0,
				 2.2, 4, 0,
				 2.2, 0, 0,
				 2.2, 0, 1,
				 2.2, 4, 1,
				 2, 4, 1,
				 2, 0, 1;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

  iris::IRISOptions options;
	std::vector<Eigen::Vector3d> seed_points;
	seed_points.push_back(Eigen::Vector3d(1,1,0.5));
	seed_points.push_back(Eigen::Vector3d(2.1,4.5, 0.5));
	seed_points.push_back(Eigen::Vector3d(3,1, 0.5));
	seed_points.push_back(Eigen::Vector3d(2.5,3.5,0.5));
	seed_points.push_back(Eigen::Vector3d(4.5,2.5,0.5));

	// Get convex regions
	std::vector<iris::Polyhedron> convex_polygons;
	for (int i = 0; i < 1; ++i)
	{
		problem.setSeedPoint(seed_points[i]);
		iris::IRISRegion region = inflate_region(problem, options);
		convex_polygons.push_back(region.getPolyhedron());
		auto vertices = region.getPolyhedron().generatorPoints();
		for (int j = 0; j < vertices.size(); ++j)
		{
			std::cout << vertices[j] << std::endl << std::endl;
		}
	}
}

void test_trajectory_with_iris()
{
	// Create bounding box
	Eigen::MatrixXd A_bounds(4,2);
	A_bounds << -1, 0,
				0, -1,
				1, 0,
				0, 1;
	Eigen::VectorXd b_bounds(4);
	b_bounds << 2, 0, 5, 5;
	iris::Polyhedron bounds(A_bounds,b_bounds);

	// Construct 2D test environment
	iris::IRISProblem problem(2);
	problem.setBounds(bounds);

	std::vector<Eigen::MatrixXd> obstacles;
	Eigen::MatrixXd obs(4,2);

	obs << 4, 0,
				 5, 2,
				 4, 2,
				 5, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	obs << -1, 0,
				 -1, 2,
				  0, 2,
					0.2, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	obs << 2, 0,
				 2, 4,
				 2.2, 4,
				 2.2, 0;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	obs << 3,3,
				 5,3,
				 5,4,
				 3,4;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);

	/*
	obs << 0.5,3,
				 2,3,
				 2,4,
				 0.5,4;
	problem.addObstacle(obs.transpose());
	obstacles.push_back(obs);
	*/

	// Get convex regions

  iris::IRISOptions options;
	std::vector<Eigen::Vector2d> seed_points;
	seed_points.push_back(Eigen::Vector2d(1,1));
	seed_points.push_back(Eigen::Vector2d(2.1,4.5));
	seed_points.push_back(Eigen::Vector2d(2.5,3.5));
	seed_points.push_back(Eigen::Vector2d(4.5,2.5));
	//seed_points.push_back(Eigen::Vector2d(0.5,3.5));

	std::vector<iris::Polyhedron> convex_polygons;

	for (int i = 0; i < seed_points.size(); ++i)
	{
		problem.setSeedPoint(seed_points[i]);
		iris::IRISRegion region = inflate_region(problem, options);
		convex_polygons.push_back(region.getPolyhedron());
	}

	std::vector<Eigen::MatrixXd> As;
	std::vector<Eigen::VectorXd> bs;
	for (int i = 0; i < convex_polygons.size(); ++i)
	{
		// Matrix containing one convex region
		As.push_back(convex_polygons[i].getA());
		bs.push_back(convex_polygons[i].getB());
	}

	// Plot convex regions
	for (int i = 0; i < convex_polygons.size(); ++i)
	{
		auto points = convex_polygons[i].generatorPoints();
		plot_2d_convex_hull(points);
	}

	plot_2d_obstacles(obstacles);

	// Create trajectory
	int num_vars = 2;
	int num_traj_segments = 8;
	int degree = 3;
	int cont_degree = 2;
	Eigen::VectorX<double> init_pos(num_vars);
	init_pos << 1.0, 1.0;

	Eigen::VectorX<double> final_pos(num_vars);
	final_pos << 4.5, 2.5;

	auto traj = trajopt::MISOSProblem(num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos);

	traj.add_convex_regions(As, bs);
	traj.create_region_binary_variables();
	//traj.add_region_constraint(0, 0, true);
	//traj.add_region_constraint(0, 1, true);
	//traj.add_region_constraint(1, 2, true);
	//traj.add_region_constraint(1, 3, true);
	//traj.add_region_constraint(2, 4, true);
	//traj.add_region_constraint(2, 5, true);
	//traj.add_region_constraint(3, 6, true);
	//traj.add_region_constraint(3, 7, true);
	traj.generate();

	plot_traj(&traj, num_traj_segments, init_pos, final_pos);
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
	plot_2d_convex_hull(points);
	plot_2d_obstacles(obstacles);
}


void test_polynomial_trajectory()
{
	const int tf = 3;
	auto sample_times = Eigen::VectorXd::LinSpaced(6, 0, tf);
	const int num_vars = 2;
	const int degree = 5;
	const int continuity_degree = 4;

	trajopt::PPTrajectory traj(sample_times, num_vars, degree, continuity_degree);

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

	return;
}


void test_mathematical_program()
{
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

	return;
}
