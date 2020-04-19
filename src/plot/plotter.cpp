#include "plot/plotter.h"

void plot_traj(
		trajopt::MISOSProblem *traj, int num_traj_segments, Eigen::VectorX<double> init_pos, Eigen::VectorX<double> final_pos
		)
{
	int tf = num_traj_segments;

	// Plot trajectory
	const double delta_t = 0.01;
	int N = (int)(tf / delta_t);

	std::vector<double> x;
	std::vector<double> y;

	for (int i = 0; i < N; ++i)
	{
		double t = 0.0 + delta_t * i;
		x.push_back(traj->eval(t)(0));
		y.push_back(traj->eval(t)(1));
	}

	// Plot segment start and ends
	std::vector<double> sample_times_x;
	std::vector<double> sample_times_y;
	for (int t = 0; t <= num_traj_segments; ++t)
	{
			sample_times_x.push_back(traj->eval(t)(0));
			sample_times_y.push_back(traj->eval(t)(1));
	}

	typedef	std::unordered_map<std::string, std::string> string_map;

	// Plot init and final pos
	std::vector<double> init_x;
	std::vector<double> init_y;
	init_x.push_back(init_pos(0));
	init_x.push_back(final_pos(0));
	init_y.push_back(init_pos(1));
	init_y.push_back(final_pos(1));

	plt::scatter(init_x, init_y, 50, string_map({{"color","blue"}}));
	plt::scatter(sample_times_x, sample_times_y, 20, string_map({{"color","red"}}));
	
	plt::plot(x, y);

	plt::show();
}

void plot_PPTrajectory(trajopt::PPTrajectory *traj, Eigen::VectorXd sample_times, double tf)
{
	// TODO move some of this into trajopt
	const double delta_t = 0.01;
	int N = (int)(tf / delta_t);

	std::vector<double> x;
	std::vector<double> y;

	for (int i = 0; i < N; ++i)
	{
		double t = 0.0 + delta_t * i;
		x.push_back(traj->eval(t)(0));
		y.push_back(traj->eval(t)(1));
	}

	std::vector<double> sample_times_x;
	std::vector<double> sample_times_y;
	for (int i = 0; i < sample_times.size(); ++i)
	{
			sample_times_x.push_back(traj->eval(sample_times[i])(0));
			sample_times_y.push_back(traj->eval(sample_times[i])(1));
	}

	typedef	std::unordered_map<std::string, std::string> string_map;
	plt::plot(x, y);
	plt::scatter(sample_times_x, sample_times_y, 20, string_map({{"color","red"}}));
	plt::show();
}

void plot_convex_hull(std::vector<Eigen::VectorXd> points)
{
	auto convex_hull = makeConvexHull(points);
	plot_region(convex_hull);
}

void plot_region(std::vector<Eigen::VectorXd> points)
{
	std::vector<double> x;
	std::vector<double> y;

	for (int i = 0; i < points.size(); ++i)
	{	
		x.push_back(points[i](0));
		y.push_back(points[i](1));
	}

	x.push_back(points[0](0));
	y.push_back(points[0](1));

	//plt::fill(x, y, {});
	plt::plot(x,y);
}

// Takes obstacles defined by simplexes and plots them
// Obstacle: (x,y)
void plot_obstacles(std::vector<Eigen::MatrixXd> obstacles)
{
	for (int i = 0; i < obstacles.size(); ++i)
	{
		std::vector<double> x;
		std::vector<double> y;
		for (int row = 0; row < obstacles[i].rows(); ++row)			
		{
			x.push_back(obstacles[i](row,0));
			y.push_back(obstacles[i](row,1));
		}
		x.push_back(obstacles[i](0,0));
		y.push_back(obstacles[i](0,1));
		plt::fill(x, y, {});
	}
}


