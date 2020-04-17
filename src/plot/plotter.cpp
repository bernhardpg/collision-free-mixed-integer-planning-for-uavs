#include "plot/plotter.h"

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

	plt::fill(x, y, {});
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
		plt::plot(x, y);
	}

	plt::show();
}


