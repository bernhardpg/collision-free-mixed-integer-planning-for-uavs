#include "trajopt/safe_regions.h"


namespace trajopt
{

SafeRegions::SafeRegions(int num_dimensions)
	: num_dimensions_(num_dimensions),
		iris_problem_(num_dimensions)
{
  options_ = iris::IRISOptions();
}

void SafeRegions::set_bounds(
		double x_min, double x_max,
		double y_min, double y_max
		)
{
	assert(num_dimensions_ == 2);

	x_min_ = x_min;
	y_min_ = y_min;

	x_max_ = x_max;
	y_max_ = y_max;

	Eigen::MatrixXd A_bounds(4,2);
	A_bounds << -1, 0,
							0, -1,
							1, 0,
							0, 1;

	Eigen::VectorXd b_bounds(4);
	b_bounds << -x_min, -y_min,
							 x_max, y_max;

	iris::Polyhedron bounds(A_bounds, b_bounds);
	iris_problem_.setBounds(bounds);
}

void SafeRegions::set_bounds(
		double x_min, double x_max,
		double y_min, double y_max,
		double z_min, double z_max
		)
{
	assert(num_dimensions_ == 3);

	x_min_ = x_min;
	y_min_ = y_min;
	z_min_ = z_min;

	x_max_ = x_max;
	y_max_ = y_max;
	z_max_ = z_max;

	auto pair = halfspace_from_bounds(x_min, x_max, y_min, y_max, z_min, z_max);
	Eigen::MatrixXd A_bounds = pair.first;
	Eigen::VectorXd b_bounds = pair.second;

	iris::Polyhedron bounds(A_bounds, b_bounds);
	iris_problem_.setBounds(bounds);
}


void SafeRegions::set_obstacles(std::vector<Eigen::Matrix3Xd> obstacles)
{
	for (auto obstacle : obstacles)
		iris_problem_.addObstacle(obstacle);

	obstacles_ = obstacles;

	// Create half space representation of obstacles
	// TODO: This only currently works with boxes without any rotation.
	for (auto obstacle : obstacles)
	{
		double x_min = x_max_;
		double y_min = y_max_;
		double z_min = z_max_;

		double x_max = x_min_;
		double y_max = y_min_;
		double z_max = z_min_;

		for (int i = 0; i < obstacle.cols(); ++i)
		{
			Eigen::Vector3d vertex = obstacle(Eigen::all, i);
			x_min = std::min(x_min, vertex(0));
			y_min = std::min(y_min, vertex(1));
			z_min = std::min(z_min, vertex(2));

			x_max = std::max(x_max, vertex(0));
			y_max = std::max(y_max, vertex(1));
			z_max = std::max(z_max, vertex(2));
		}

		auto pair = halfspace_from_bounds(x_min, x_max, y_min, y_max, z_min, z_max);
		Eigen::MatrixXd A = pair.first;
		Eigen::VectorXd b = pair.second;
			
		obstacles_As_.push_back(A);
		obstacles_bs_.push_back(b);
	}
}

void SafeRegions::calc_safe_regions_from_seedpoints(std::vector<Eigen::Vector3d> seedpoints)
{
	seedpoints_ = seedpoints;

	// Obtain convex regions
	for (auto seedpoint : seedpoints_)
	{
		iris_problem_.setSeedPoint(seedpoint);
		iris::IRISRegion region = inflate_region(iris_problem_, options_);
		safe_regions_.push_back(region.getPolyhedron());
	}

	for (int i = 0; i < safe_regions_.size(); ++i)
	{
		safe_region_As_.push_back(safe_regions_[i].getA());
		safe_region_bs_.push_back(safe_regions_[i].getB());
	}
}

void SafeRegions::calc_safe_regions_auto(int num_seeds)
{
	// TODO Implement for 2 dimensions if needed
	assert(num_dimensions_ == 3);

	for(int i = 0; i < num_seeds; ++i)
		calc_safe_region(find_best_point());
}

// *********
// Private helper functions
// *********

void SafeRegions::calc_safe_region(Eigen::Vector3d seedpoint)
{
	iris_problem_.setSeedPoint(seedpoint);
	iris::IRISRegion region = inflate_region(iris_problem_, options_);
	iris::Polyhedron iris_poly = region.getPolyhedron();
	safe_regions_.push_back(iris_poly);

	// Add halfspace representations
	safe_region_As_.push_back(iris_poly.getA());
	safe_region_bs_.push_back(iris_poly.getB());
}

Eigen::Vector3d SafeRegions::find_best_point()
{
	// TODO this should be changeable
	double dx = 0.3;
	double dy = 0.3;
	double dz = 1;

	Eigen::Vector3d point(x_min_, y_min_, z_min_);

	Eigen::Vector3d best_point = point;
	double max_dist = 0;

	for (double z = z_min_; z <= z_max_; z += dz)
		for (double y = y_min_; y <= y_max_; y += dy)
			for (double x = x_min_; x <= x_max_; x += dx)
			{
				Eigen::Vector3d point(x, y, z);
				if (!is_collision(point))
				{
					double curr_dist = calc_min_dist(point);

					if (curr_dist > max_dist)
					{
						best_point = point;
						max_dist = curr_dist; 
					}
				}
			}

	assert(!is_collision(best_point));

	return best_point;
}

double SafeRegions::calc_min_dist(Eigen::Vector3d point)
{
	double min_dist = 999999;

	// Check distance to all obstacles
	for (auto obstacle : obstacles_)
	{
		for (int i = 0; i < obstacle.cols(); ++i)
			for (int j = 0; j < obstacle.cols(); ++j)
			{
				// Don't check same point
				if (i == j) continue;

				Eigen::Vector3d p1 = obstacle(Eigen::all, i);
				Eigen::Vector3d p2 = obstacle(Eigen::all, j);

				double dist = dist_point_to_line(point, p1, p2);
				min_dist = std::min(dist, min_dist);
			}
	}

	// Check distance to all regions
	for (auto region : safe_regions_)
	{
		std::vector<Eigen::VectorXd> vertices = region.generatorPoints();

		for (int i = 0; i < vertices.size(); ++i)
			for (int j = 0; j < vertices.size(); ++j)
			{
				// Don't check same point
				if (i == j) continue;

				Eigen::Vector3d p1 = vertices[i];
				Eigen::Vector3d p2 = vertices[j];

				double dist = dist_point_to_line(point, p1, p2);
				min_dist = std::min(dist, min_dist);
			}
	}

	return min_dist;
}


bool SafeRegions::is_collision(Eigen::Vector3d point)
{
	// Check collision with regions given as half spaces
	for (int i = 0; i < safe_region_As_.size(); ++i)
	{
		auto A = safe_region_As_[i];
		auto b = safe_region_bs_[i];

		if (point_inside_halfspace(point, A, b))
		{
			return true;
		}
	}

	// Check collision with obstacles given as half spaces
	for (int i = 0; i < obstacles_As_.size(); ++i)
	{
		auto A = obstacles_As_[i];
		auto b = obstacles_bs_[i];
		
		if (point_inside_halfspace(point, A, b))
		{
			return true;
		}
	}

	return false;
}

bool SafeRegions::point_inside_halfspace(
		Eigen::Vector3d point, Eigen::MatrixXd A, Eigen::VectorXd b
		)
{
	// A * point <= b : Inside half space
	Eigen::VectorXd temp = A * point;
	for (int i = 0; i < temp.size(); ++i)
	{
		if (temp(i) > b(i)) // outside at least one face
			return false;
	}
	return true;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> SafeRegions::halfspace_from_bounds(
		double x_min, double x_max,
		double y_min, double y_max,
		double z_min, double z_max
		)
{
		Eigen::MatrixXd A(6,3);
		A << -1, 0, 0,
					0, -1, 0,
					0, 0, -1,
					1, 0, 0,
					0, 1, 0,
					0, 0, 1;

		Eigen::VectorXd b(6);
		b << -x_min, -y_min, -z_min,
					x_max, y_max, z_max;

		std::pair<Eigen::MatrixXd, Eigen::VectorXd> pair = std::make_pair(A, b);

		return pair;
}

// Source:
// https://stackoverflow.com/questions/10983872/distance-from-a-point-to-a-polygon
double SafeRegions::dist_point_to_line(
		Eigen::Vector3d point, Eigen::Vector3d p1, Eigen::Vector3d p2
		)
{
		double dist;

		auto r = (p2 - p1).dot(point - p1);
		r /= pow((p2 - p1).norm(), 2);

		// p1 closest point
		if (r < 0)
			dist = (point - p1).norm();
		// p2 closest point
		else if (r > 1)
			dist = (p2 - point).norm();
		// closest point is on line
		else
			dist = sqrt(
					pow((point - p1).norm(), 2)
					- pow((r * (p2 - p1).norm()), 2)
					);

		return dist;
}

} // namespace trajopt
