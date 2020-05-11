#include "trajopt/safe_regions.h"


namespace trajopt
{

SafeRegions::SafeRegions(int num_dimensions)
	: num_dimensions_(num_dimensions),
		iris_problem_(num_dimensions)
{
  options_ = iris::IRISOptions();
}

void SafeRegions::set_bounds(Eigen::MatrixXd A, Eigen::VectorXd b)
{
	iris::Polyhedron bounds(A, b);
	iris_problem_.setBounds(bounds);
}

void SafeRegions::set_seedpoints(std::vector<Eigen::Vector3d> seed_points)
{
	seed_points_ = seed_points;
}

void SafeRegions::set_obstacles(std::vector<Eigen::Matrix3Xd> obstacles)
{
	for (auto obstacle : obstacles)
		iris_problem_.addObstacle(obstacle);
}

void SafeRegions::calc_safe_regions()
{
	// Obtain convex regions
	for (auto seed_point : seed_points_)
	{
		iris_problem_.setSeedPoint(seed_point);
		iris::IRISRegion region = inflate_region(iris_problem_, options_);
		convex_polygons_.push_back(region.getPolyhedron());
	}

	for (int i = 0; i < convex_polygons_.size(); ++i)
	{
		// Matrix containing one convex region
		safe_region_As_.push_back(convex_polygons_[i].getA());
		safe_region_bs_.push_back(convex_polygons_[i].getB());
	}
}
} // namespace trajopt
