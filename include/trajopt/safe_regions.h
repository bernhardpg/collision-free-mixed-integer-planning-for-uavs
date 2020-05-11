#pragma once

#include "iris/iris.h"

namespace trajopt
{
	// A wrapper class for IRIS
	// Somewhere to put functionality for seeding etc.
	class SafeRegions
	{
		public:
			SafeRegions(int num_dimensions);

			void set_bounds(Eigen::MatrixXd A, Eigen::VectorXd b);
			void set_seedpoints(std::vector<Eigen::Vector3d> seed_points);
			void set_obstacles(std::vector<Eigen::Matrix3Xd> obstacles);
			void calc_safe_regions();

			std::vector<Eigen::MatrixXd> get_As() { return safe_region_As_; };
			std::vector<Eigen::VectorXd> get_bs() { return safe_region_bs_; };
			std::vector<iris::Polyhedron> get_polyhedrons() { return convex_polygons_; };

		private:
			int num_dimensions_;
			iris::IRISProblem iris_problem_;
			iris::IRISOptions options_;
			std::vector<Eigen::Vector3d> seed_points_;

			std::vector<iris::Polyhedron> convex_polygons_;
			std::vector<Eigen::MatrixXd> safe_region_As_;
			std::vector<Eigen::VectorXd> safe_region_bs_;
	};
} // namespace trajopt

