#pragma once

#include "iris/iris.h"
#include <cmath>

namespace trajopt
{
	// A wrapper class for IRIS
	// Somewhere to put functionality for seeding etc.
	class SafeRegions
	{
		public:
			SafeRegions(int num_dimensions);

			void set_bounds(
					double x_min, double x_max,
					double y_min, double y_max
					);
			void set_bounds(
					double x_min, double x_max,
					double y_min, double y_max,
					double z_min, double z_max
					);
			void set_obstacles(std::vector<Eigen::Matrix3Xd> obstacles);

			void calc_safe_regions_auto(int num_seeds);
			void calc_safe_regions_from_seedpoints(
					std::vector<Eigen::Vector3d> seedpoints
					);

			std::vector<Eigen::MatrixXd> get_As() { return safe_region_As_; };
			std::vector<Eigen::VectorXd> get_bs() { return safe_region_bs_; };
			std::vector<iris::Polyhedron> get_polyhedrons() { return safe_regions_; };

		private:
			int num_dimensions_;
			double x_min_;
			double x_max_;
			double y_min_;
			double y_max_;
			double z_min_;
			double z_max_;
			iris::IRISProblem iris_problem_;
			iris::IRISOptions options_;
			std::vector<Eigen::Vector3d> seedpoints_;
			std::vector<Eigen::Matrix3Xd> obstacles_;
			std::vector<Eigen::MatrixXd> obstacles_As_;
			std::vector<Eigen::VectorXd> obstacles_bs_;

			std::vector<iris::Polyhedron> safe_regions_;
			std::vector<Eigen::MatrixXd> safe_region_As_;
			std::vector<Eigen::VectorXd> safe_region_bs_;

			void calc_safe_region(Eigen::Vector3d seedpoint);

			Eigen::Vector3d find_best_point();
			bool is_collision(Eigen::Vector3d point);
			double calc_min_dist(Eigen::Vector3d point);

			bool point_inside_halfspace(
					Eigen::Vector3d point, Eigen::MatrixXd A, Eigen::VectorXd b
					);
			double dist_point_to_line(
					Eigen::Vector3d point, Eigen::Vector3d p1, Eigen::Vector3d p2
					);
			std::pair<Eigen::MatrixXd, Eigen::VectorXd> halfspace_from_bounds(
					double x_min, double x_max,
					double y_min, double y_max,
					double z_min, double z_max
					);

	};
} // namespace trajopt

