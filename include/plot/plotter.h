#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "plot/matplotlibcpp.h"
#include "tools/ConvexHull.h"
#include "trajopt/PPTrajectory.h"
#include "trajopt/MISOSProblem.h"

namespace plt = matplotlibcpp;

void plot_traj(
		trajopt::MISOSProblem *traj, int num_traj_segments, Eigen::VectorX<double> init_pos, Eigen::VectorX<double> final_pos
		);
void plot_2d_obstacles(std::vector<Eigen::MatrixXd> obstacles);
void plot_2d_region(std::vector<Eigen::VectorXd> points, bool filled);
void plot_3d_regions_footprint(
		std::vector<iris::Polyhedron> convex_polygons, double cross_section_height
		);
void plot_3d_obstacles_footprints(
		std::vector<Eigen::Matrix3Xd> obstacles, double cross_section_height
		);

void plot_2d_convex_hull(std::vector<Eigen::VectorXd> points, bool filled, bool show);
void plot_2d_convex_hull(std::vector<Eigen::VectorXd> points, bool filled);
void plot_2d_convex_hull(std::vector<Eigen::VectorXd> points);
