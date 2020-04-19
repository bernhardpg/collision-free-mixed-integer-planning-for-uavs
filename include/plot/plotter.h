#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "plot/matplotlibcpp.h"
#include "tools/ConvexHull.h"
#include "trajopt/PPTrajectory.h"
#include "trajectory_optimization.h"

namespace plt = matplotlibcpp;

void plot_traj(
		trajopt::MISOSProblem *traj, int num_traj_segments, Eigen::VectorX<double> init_pos, Eigen::VectorX<double> final_pos
		);
void plot_PPTrajectory(
		trajopt::PPTrajectory *traj, Eigen::VectorXd sample_times, double tf
		);
void plot_obstacles(std::vector<Eigen::MatrixXd> obstacles);
void plot_convex_hull(std::vector<Eigen::VectorXd> points);
void plot_region(std::vector<Eigen::VectorXd> points);
