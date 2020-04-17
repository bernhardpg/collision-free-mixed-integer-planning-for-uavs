#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "plot/matplotlibcpp.h"
#include "tools/ConvexHull.h"

namespace plt = matplotlibcpp;

void plot_obstacles(std::vector<Eigen::MatrixXd> obstacles);
void plot_convex_hull(std::vector<Eigen::VectorXd> points);
void plot_region(std::vector<Eigen::VectorXd> points);
