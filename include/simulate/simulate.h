#pragma once

#include <iostream>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/geometry/geometry_visualization.h"
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/math/rigid_transform.h>

#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/examples/quadrotor/quadrotor_geometry.h"

#include "tools/geometry.h"
#include "iris/iris.h"
#include "trajopt/MISOSProblem.h"
#include "plot/plotter.h"
#include "simulate/publish_trajectory.h"

void simulate();
void find_trajectory(std::vector<Eigen::Matrix3Xd> obstacles);
