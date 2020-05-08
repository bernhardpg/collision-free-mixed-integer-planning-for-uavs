#pragma once

#include <iostream>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include <drake/systems/primitives/signal_logger.h>

#include "drake/common/is_approx_equal_abstol.h"

#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/examples/quadrotor/quadrotor_geometry.h"

void simulate();
