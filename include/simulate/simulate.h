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
#include "trajopt/safe_regions.h"
#include "trajopt/MISOSProblem.h"
#include "controller/tvlqr.h"
#include "plot/plotter.h"
#include "simulate/publish_trajectory.h"

// TODO add namespace


class DrakeSimulation
{
	public:
		DrakeSimulation(
				double m, double arm_length, Eigen::Matrix3d inertia
				);

	void build_quadrotor_diagram();
	void connect_to_drake_visualizer();
	std::vector<Eigen::Matrix3Xd> get_obstacles();
	void add_controller_tvlqr();
	void run_simulation(Eigen::VectorXd x0);

	private:
		double m_;
		double arm_length_;
		Eigen::MatrixXd inertia_;

		drake::systems::DiagramBuilder<double> builder_;
		drake::geometry::SceneGraph<double>* scene_graph_;
		drake::multibody::MultibodyPlant<double>* plant_; // TODO rename
		std::unique_ptr<drake::systems::Diagram<double>> diagram_;

		drake::examples::quadrotor::QuadrotorPlant<double>* quadrotor_plant_;
		controller::DrakeControllerTVLQR* controller_tvlqr_;
};

void simulate();
void find_trajectory(std::vector<Eigen::Matrix3Xd> obstacles);

