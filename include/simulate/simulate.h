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
#include "unistd.h"

// TODO add namespace


class DrakeSimulation
{
	public:
		DrakeSimulation(
				double m,
				double arm_length,
				Eigen::Matrix3d inertia,
				double k_f,
				double k_m,
				std::string obstacle_model_path
				);

		void build_quadrotor_diagram();
		void connect_to_drake_visualizer();
		void retrieve_obstacles();
		void add_controller_tvlqr(trajopt::MISOSProblem* traj);
		void run_simulation(Eigen::VectorXd x0);
		void calculate_safe_regions(int num_safe_regions);

		std::vector<Eigen::MatrixXd> get_safe_regions_As();
		std::vector<Eigen::VectorXd> get_safe_regions_bs();

	private:
		double m_;
		double arm_length_;
		Eigen::MatrixXd inertia_;
		double k_f_;
		double k_m_;

		drake::systems::DiagramBuilder<double> builder_;
		drake::geometry::SceneGraph<double>* scene_graph_;
		drake::multibody::MultibodyPlant<double>* plant_; // TODO rename
		std::unique_ptr<drake::systems::Diagram<double>> diagram_;

		drake::examples::quadrotor::QuadrotorPlant<double>* quadrotor_plant_;
		controller::DrakeControllerTVLQR* controller_tvlqr_;

		std::vector<Eigen::Matrix3Xd> obstacles_;
		std::vector<Eigen::MatrixXd> safe_region_As_;
		std::vector<Eigen::VectorXd> safe_region_bs_;
};

void simulate();
void find_trajectory(
		Eigen::Vector3d init_pos,
		Eigen::Vector3d final_pos,
		int num_traj_segments,
		std::vector<Eigen::MatrixXd> safe_region_As,
		std::vector<Eigen::VectorXd> safe_region_bs,
		trajopt::MISOSProblem* traj
		);
void publish_traj_to_visualizer(trajopt::MISOSProblem* traj);
