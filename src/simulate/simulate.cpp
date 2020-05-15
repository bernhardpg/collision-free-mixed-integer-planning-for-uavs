#include "simulate/simulate.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 13, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");

DrakeSimulation::DrakeSimulation(
			double m,
			double arm_length,
			Eigen::Matrix3d inertia,
			double k_f,
			double k_m,
			std::string obstacle_model_path
		)
	:
		m_(m),
		arm_length_(arm_length),
		inertia_(inertia),
		k_f_(k_f),
		k_m_(k_m)
{
	auto pair = drake::multibody::AddMultibodyPlantSceneGraph(&builder_, 0.0);
	scene_graph_ = &pair.scene_graph;
	plant_ = &pair.plant;

	// Add obstacles from file
	drake::multibody::Parser parser(plant_, scene_graph_);
	auto obstacle_model = parser.AddModelFromFile(obstacle_model_path);
	plant_->WeldFrames(
			plant_->world_frame(), plant_->GetFrameByName("ground", obstacle_model));
	plant_->Finalize();

	// Load quadrotor model
	quadrotor_plant_ = builder_
		.AddSystem<drake::examples::quadrotor::QuadrotorPlant<double>>(
				m_, arm_length_, inertia_, k_f_, k_m_
				);
	quadrotor_plant_->set_name("quadrotor");
	drake::examples::quadrotor::QuadrotorGeometry::AddToBuilder(
      &builder_, quadrotor_plant_->get_output_port(0), scene_graph_);
}

void DrakeSimulation::build_quadrotor_diagram()
{
	diagram_ = builder_.Build();
}

void DrakeSimulation::connect_to_drake_visualizer()
{
	// Connect to 3D visualization
	drake::geometry::ConnectDrakeVisualizer(&builder_, *scene_graph_);
	//drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant);
}

void DrakeSimulation::retrieve_obstacles()
{
	// Get query_object to pass geometry queries to (needs root context from diagram)
	const auto diagram_context = diagram_->CreateDefaultContext();

	const auto& query_object =
			scene_graph_->get_query_output_port()
			.Eval<drake::geometry::QueryObject<double>>(
						scene_graph_->GetMyContextFromRoot(*diagram_context)
					);
	const auto& inspector = scene_graph_->model_inspector();

	// TODO refactor geometry functions into simulation?
	auto obstacle_geometries = geometry::getObstacleGeometries(plant_);
	obstacles_ = geometry::getObstaclesVertices(&query_object, &inspector, obstacle_geometries);
}

void DrakeSimulation::add_controller_tvlqr(trajopt::MISOSProblem* traj)
{
	auto tvlqr_constructor =
		controller::ControllerTVLQR(m_, arm_length_, inertia_, k_f_, k_m_);
	controller_tvlqr_ = builder_.AddSystem(
			tvlqr_constructor
			.construct_drake_controller(0.01, traj)
			);
	controller_tvlqr_->set_name("TVLQR");

	builder_.Connect(
			quadrotor_plant_->get_output_port(0), controller_tvlqr_->get_input_port()
			);
	builder_.Connect(
			controller_tvlqr_->get_output_port(), quadrotor_plant_->get_input_port(0)
			);
}

void DrakeSimulation::run_simulation(Eigen::VectorXd x0)
{
	auto simulator = drake::systems::Simulator<double>(*diagram_);

	// To set initial values for the simulation:
	// * Get the Diagram's context.
	// * Get the part of the Diagram's context associated with particle_plant.
	// * Get the part of the particle_plant's context associated with state.
	// * Fill the state with initial values.
	drake::systems::Context<double>& simulator_context = simulator.get_mutable_context();
	drake::systems::Context<double>& quadrotor_plant_context =
			diagram_->GetMutableSubsystemContext(*quadrotor_plant_, &simulator_context);

	quadrotor_plant_context.get_mutable_continuous_state_vector().SetFromVector(x0);

	simulator.Initialize();
	simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
	simulator.AdvanceTo(0.1); // seconds
	sleep(10);
	simulator.AdvanceTo(FLAGS_simulation_time); // seconds
}

void DrakeSimulation::calculate_safe_regions(int num_safe_regions)
{
	// Get convex safe regions
	trajopt::SafeRegions safe_regions(3);
	// TODO get from ground object
	safe_regions.set_bounds(-5, 5, -2.5, 12.5, 0, 2); // Matches 'ground' object in obstacles.urdf
	//simple: safe_regions.set_bounds(-5, 5, -2.5, 12.5, 0, 2); // Matches 'ground' object in obstacles.urdf
	safe_regions.set_obstacles(obstacles_);
	safe_regions.calc_safe_regions_auto(num_safe_regions);
	safe_region_As_ = safe_regions.get_As();
	safe_region_bs_ = safe_regions.get_bs();

	// TODO hardcoded in bottom and top for plot
	plot_3d_obstacles_footprints(obstacles_, 0);
	plot_3d_regions_footprint(safe_regions.get_polyhedrons(), 0);
}

std::vector<Eigen::MatrixXd> DrakeSimulation::get_safe_regions_As()
{
	return safe_region_As_;
}

std::vector<Eigen::VectorXd> DrakeSimulation::get_safe_regions_bs()
{
	return safe_region_bs_;
}

void simulate()
{
	DRAKE_DEMAND(FLAGS_simulation_time > 0);

	// *******
	// Model parameters
	// *******

	Eigen::Matrix3d inertia;
	/*
	inertia << 0.07, 0, 0,
				0, 0.08, 0,
				0, 0, 0.12; // From .urdf file
	double m = 2.856;
	double arm_length = 0.2;
	double k_f_ = 1.0;
	double k_m_ = 0.0245;
	*/
	// Skydio model
	inertia << 0.0015, 0, 0,  // BR
						 0, 0.0025, 0,  // BR
						 0, 0, 0.0035;
	double m = 0.775;
	double arm_length = 0.15;
	double k_f_ = 1.0;
	double k_m_ = 0.0245;

	Eigen::Vector3d init_pos(0.0, -2, 1.0);
	Eigen::Vector3d final_pos(0.0, 10, 1.0);
	int num_safe_regions = 4;

	// Build the simulation first to calculate the safe regions from the obstacles
	std::string obstacle_model_path = "models/obstacles_simple.urdf";
	auto obst_sim = DrakeSimulation(
			m, arm_length, inertia, k_f_, k_m_, obstacle_model_path
			);
	obst_sim.build_quadrotor_diagram();
	obst_sim.retrieve_obstacles();
	obst_sim.calculate_safe_regions(num_safe_regions);
	std::cout << "Calculated safe regions" << std::endl;
	auto safe_regions_As = obst_sim.get_safe_regions_As();
	auto safe_regions_bs = obst_sim.get_safe_regions_bs();

	// Calculate trajectory
	int num_vars = 3;
	int num_traj_segments = 5;
	int degree = 5;
	int cont_degree = 4;

	auto traj = trajopt::MISOSProblem(
			num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos
			);

	find_trajectory(
			init_pos, final_pos, num_traj_segments, safe_regions_As, safe_regions_bs, &traj
			);

	std::cout << "Trajectory found. Press any key to simulate\n";
	system("read");

	publish_traj_to_visualizer(&traj);

	// Initial conditions
	Eigen::VectorX<double> x0 = Eigen::VectorX<double>::Zero(12);
	x0 << init_pos,
				0,0,0,
				0,0,0,
				0,0,0;

	// Build the real simulation with the found controller
	auto sim = DrakeSimulation(
			m, arm_length, inertia, k_f_, k_m_, obstacle_model_path
			);
	sim.add_controller_tvlqr(&traj);
	sim.connect_to_drake_visualizer();
	sim.build_quadrotor_diagram();
	std::cout << "Running drake simulation" << std::endl;
	for (int i = 0; i < 10; ++i)
	{
		sim.run_simulation(x0);
	}

}

void find_trajectory(
		Eigen::Vector3d init_pos,
		Eigen::Vector3d final_pos,
		int num_traj_segments,
		std::vector<Eigen::MatrixXd> safe_region_As,
		std::vector<Eigen::VectorXd> safe_region_bs,
		trajopt::MISOSProblem* traj
		)
{
	auto traj_3rd_deg = trajopt::MISOSProblem(
			num_traj_segments, 3, 3, 2, init_pos, final_pos
			);

	traj_3rd_deg.add_convex_regions(safe_region_As, safe_region_bs);
	traj_3rd_deg.create_region_binary_variables();
	traj_3rd_deg.generate();
	Eigen::MatrixX<int> safe_region_assignments = traj_3rd_deg.get_region_assignments();
	std::cout << "Found 3rd order trajectory" << std::endl;

	// Create trajectory with degree 5 with fixed region constraints
	traj->add_convex_regions(safe_region_As, safe_region_bs);
	traj->add_safe_region_assignments(safe_region_assignments);
	traj->generate();
	std::cout << "Found 5th order trajectory" << std::endl;
}

// TODO replace num_traj_segments w end time
void publish_traj_to_visualizer(trajopt::MISOSProblem* traj)
{
	double end_time = traj->get_end_time();
  std::vector<std::string> names;
  std::vector<Eigen::Isometry3d> poses;
  for (double t = 0.0; t < end_time; t += 0.1) {
    names.push_back("X" + std::to_string(int(t * 100)));
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = traj->eval(t);
    poses.push_back(pose);
  }

	drake::lcm::DrakeLcm lcm;
  PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", poses, names, &lcm);
}
