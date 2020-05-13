#include "simulate/simulate.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");

DrakeSimulation::DrakeSimulation(
			double m,
			double arm_length,
			Eigen::Matrix3d inertia,
			double k_f,
			double k_m
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
	auto obstacle_model = parser.AddModelFromFile("obstacles.urdf");
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

std::vector<Eigen::Matrix3Xd> DrakeSimulation::get_obstacles()
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
	std::vector<Eigen::Matrix3Xd> obstacles = geometry::getObstaclesVertices(&query_object, &inspector, obstacle_geometries);

	return obstacles;
}

void DrakeSimulation::add_controller_tvlqr()
{
	auto tvlqr_constructor = controller::ControllerTVLQR(m_, arm_length_, inertia_, k_f_, k_m_);
	controller_tvlqr_ = builder_.AddSystem(
			tvlqr_constructor.construct_drake_controller(0.0, FLAGS_simulation_time, 0.01)
			);
	controller_tvlqr_->set_name("TVLQR");

	builder_.Connect(quadrotor_plant_->get_output_port(0), controller_tvlqr_->get_input_port());
	builder_.Connect(controller_tvlqr_->get_output_port(), quadrotor_plant_->get_input_port(0));
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
	simulator.AdvanceTo(FLAGS_simulation_time); // seconds
}


void simulate()
{
	std::cout << "Running drake simulation" << std::endl;
	DRAKE_DEMAND(FLAGS_simulation_time > 0);

	// *******
	// Model parameters
	// *******

	Eigen::Matrix3d inertia;
	inertia << 0.07, 0, 0,
				0, 0.08, 0,
				0, 0, 0.12; // From .urdf file
	double m = 2.856;
	double arm_length = 0.2;
	double k_f_ = 1.0;
	double k_m_ = 0.0245;

	// Build the simulation first to get the obstacles
	auto obst_sim = DrakeSimulation(m, arm_length, inertia, k_f_, k_m_);
	obst_sim.build_quadrotor_diagram();
	auto obstacles = obst_sim.get_obstacles();

	// Calculate trajectory
	find_trajectory(obstacles);

	// Initial conditions
	Eigen::VectorX<double> x0 = Eigen::VectorX<double>::Zero(12);
	x0 = Eigen::VectorX<double>::Random(12);
	/*
	x0 << 0,0,1,
				0,0,0,
				0,0,0,
				0,0,0;*/

	// Build the real simulation
	auto sim = DrakeSimulation(m, arm_length, inertia, k_f_, k_m_);
	sim.add_controller_tvlqr();
	sim.connect_to_drake_visualizer();
	sim.build_quadrotor_diagram();
	sim.run_simulation(x0);
}

void find_trajectory(std::vector<Eigen::Matrix3Xd> obstacles)
{
	// Get convex safe regions
	trajopt::SafeRegions safe_regions(3);
	// Matches 'ground' object in obstacles.urdf
	safe_regions.set_bounds(-5, 5, -2.5, 12.5, 0, 2);
	safe_regions.set_obstacles(obstacles);

	// Calc regions from seed points to save time
	std::vector<Eigen::Vector3d> seed_points;
	seed_points.push_back(Eigen::Vector3d(1,1,0.5));
	seed_points.push_back(Eigen::Vector3d(-4,6,0.5));
	seed_points.push_back(Eigen::Vector3d(-2,3.5,0.5));
	seed_points.push_back(Eigen::Vector3d(-2,6,0.5));
	seed_points.push_back(Eigen::Vector3d(0,5,0.5));
	safe_regions.calc_safe_regions_from_seedpoints(seed_points);

	//safe_regions.calc_safe_regions_auto(8);
	auto safe_region_As = safe_regions.get_As();
	auto safe_region_bs = safe_regions.get_bs();

	std::cout << "Calculated safe regions" << std::endl;
	//plot_3d_obstacles_footprints(obstacles);
	//plot_3d_regions_footprint(safe_regions.get_polyhedrons());

	// ********************
	// Calculate trajectory
	// ********************

	int num_vars = 3;
	int num_traj_segments = 8;
	int degree = 3;
	int cont_degree = 2;

	Eigen::VectorX<double> init_pos(num_vars);
	Eigen::VectorX<double> final_pos(num_vars);
	init_pos << 0.0, 0.0, 1.0;
	final_pos << 0.0, 6.5, 1.0;

	auto traj_3rd_deg = trajopt::MISOSProblem(
			num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos
			);
	traj_3rd_deg.add_convex_regions(safe_region_As, safe_region_bs);
	traj_3rd_deg.create_region_binary_variables();
	traj_3rd_deg.generate();
	Eigen::MatrixX<int> safe_region_assignments = traj_3rd_deg.get_region_assignments();
	std::cout << "Found 3rd order trajectory" << std::endl;

	// Create trajectory with degree 5 with fixed region constraints
	degree = 5;
	cont_degree = 4;

	auto traj = trajopt::MISOSProblem(num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos);
	traj.add_convex_regions(safe_region_As, safe_region_bs);
	traj.add_safe_region_assignments(safe_region_assignments);
	traj.generate();
	std::cout << "Found 5th order trajectory" << std::endl;

	// TODO cleanup
	// Visualize trajectory

	// Publish to visualizer
  std::vector<std::string> names;
  std::vector<Eigen::Isometry3d> poses;
  for (double t = 0.0; t < (double)num_traj_segments; t += 0.1) {
    names.push_back("X" + std::to_string(int(t * 100)));
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = traj.eval(t);
    poses.push_back(pose);
  }
	drake::lcm::DrakeLcm lcm;
  PublishFramesToLcm("DRAKE_DRAW_TRAJECTORY", poses, names, &lcm);

}
