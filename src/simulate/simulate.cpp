#include "simulate/simulate.h"

DEFINE_double(target_realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, 10, "How long to simulate the pendulum");
DEFINE_double(max_time_step, 1.0e-3,
              "Simulation time step used for integrator.");

void simulate()
{
	std::cout << "Running drake simulation" << std::endl;

	// *******
	// Model parameters
	// *******

	Eigen::Matrix3d inertia;
	inertia << 0.07, 0, 0,
				0, 0.08, 0,
				0, 0, 0.12; // From .urdf file
	double m = 2.856;
	double arm_length = 0.2;

	// **********
	// Build diagram 
	// **********
	
	DRAKE_DEMAND(FLAGS_simulation_time > 0);

	// Setup diagram
	drake::systems::DiagramBuilder<double> builder;
	auto [plant, scene_graph] =
        drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

	// Add obstacles from file
	drake::multibody::Parser parser(&plant);
	auto obstacle_model = parser.AddModelFromFile("obstacles.urdf");
	plant.WeldFrames(
			plant.world_frame(), plant.GetFrameByName("ground", obstacle_model));
	plant.Finalize();

	// Load quadrotor model
	auto quadrotor_plant = builder
		.AddSystem<drake::examples::quadrotor::QuadrotorPlant<double>>(
				m, arm_length, inertia, 1, 0.0245
				);
	quadrotor_plant->set_name("quadrotor");
	drake::examples::quadrotor::QuadrotorGeometry::AddToBuilder(
      &builder, quadrotor_plant->get_output_port(0), &scene_graph);

	// Create LQR controller
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};
	auto lqr_controller = builder.AddSystem(
			drake::examples::quadrotor::StabilizingLQRController(quadrotor_plant, kNominalPosition)
			);
  lqr_controller->set_name("lqr_controller");

	// Connect controller and quadrotor
	builder.Connect(quadrotor_plant->get_output_port(0), lqr_controller->get_input_port());
  builder.Connect(lqr_controller->get_output_port(), quadrotor_plant->get_input_port(0));

	// Connect to 3D visualization 
	drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
	//drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant);

	auto diagram = builder.Build();

	// *****
	// Get obstacle vertices 
	// *****

	// Get query_object to pass geometry queries to (needs root context from diagram)
	const auto diagram_context = diagram->CreateDefaultContext();
	const auto& query_object =
			scene_graph.get_query_output_port()
			.Eval<drake::geometry::QueryObject<double>>(
						scene_graph.GetMyContextFromRoot(*diagram_context)
					);
	const auto& inspector = scene_graph.model_inspector();

	auto obstacle_geometries = geometry::getObstacleGeometries(&plant);
	std::vector<Eigen::Matrix3Xd> obstacles = geometry::getObstaclesVertices(&query_object, &inspector, obstacle_geometries);

	// ****
	// Get convex regions using IRIS
	// ****

	// Create bounding box
	// Matches 'ground' object in obstacles.urdf
	Eigen::MatrixXd A_bounds(6,3);
	A_bounds << -1, 0, 0,
							0, -1, 0,
							0, 0, -1,
							1, 0, 0,
							0, 1, 0,
							0, 0, 1;

	Eigen::VectorXd b_bounds(6);
	b_bounds << 5, 2.5, 0, 5, 12.5, 2;
	iris::Polyhedron bounds(A_bounds,b_bounds);

	// Initialize IRIS problem
	iris::IRISProblem iris_problem(3);
  iris::IRISOptions options;
	iris_problem.setBounds(bounds);

	for (auto obstacle : obstacles)
		iris_problem.addObstacle(obstacle);

	// Add seedpoints
	std::vector<Eigen::Vector3d> seed_points;
	seed_points.push_back(Eigen::Vector3d(1,1,0.5));
	seed_points.push_back(Eigen::Vector3d(-4,6,0.5));
	seed_points.push_back(Eigen::Vector3d(-2,6,0.5));
	seed_points.push_back(Eigen::Vector3d(0,5,0.5));

	// Obtain convex regions
	std::vector<iris::Polyhedron> convex_polygons;
	for (auto seed_point : seed_points)
	{
		iris_problem.setSeedPoint(seed_point);
		iris::IRISRegion region = inflate_region(iris_problem, options);
		convex_polygons.push_back(region.getPolyhedron());
	}

	plot_convex_regions_footprint(convex_polygons);

	std::vector<Eigen::MatrixXd> obstacle_As;
	std::vector<Eigen::VectorXd> obstacle_bs;
	for (int i = 0; i < convex_polygons.size(); ++i)
	{
		// Matrix containing one convex region
		obstacle_As.push_back(convex_polygons[i].getA());
		obstacle_bs.push_back(convex_polygons[i].getB());
	}

	// ********************
	// Calculate trajectory
	// ********************

	int num_vars = 3;
	int num_traj_segments = 8;
	int degree = 3;
	int cont_degree = 2;
	Eigen::VectorX<double> init_pos(num_vars);
	init_pos << 0.0, 0.0, 1.0;

	Eigen::VectorX<double> final_pos(num_vars);
	final_pos << 0.0, 6.5, 1.0;

	auto traj = trajopt::MISOSProblem(num_traj_segments, num_vars, degree, cont_degree, init_pos, final_pos);

	traj.add_region_constraint(obstacle_As[0], obstacle_bs[0], 0);
	traj.add_region_constraint(obstacle_As[0], obstacle_bs[0], 1);
	traj.add_region_constraint(obstacle_As[1], obstacle_bs[1], 2);
	traj.add_region_constraint(obstacle_As[1], obstacle_bs[1], 3);
	traj.add_region_constraint(obstacle_As[2], obstacle_bs[2], 4);
	traj.add_region_constraint(obstacle_As[2], obstacle_bs[2], 5);
	traj.add_region_constraint(obstacle_As[3], obstacle_bs[3], 6);
	traj.add_region_constraint(obstacle_As[3], obstacle_bs[3], 7);

	traj.generate();
	return;

	// ********
	// Simulate
	// ********

	drake::systems::Simulator<double> simulator(*diagram);

	// Initial conditions
	Eigen::VectorX<double> x0 = Eigen::VectorX<double>::Zero(12);
	//x0 = Eigen::VectorX<double>::Random(12);
	x0 << 0,0,2,
				0,0,0,
				0,0,0,
				0,0,0;

	// To set initial values for the simulation:
	// * Get the Diagram's context.
	// * Get the part of the Diagram's context associated with particle_plant.
	// * Get the part of the particle_plant's context associated with state.
	// * Fill the state with initial values.
	drake::systems::Context<double>& simulator_context = simulator.get_mutable_context();
	drake::systems::Context<double>& quadrotor_plant_context =
			diagram->GetMutableSubsystemContext(*quadrotor_plant, &simulator_context);

	quadrotor_plant_context.get_mutable_continuous_state_vector().SetFromVector(x0);

	simulator.Initialize();
	simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
	simulator.AdvanceTo(FLAGS_simulation_time); // seconds

	//auto logger = builder.AddSystem<drake::systems::SignalLogger<double>>(4);
}
