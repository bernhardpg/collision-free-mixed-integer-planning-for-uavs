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
	// Build model diagram 
	// **********
	
	DRAKE_DEMAND(FLAGS_simulation_time > 0);

	// Setup diagram
	drake::systems::DiagramBuilder<double> builder;
	auto [plant, scene_graph] =
        drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

	// Add obstacles from file
	drake::multibody::Parser parser(&plant, &scene_graph);
	auto obstacles = parser.AddModelFromFile(
        drake::FindResourceOrThrow("drake/examples/quadrotor/office.urdf"));
	plant.WeldFrames(
			plant.world_frame(), plant.GetFrameByName("wall1", obstacles));

	// Load quadrotor model
	auto quadrotor_plant = builder
		.AddSystem<drake::examples::quadrotor::QuadrotorPlant<double>>(
				m, arm_length, inertia, 1, 0.0245
				);
	quadrotor_plant->set_name("quadrotor");
	// Add quadrotor geometry
	drake::examples::quadrotor::QuadrotorGeometry::AddToBuilder(
      &builder, quadrotor_plant->get_output_port(0), &scene_graph);

	// Create LQR controller
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 3.0, -2.0, 1.0).
      finished())};
	auto lqr_controller = builder.AddSystem(
			drake::examples::quadrotor::StabilizingLQRController(quadrotor_plant, kNominalPosition)
			);
  lqr_controller->set_name("lqr_controller");

	// Connect controller and quadrotor
	builder.Connect(quadrotor_plant->get_output_port(0), lqr_controller->get_input_port());
  builder.Connect(lqr_controller->get_output_port(), quadrotor_plant->get_input_port(0));

	plant.Finalize();
	// Connect to 3D visualization 
	drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
	drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant);

	auto diagram = builder.Build();


	// ********
	// Simulate
	// ********
	


	drake::systems::Simulator<double> simulator(*diagram);

	// Initial conditions
	Eigen::VectorX<double> x0 = Eigen::VectorX<double>::Zero(12);
	//x0 = Eigen::VectorX<double>::Random(12);
	x0 << 0,1,2,
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

/*


	auto diagram = builder.Build();*/


	//auto logger = builder.AddSystem<drake::systems::SignalLogger<double>>(4);

	// Connect controller and quadrotor
	//builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
	//builder.Connect(controller->get_output_port(), logger->get_input_port());


	// *********
	// Simulate
	// *********

	/*// Initial conditions
	Eigen::VectorX<double> x0 = Eigen::VectorX<double>::Zero(12);
	x0 = Eigen::VectorX<double>::Random(12);

	// Simulation
	drake::systems::Simulator<double> simulator(*diagram);
	auto diagram_context = diagram->CreateDefaultContext();

	simulator.get_mutable_context().get_mutable_continuous_state_vector().SetFromVector(x0);

	simulator.Initialize();
	simulator.set_target_realtime_rate(1.0);

	// The following accuracy is necessary for the example to satisfy its
	// ending state tolerances.
	simulator.get_mutable_integrator().set_target_accuracy(5e-5);
	simulator.AdvanceTo(7.0); // seconds

  // Goal state verification.
	const drake::systems::Context<double>& context = simulator.get_context();
	const drake::systems::ContinuousState<double>& state = context.get_continuous_state();
	const Eigen::VectorX<double>& position_vector = state.CopyToVector();
	*/

}
