#include "simulate/simulate.h"

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

	// ********
	// Set up quadrotor system
	// ********

	drake::systems::DiagramBuilder<double> builder;

	auto quadrotor = builder
		.AddSystem<drake::examples::quadrotor::QuadrotorPlant<double>>(
				m, arm_length, inertia, 1, 0.0245
				);
	quadrotor->set_name("quadrotor");

	//auto logger = builder.AddSystem<drake::systems::SignalLogger<double>>(4);

	// Connect controller and quadrotor
	//builder.Connect(quadrotor->get_output_port(0), controller->get_input_port());
	//builder.Connect(controller->get_output_port(), logger->get_input_port());



	// // TODO REMOVE
	// Set up LQR controller for test
  const Eigen::Vector3d kNominalPosition{((Eigen::Vector3d() << 0.0, 0.0, 1.0).
      finished())};

	auto lqr_controller = builder.AddSystem(
			drake::examples::quadrotor::StabilizingLQRController(quadrotor, kNominalPosition)
			);
  lqr_controller->set_name("lqr_controller");

	// Connect controller and quadrotor
	builder.Connect(quadrotor->get_output_port(0), lqr_controller->get_input_port());
  builder.Connect(lqr_controller->get_output_port(), quadrotor->get_input_port(0));

	// *********
	// Set up visualization
	// *********

  auto scene_graph = builder.AddSystem<drake::geometry::SceneGraph>();
	drake::examples::quadrotor::QuadrotorGeometry::AddToBuilder(
      &builder, quadrotor->get_output_port(0), scene_graph);
	drake::geometry::ConnectDrakeVisualizer(&builder, *scene_graph);

	auto diagram = builder.Build();


	// *********
	// Simulate
	// *********

	// Initial conditions
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

}
