#include "trajectory_optimization.h"
#include <Eigen/Core>


namespace trajopt
{

void program_init()
{
	std::cout << "Running program_init()" << std::endl;
	drake::solvers::MathematicalProgram	prog;

	Eigen::Matrix2d A;
	A << 1, 1, 1, 1;
	Eigen::Vector2d b;
	b << 10, 10;

	auto state = prog.NewContinuousVariables(2, "state");
	prog.AddLinearConstraint((A * state).array() >= b.array());
	prog.AddCost(state.transpose() * state);

	drake::solvers::MathematicalProgramResult result = Solve(prog);
	assert(result.is_success());
	std::cout << "Result: " << std::endl << result.GetSolution() << std::endl;

	return;
	}

}
