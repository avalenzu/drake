#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/manipulation/planner/kinematic_planning_problem.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

using common::CallMatlab;

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

class FromKinematicPlanningProblemTest : public ::testing::Test {
 public:
  void SetUp() {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        FindResourceOrThrow(kIiwaUrdf), multibody::joints::kFixed, nullptr,
        tree.get());
    problem_ = std::make_unique<KinematicPlanningProblem>(std::move(tree));
  }

 protected:
  std::unique_ptr<KinematicPlanningProblem> problem_;
};

GTEST_TEST(KinematicTrajectoryOptimizationTest, UnconstrainedTest) {
  const int kNumControlPoints = 11;
  const int kSplineOrder = 4;
  const int kNumInternalIntervals{kNumControlPoints - kSplineOrder + 1};
  KinematicTrajectoryOptimization program{
      1 /*num_positions*/, kNumControlPoints,
      -1 /*num_evaluation_points*/, kSplineOrder};
  program.AddLinearConstraint(program.position(0.0)(0) == 0);
  program.AddLinearConstraint(program.velocity(0.0)(0) == 0);
  program.AddLinearConstraint(program.acceleration(0.0)(0) == 0);
  program.AddLinearConstraint(program.jerk(0.0)(0) == 0);

  program.AddLinearConstraint(program.position(0.5)(0) == 0.5);
  program.AddLinearConstraint(program.velocity(0.5)(0) == 0);
  program.AddLinearConstraint(program.acceleration(0.5)(0) == 0);
  
  program.AddLinearConstraint(program.position(1.0)(0) == 1);
  program.AddLinearConstraint(program.velocity(1.0)(0) == 0);
  program.AddLinearConstraint(program.acceleration(1.0)(0) == 0);
  program.AddLinearConstraint(program.jerk(1.0)(0) == 0);

  VectorX<double> evaluation_times = VectorX<double>::LinSpaced(2*kNumInternalIntervals+1, 0.0, 1.0);
  for (int i = 0; i < evaluation_times.size(); ++i) {
    program.AddQuadraticCost((evaluation_times(1) - evaluation_times(0))*program.jerk(evaluation_times(i))(0) *
                             program.jerk(evaluation_times(i))(0));
  }
  solvers::SolutionResult result = program.Solve();
  EXPECT_EQ(result, solvers::kSolutionFound);

  const int kNumPlottingPoints(1000);
  PiecewisePolynomialTrajectory solution_trajectory{
      program.ReconstructPositionTrajectory()};
  const VectorX<double> x{VectorX<double>::LinSpaced(
      kNumPlottingPoints, solution_trajectory.get_start_time(),
      solution_trajectory.get_end_time())};
  VectorX<double> position_values(x.size());
  VectorX<double> velocity_values(x.size());
  VectorX<double> acceleration_values(x.size());
  VectorX<double> jerk_values(x.size());
  for (int i = 0; i < kNumPlottingPoints; ++i) {
    position_values(i) = solution_trajectory.value(x(i))(0);
    velocity_values(i) = solution_trajectory.derivative(1)->value(x(i))(0);
    acceleration_values(i) = solution_trajectory.derivative(2)->value(x(i))(0);
    jerk_values(i) = solution_trajectory.derivative(3)->value(x(i))(0);
  }
  CallMatlab("subplot", 4, 1, 1);
  CallMatlab("plot", x, position_values, "LineWidth", 2.0);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Position");
  std::string plot_title("Solution Result:" + std::to_string(result) +
                         ", Solver: " + program.GetSolverId()->name());
  CallMatlab("title", plot_title);
  CallMatlab("subplot", 4, 1, 2);
  CallMatlab("plot", x, velocity_values);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Velocity");
  CallMatlab("subplot", 4, 1, 3);
  CallMatlab("plot", x, acceleration_values);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Acceleration");
  CallMatlab("subplot", 4, 1, 4);
  CallMatlab("plot", x, jerk_values);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Jerk");
}

// TEST_F(FromKinematicPlanningProblemTest, UnconstrainedTest) {
// KinematicTrajectoryOptimization program{
// problem_.get(), 2 [>num_control_points*/, -1 /*num_evaluation_points<],
// 4 [>spline_order<]};
// solvers::SolutionResult result = program.Solve();
// EXPECT_EQ(result, solvers::kSolutionFound);
//}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
