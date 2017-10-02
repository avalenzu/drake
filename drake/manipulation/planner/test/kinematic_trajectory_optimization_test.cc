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
  KinematicTrajectoryOptimization program{
      1 /*num_positions*/, 4 /*num_control_points*/,
      -1 /*num_evaluation_points*/, 4 /*spine_order*/};
  program.AddLinearConstraint(program.position(0)(0) == 0);
  program.AddLinearConstraint(program.position(1)(0) == 3);
  solvers::SolutionResult result = program.Solve();
  EXPECT_EQ(result, solvers::kSolutionFound);

  const int kNumPlottingPoints(1000);
  PiecewisePolynomialTrajectory solution_trajectory{
      program.ReconstructPositionTrajectory()};
  const VectorX<double> x{VectorX<double>::LinSpaced(
      kNumPlottingPoints, solution_trajectory.get_start_time(),
      solution_trajectory.get_end_time())};
  VectorX<double> solution_values(x.size());
  for (int i = 0; i < kNumPlottingPoints; ++i) {
    solution_values(i) = solution_trajectory.value(x(i))(0);
  }
  CallMatlab("plot", x, solution_values, "LineWidth", 2.0);
  std::string plot_title("Solution Result:" + std::to_string(result) +
                         ", Solver: " + program.GetSolverId()->name());
  CallMatlab("title", plot_title);
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
