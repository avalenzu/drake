#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/manipulation/planner/kinematic_planning_problem.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

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
      1 /*num_positions*/, 2 /*num_control_points*/,
      -1 /*num_evaluation_points*/, 4 /*spine_order*/};
  program.AddLinearConstraint(program.control_points().leftCols(1)(0) == 0);
  program.AddLinearConstraint(program.control_points().rightCols(1)(0) == 1);
  solvers::SolutionResult result = program.Solve();
  EXPECT_EQ(result, solvers::kSolutionFound);
}

//TEST_F(FromKinematicPlanningProblemTest, UnconstrainedTest) {
  //KinematicTrajectoryOptimization program{
      //problem_.get(), 2 [>num_control_points*/, -1 /*num_evaluation_points<],
      //4 [>spline_order<]};
  //solvers::SolutionResult result = program.Solve();
  //EXPECT_EQ(result, solvers::kSolutionFound);
//}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
