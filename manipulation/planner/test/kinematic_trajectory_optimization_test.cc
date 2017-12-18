#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using drake::systems::trajectory_optimization::MultipleShooting;

namespace drake {
namespace manipulation {
namespace planner {
namespace {

GTEST_TEST(KinematicTrajectoryOptimizationTest, ConstructorTest) {
  // Test that the constructor works and that the expected ports are
  // present.
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  RigidBodyTree<double> tree{};
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(kModelPath, multibody::joints::kFixed, nullptr, &tree);
  const int kNumSampleTimes{2};

  KinematicTrajectoryOptimization prog{tree, kNumSampleTimes};
  EXPECT_EQ(prog.num_time_samples(), kNumSampleTimes);
}

GTEST_TEST(KinematicTrajectoryOptimizationTest, SolveTest) {
  // Test that the constructor works and that the expected ports are
  // present.
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  RigidBodyTree<double> tree{};
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(kModelPath, multibody::joints::kFixed, nullptr, &tree);
  const int kNumSampleTimes{2};

  KinematicTrajectoryOptimization kin_traj_opt{tree, kNumSampleTimes};

  MultipleShooting* prog = kin_traj_opt.mutable_prog();

  // x[0] = 0.
  VectorX<double> x0{VectorX<double>::Zero(prog->initial_state().size())};
  prog->AddLinearConstraint(prog->initial_state() == x0);

  const solvers::VectorXDecisionVariable& u = prog->input();
  prog->AddRunningCost( u.transpose() * u );

  solvers::SolutionResult result{prog->Solve()};

  EXPECT_EQ(result, solvers::kSolutionFound);
  for (int i = 0; i < (kNumSampleTimes - 1); ++i) {
    for (int j = 0; j < prog->state().size(); ++j) {
      EXPECT_EQ(prog->GetSolution(prog->state(i)[j]), 0.0);
    }
    for (int j = 0; j < prog->input().size(); ++j) {
      EXPECT_EQ(prog->GetSolution(prog->input(i)[j]), 0.0);
    }
  }


}
}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
