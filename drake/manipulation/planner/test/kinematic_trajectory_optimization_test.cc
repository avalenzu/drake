#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <gtest/gtest.h>

namespace drake {
namespace manipulation {
namespace planner {
namespace {

using drake::multibody::MultibodyTree;

GTEST_TEST(KinematicTrajectoryOptimizationTest, ConstructorTest) {
  // Test that the constructor works and that the expected ports are
  // present.
  MultibodyTree<double> tree;
  EXPECT_NO_THROW(KinematicTrajectoryOptimization(tree, 0));
}
}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
