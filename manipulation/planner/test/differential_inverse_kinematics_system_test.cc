#include "drake/manipulation/planner/differential_inverse_kinematics_system.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

using examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities;
using systems::DiagramBuilder;

const char kEndEffectorFrameName[] = "iiwa_frame_ee";

std::unique_ptr<RigidBodyTree<double>> BuildTree() {
  const std::string iiwa_absolute_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf");
  manipulation::util::WorldSimTreeBuilder<double> tree_builder{};
  tree_builder.StoreModel("robot", iiwa_absolute_path);
  tree_builder.AddFixedModelInstance("robot", Vector3<double>::Zero());
  return tree_builder.Build();
}

GTEST_TEST(DifferentialInverseKinematicsSystemTest, ConstructorTest) {
  DiagramBuilder<double> builder;
  auto dut = builder.AddSystem<DifferentialInverseKinematicsSystem>(
      BuildTree(), kEndEffectorFrameName);
  auto robot{BuildTree()};
  auto sys = builder.Build();
  auto context = sys->CreateDefaultContext();

  // Check default parameter values
  EXPECT_TRUE(CompareMatrices(dut->nominal_joint_position(*context),
                              robot->getZeroConfiguration()));
  ASSERT_TRUE(dut->JointPositionLimits(*context));
  EXPECT_TRUE(CompareMatrices(dut->JointPositionLimits(*context)->first,
                              robot->joint_limit_min));
  EXPECT_TRUE(CompareMatrices(dut->JointPositionLimits(*context)->second,
                              robot->joint_limit_max));
  ASSERT_FALSE(dut->JointVelocityLimits(*context));
  ASSERT_FALSE(dut->JointAccelerationLimits(*context));
  EXPECT_TRUE(CompareMatrices(dut->EndEffectorVelocityGain(*context),
                              VectorX<double>::Ones(6)));
  EXPECT_EQ(dut->Timestep(*context), 1);
  EXPECT_EQ(&dut->end_effector_frame(),
            dut->robot().findFrame(kEndEffectorFrameName).get());
}
}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
