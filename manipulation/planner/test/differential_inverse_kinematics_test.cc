#include "drake/manipulation/planner/differential_inverse_kinematics.h"

#include <memory>
#include <random>
#include <string>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace planner {

std::ostream& operator<<(std::ostream& os,
                         const DifferentialInverseKinematicsStatus value) {
  switch (value) {
    case (DifferentialInverseKinematicsStatus::kSolutionFound):
      return os << "Solution found.";
    case (DifferentialInverseKinematicsStatus::kNoSolutionFound):
      return os << "No solution found.";
    case (DifferentialInverseKinematicsStatus::kStuck):
      return os << "Stuck!";
  }
  DRAKE_ABORT();
}
namespace {

using examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities;

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

GTEST_TEST(DifferentialInverseKinematicsTest, PositiveTest) {
  std::unique_ptr<RigidBodyTree<double>> tree = BuildTree();
  std::default_random_engine rand{4};
  VectorX<double> q_nominal = tree->getZeroConfiguration();
  VectorX<double> q = tree->getRandomConfiguration(rand);
  const KinematicsCache<double> cache0 = tree->doKinematics(q);
  std::shared_ptr<RigidBodyFrame<double>> frame_E =
      tree->findFrame(kEndEffectorFrameName);
  auto V_WE = (Vector6<double>() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();
  double dt = 1e-3;

  const int num_velocities{tree->get_num_velocities()};
  auto v_last = VectorX<double>::Zero(num_velocities);
  std::pair<VectorX<double>, VectorX<double>> q_bounds = {
      tree->joint_limit_min, tree->joint_limit_max};
  std::pair<VectorX<double>, VectorX<double>> v_bounds{
      -get_iiwa_max_joint_velocities(), get_iiwa_max_joint_velocities()};
  std::pair<VectorX<double>, VectorX<double>> vd_bounds{
      VectorX<double>::Constant(num_velocities, -40),
      VectorX<double>::Constant(num_velocities, 40)};
  double unconstrained_dof_v_limit{0.6};

  DifferentialInverseKinematicsResult function_result =
      DoDifferentialInverseKinematics(*tree, cache0, *frame_E, V_WE, dt,
                                      q_nominal, v_last, q_bounds, v_bounds,
                                      vd_bounds, unconstrained_dof_v_limit);
  DifferentialInverseKinematicsStatus function_status{function_result.status};
  drake::log()->info("function_status = {}", function_status);

  DifferentialInverseKinematics diff_ik(BuildTree(), kEndEffectorFrameName);
  diff_ik.SetJointVelocityLimits(v_bounds);
  diff_ik.SetJointAccelerationLimits(vd_bounds);
  diff_ik.set_unconstrained_degrees_of_freedom_velocity_limit(
      unconstrained_dof_v_limit);
  DifferentialInverseKinematicsResult object_result =
      diff_ik.ComputeJointVelocities(q, v_last, V_WE, dt);

  ASSERT_TRUE(function_result.joint_velocities != nullopt);
  ASSERT_TRUE(object_result.joint_velocities != nullopt);

  drake::log()->info("function_result.joint_velocities = {}",
                     function_result.joint_velocities->transpose());
  const KinematicsCache<double> cache1 =
      tree->doKinematics(q, function_result.joint_velocities.value());

  Vector6<double> V_WE_actual =
      tree->CalcFrameSpatialVelocityInWorldFrame(cache1, *frame_E);
  drake::log()->info("V_WE_actual = {}", V_WE_actual.transpose());
  drake::log()->info("V_WE = {}", V_WE.transpose());

  const double velocity_tolerance{1e-7};
  EXPECT_TRUE(CompareMatrices(V_WE_actual.normalized(), V_WE.normalized(),
                              velocity_tolerance));
  ASSERT_EQ(function_result.joint_velocities->size(), num_velocities);
  for (int i = 0; i < num_velocities; ++i) {
    EXPECT_GE(q(i) + dt * (*function_result.joint_velocities)(i),
              q_bounds.first(i));
    EXPECT_LE(q(i) + dt * (*function_result.joint_velocities)(i),
              q_bounds.second(i));
    EXPECT_GE((*function_result.joint_velocities)(i), v_bounds.first(i));
    EXPECT_LE((*function_result.joint_velocities)(i), v_bounds.second(i));
    EXPECT_GE((*function_result.joint_velocities)(i)-v_last(i),
              dt * vd_bounds.first(i) - velocity_tolerance);
    EXPECT_LE((*function_result.joint_velocities)(i)-v_last(i),
              dt * vd_bounds.second(i) + velocity_tolerance);
  }

  EXPECT_TRUE(CompareMatrices(*object_result.joint_velocities,
                              *function_result.joint_velocities,
                              velocity_tolerance));
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
