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
namespace {

using examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities;

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
  std::default_random_engine rand{1235};
  VectorX<double> q_nominal = tree->getZeroConfiguration();
  VectorX<double> q = tree->getRandomConfiguration(rand);
  const KinematicsCache<double> cache0 = tree->doKinematics(q);
  std::shared_ptr<RigidBodyFrame<double>> frame_E =
      tree->findFrame("iiwa_frame_ee");
  auto V_WE = (Vector6<double>() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();
  double dt = 1e-3;

  auto v_last = VectorX<double>::Zero(tree->get_num_velocities());
  std::pair<VectorX<double>, VectorX<double>> q_bounds = {
      tree->joint_limit_min, tree->joint_limit_max};
  std::pair<VectorX<double>, VectorX<double>> v_bounds{
      -get_iiwa_max_joint_velocities(), get_iiwa_max_joint_velocities()};
  auto unconstrained_dof_v_limit = Eigen::VectorXd::Constant(1, 0.6);

  optional<VectorX<double>> v = DifferentialInverseKinematics(
      *tree, cache0, *frame_E, V_WE, dt, q_nominal, v_last, q_bounds, v_bounds,
      unconstrained_dof_v_limit);

  ASSERT_TRUE(v != nullopt);

  drake::log()->info("v = {}", v->transpose());
  const KinematicsCache<double> cache1 = tree->doKinematics(q, v.value());

  Vector6<double> V_WE_actual =
      tree->CalcFrameSpatialVelocityInWorldFrame(cache1, *frame_E);
  drake::log()->info("V_WE_actual = {}", V_WE_actual.transpose());
  drake::log()->info("V_WE = {}", V_WE.transpose());

  const double velocity_tolerance{1e-7};
  EXPECT_TRUE(CompareMatrices(V_WE_actual.normalized(), V_WE.normalized(),
                              velocity_tolerance));
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
