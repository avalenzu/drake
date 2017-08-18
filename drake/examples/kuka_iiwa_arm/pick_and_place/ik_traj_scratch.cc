/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual tree hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/robot_plan_interpolator.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/action.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/zero_order_hold.h"
#include "drake/systems/primitives/signal_logger.h"

DEFINE_double(ptol, 0.001,
              "Position tolerance");
DEFINE_string(urdf, "", "Name of urdf to load");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;

int DoMain() {
  drake::log()->set_level(spdlog::level::debug);
  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  // Adds a plant.
  const char* kModelPath =
      "drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kModelPath));

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      urdf, multibody::joints::kFixed, tree.get());

  const int kNumJoints = tree->get_num_positions();

  auto trajectory_generator_ =
      builder.AddSystem<RobotPlanInterpolator>(urdf);
  auto hold =
      builder.AddSystem<systems::ZeroOrderHold<double>>(0.1, 2 * kNumJoints);
  auto logger_ =
      builder.AddSystem<systems::SignalLogger<double>>(2 * kNumJoints);
  auto visualizer =
    builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);

  builder.Connect(trajectory_generator_->get_state_output_port(),
      hold->get_input_port());
  builder.Connect(hold->get_output_port(),
      trajectory_generator_->get_state_input_port());
  builder.Connect(trajectory_generator_->get_state_output_port(),
      logger_->get_input_port(0));
  builder.Connect(trajectory_generator_->get_state_output_port(),
      visualizer->get_input_port(0));

  std::unique_ptr<Diagram<double>> sys = builder.Build();

  auto simulator = std::make_unique<systems::Simulator<double>>(*sys);
  simulator->set_target_realtime_rate(1);

  std::vector<Isometry3<double>> X_WE_vector;
  X_WE_vector.push_back(Isometry3<double>::Identity());
  X_WE_vector.back().translation() = Vector3<double>(0.62, -0.37, 0.77);
  X_WE_vector.push_back(Isometry3<double>::Identity());
  X_WE_vector.back().translation() = Vector3<double>(0.62, -0.37, 0.57);
  X_WE_vector.push_back(Isometry3<double>::Identity());
  X_WE_vector.back().translation() = Vector3<double>(0.62, -0.37, 0.27);

  //const int kNumKnots = 2*X_WE_vector.size()-1;
  const int kNumKnots = X_WE_vector.size();

  VectorX<double> t = VectorX<double>::LinSpaced(kNumKnots, 0, kNumKnots - 1);
  MatrixX<double> q_nom = MatrixX<double>::Zero(kNumJoints, kNumKnots);
  MatrixX<double> q_seed = MatrixX<double>::Zero(kNumJoints, kNumKnots);
  VectorX<double> qdot0_seed = VectorX<double>::Zero(kNumJoints);
  IKResults ik_res;

  IKoptions ikoptions(tree.get());
  ikoptions.setFixInitialState(false);
  ikoptions.setQ(MatrixX<double>::Zero(kNumJoints, kNumJoints));

  // Create vectors to hold the constraint objects
  std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.resize(kNumKnots);
  Vector2<double> tspan;
  tspan << 0, 0.5;
  int end_effector_body_idx = tree->FindBodyIndex("iiwa_link_ee");
  Matrix3X<double> end_effector_points = Matrix3X<double>::Zero(3, 1);
  Vector3<double> position_tolerance = FLAGS_ptol*Vector3<double>::Ones();

  int constraint_array_idx{0};
  for (Isometry3<double> X_WE : X_WE_vector) {
      const Vector3<double>& r_WE = X_WE.translation();
      drake::log()->debug("r_WE = ({}), tspan = ({})", r_WE.transpose(),
                          tspan.transpose());
      // Construct position and orientation constraints
      position_constraints.emplace_back(new WorldPositionConstraint(
          tree.get(), end_effector_body_idx, end_effector_points,
          r_WE - position_tolerance, r_WE + position_tolerance, tspan));
      constraint_array[constraint_array_idx] = position_constraints.back().get();
      ++constraint_array_idx;
      tspan[0] += 1.0;
      tspan[1] += 1.0;
  }

  //MatrixX<double> q_sol(kNumJoints, kNumKnots);
  //MatrixX<double> qdot_sol(kNumJoints, kNumKnots);
  //MatrixX<double> qddot_sol(kNumJoints, kNumKnots);
  //int info;
  //std::vector<std::string> infeasible_constraint;

  //inverseKinTraj(tree.get(), kNumKnots, t.data(), qdot0_seed, q_seed, q_nom, constraint_array.size(), constraint_array.data(), ikoptions, &q_sol, &qdot_sol, &qddot_sol, &info, &infeasible_constraint);
  ik_res = inverseKinTrajSimple(tree.get(), t, q_seed, q_nom,
      constraint_array, ikoptions);
  const std::vector<VectorX<double>>& q{ik_res.q_sol};
  int info = ik_res.info[0];

  IiwaMove iiwa_move;
  MatrixX<double> q_sol(q.front().size(), q.size());
  std::vector<double> time;
  for (int i = 0; i < kNumKnots; ++i) {
    q_sol.col(i) = q[i];
    drake::log()->debug("Knot {}: q = ({})", i, q_sol.col(i).transpose());
    time.push_back(t[i]);
  }
  if (info != 1) {
    drake::log()->warn("IK traj failed with info {}", info);
    //for (std::string str : infeasible_constraint) {
      //drake::log()->debug(str);
    //}
  }
  std::vector<int> info_vector(time.size(), info);

  robotlocomotion::robot_plan_t plan{EncodeKeyFrames(*tree, time, info_vector, q_sol)};

  auto& plan_source_context = sys->GetMutableSubsystemContext(
      *trajectory_generator_, simulator->get_mutable_context());
  trajectory_generator_->Initialize(
      plan_source_context.get_time(), q_sol.col(0),
      plan_source_context.get_mutable_state());

  plan_source_context.FixInputPort(
      0, std::make_unique<systems::Value<robotlocomotion::robot_plan_t>>(
        plan));
  drake::log()->debug("Steping to t = {} s", time.back());
  simulator->StepTo(time.back());
  return 0;
}

}  // namespace
}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::pick_and_place::DoMain();
}
