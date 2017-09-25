#include <random>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(duration, 10, "Maximum duration of trajectory.");
DEFINE_double(min_timestep, 0.01, "Minimum duration of a single timestep.");
DEFINE_double(max_timestep, 1.0, "Maximum duration of a single timestep.");
DEFINE_double(spatial_velocity_weight, 1e0,
              "Relative weight of end-effector spatial velocity cost");
DEFINE_double(velocity_weight, 0e0,
              "Relative weight of velocity-squared cost");
DEFINE_double(acceleration_weight, 0e0,
              "Relative weight of acceleration-squared cost");
DEFINE_double(jerk_weight, 1e0,
              "Relative weight of jerk-squared cost");
DEFINE_double(tfinal_weight, 1e1,
              "Relative weight of final-time cost");
DEFINE_double(orientation_tolerance, 1.0, "Orientation tolerance (degrees)");
DEFINE_double(position_tolerance, 0.001, "Position tolerance");
DEFINE_double(realtime_rate, 1.0, "Playback speed relative to real-time");
DEFINE_double(collision_avoidance_threshold, 0.05, "Minimum distance to obstacles at all points.");
DEFINE_double(collision_avoidance_knot_threshold, 0.1,
              "Minimum distance to obstacles at knot points. Should be greater "
              "than collsion_avoidance_threshold.");
DEFINE_double(max_velocity, 1.5, "Maximum joint-velocity for all joints");
DEFINE_double(optimality_tolerance, 1e-6, "Major optimality tolerance for solver");
DEFINE_string(initial_ee_position, "0.5 0.5 0.5", "Initial end-effector position");
DEFINE_string(final_ee_position, "0.5 -0.5 0.5", "Final end-effector position");
DEFINE_string(initial_ee_orientation, "0.0 0.0 0.0", "Initial end-effector orientation (RPY in degrees)");
DEFINE_string(final_ee_orientation, "0.0 0.0 0.0", "Final end-effector position (RPY in degrees)");
DEFINE_string(obstacle_0_position, "0.5 0.0 0.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_0_size, "0.2 0.2 1.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_1_position, "0.5 0.0 0.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_1_size, "0.2 0.2 1.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_2_position, "0.5 0.0 0.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_2_size, "0.2 0.2 1.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_3_position, "0.5 0.0 0.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_3_size, "0.2 0.2 1.0", "Dimensions of obstacle (m)");
DEFINE_string(velocity_cost_body, "iiwa_link_ee", "Name of the body whose spatial velocity will be penalized.");
DEFINE_bool(animate_with_zoh, false, "If true, use a zero-order hold to display trajectory");
DEFINE_bool(loop_animation, true, "If true, repeat playback indefinitely");
DEFINE_bool(flat_terrain, true, "If true, add flat terrain to the world.");
DEFINE_int32(iteration_limit, 1e3, "Number of iterations allowed");
DEFINE_int32(num_knots, 100, "Number of knot points.");
DEFINE_int32(initial_num_knots, 2, "Number of knot points.");
DEFINE_int32(system_order, 3, "Order of the dynamics model for the system.");

using drake::solvers::SolutionResult;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::SignalLogger;
using drake::systems::TrajectorySource;
using drake::systems::DrakeVisualizer;
using drake::systems::Simulator;

namespace drake {
namespace manipulation {
namespace planner {
int DoMain() {
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_mesh_collision.urdf");
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());
  if (FLAGS_flat_terrain) {
    drake::multibody::AddFlatTerrainToWorld(iiwa.get());
  }


  const double kDuration{FLAGS_duration};
  const double kMinimumTimestep{FLAGS_min_timestep};
  const double kMaximumTimestep{FLAGS_max_timestep};
  const int kFinalNumKnots{FLAGS_num_knots};
  int num_knots{FLAGS_initial_num_knots};
  drake::log()->info("Number of knots: {}", num_knots);

  KinematicTrajectoryOptimization kin_traj_opt{
      std::move(iiwa), num_knots, kMinimumTimestep, kMaximumTimestep};
  kin_traj_opt.set_system_order(FLAGS_system_order);
  kin_traj_opt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                        "Major iterations limit", FLAGS_iteration_limit);
  kin_traj_opt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                               "Major optimality tolerance",
                               FLAGS_optimality_tolerance);

  // Add obstacles to the world
  std::istringstream iss_obstacle_0_position{FLAGS_obstacle_0_position};
  std::istringstream iss_obstacle_0_size{FLAGS_obstacle_0_size};
  Vector3<double> obstacle_0_size;
  Isometry3<double> X_WO_0{Isometry3<double>::Identity()};
  for (int i = 0; i < 3; ++i) {
    iss_obstacle_0_size >> obstacle_0_size(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_0_size.fail());
    iss_obstacle_0_position >> X_WO_0.translation()(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_0_position.fail());
  }
  // Move the obstacle up to sit on the ground plane
  X_WO_0.translation().z() += obstacle_0_size.z() / 2;

  kin_traj_opt.AddFixedBoxToWorld(obstacle_0_size, X_WO_0);

  std::istringstream iss_obstacle_1_position{FLAGS_obstacle_1_position};
  std::istringstream iss_obstacle_1_size{FLAGS_obstacle_1_size};
  Vector3<double> obstacle_1_size;
  Isometry3<double> X_WO_1{Isometry3<double>::Identity()};
  for (int i = 0; i < 3; ++i) {
    iss_obstacle_1_size >> obstacle_1_size(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_1_size.fail());
    iss_obstacle_1_position >> X_WO_1.translation()(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_1_position.fail());
  }
  // Move the obstacle up to sit on the ground plane
  X_WO_1.translation().z() += obstacle_1_size.z() / 2;

  kin_traj_opt.AddFixedBoxToWorld(obstacle_1_size, X_WO_1);

  std::istringstream iss_obstacle_2_position{FLAGS_obstacle_2_position};
  std::istringstream iss_obstacle_2_size{FLAGS_obstacle_2_size};
  Vector3<double> obstacle_2_size;
  Isometry3<double> X_WO_2{Isometry3<double>::Identity()};
  for (int i = 0; i < 3; ++i) {
    iss_obstacle_2_size >> obstacle_2_size(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_2_size.fail());
    iss_obstacle_2_position >> X_WO_2.translation()(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_2_position.fail());
  }
  // Move the obstacle up to sit on the ground plane
  X_WO_2.translation().z() += obstacle_2_size.z() / 2;

  kin_traj_opt.AddFixedBoxToWorld(obstacle_2_size, X_WO_2);

  std::istringstream iss_obstacle_3_position{FLAGS_obstacle_3_position};
  std::istringstream iss_obstacle_3_size{FLAGS_obstacle_3_size};
  Vector3<double> obstacle_3_size;
  Isometry3<double> X_WO_3{Isometry3<double>::Identity()};
  for (int i = 0; i < 3; ++i) {
    iss_obstacle_3_size >> obstacle_3_size(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_3_size.fail());
    iss_obstacle_3_position >> X_WO_3.translation()(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_3_position.fail());
  }
  // Move the obstacle up to sit on the ground plane
  X_WO_3.translation().z() += obstacle_3_size.z() / 2;

  kin_traj_opt.AddFixedBoxToWorld(obstacle_3_size, X_WO_3);

  lcm::DrakeLcm lcm;
  lcm.StartReceiveThread();
  DrakeVisualizer visualizer{kin_traj_opt.tree(), &lcm, true};
  visualizer.PublishLoadRobot();

  const int kNumPositions = kin_traj_opt.num_positions();
  const int kNumVelocities = kin_traj_opt.num_velocities();

  // Enforce uniform spacing of knots
  kin_traj_opt.AddEqualTimeIntervalsConstraints();

  // Enforce maximum duration
  const double kMinDuration{0.1};
  kin_traj_opt.AddDurationBounds(kMinDuration, kDuration);

  // q[0] = q0
  VectorX<double> q0 = kin_traj_opt.tree().getZeroConfiguration();
  kin_traj_opt.AddLinearConstraint(
      kin_traj_opt.position() == q0, 0);

  const VectorX<double> kZeroNumVelocity{
      VectorX<double>::Zero(kin_traj_opt.num_velocities())};
  // v[0] = 0 and a[0] = 0.
  kin_traj_opt.AddLinearConstraint(kin_traj_opt.velocity() == kZeroNumVelocity,
                                   0);
  kin_traj_opt.AddLinearConstraint(
      kin_traj_opt.acceleration() == kZeroNumVelocity, 0);

  // v[tmid] = 0 and a[tmid] = 0.
  kin_traj_opt.AddLinearConstraint(kin_traj_opt.velocity() == kZeroNumVelocity,
                                   0.5);
  kin_traj_opt.AddLinearConstraint(
      kin_traj_opt.acceleration() == kZeroNumVelocity, 0.5);

  // v[tf] = 0 and a[tf] = 0.
  kin_traj_opt.AddLinearConstraint(kin_traj_opt.velocity() == kZeroNumVelocity,
                                   1);
  kin_traj_opt.AddLinearConstraint(
      kin_traj_opt.acceleration() == kZeroNumVelocity, 1);

  // v[i] < v_max
  const VectorX<double> kMaxVelocity{
      FLAGS_max_velocity *
      VectorX<double>::Ones(kin_traj_opt.num_velocities())};
  kin_traj_opt.AddLinearConstraint(kin_traj_opt.velocity() <= kMaxVelocity);
  kin_traj_opt.AddLinearConstraint(-kMaxVelocity <= kin_traj_opt.velocity());

  if (FLAGS_velocity_weight > 0) {
    kin_traj_opt.AddRunningCost(FLAGS_velocity_weight *
        kin_traj_opt.velocity().transpose() *
        kin_traj_opt.velocity());
  }
  if (FLAGS_acceleration_weight > 0) {
    kin_traj_opt.AddRunningCost(FLAGS_acceleration_weight *
        kin_traj_opt.acceleration().transpose() *
        kin_traj_opt.acceleration());
  }
  if (FLAGS_jerk_weight > 0) {
    kin_traj_opt.AddRunningCost(FLAGS_jerk_weight *
        kin_traj_opt.jerk().transpose() *
        kin_traj_opt.jerk());
  }
  if (FLAGS_tfinal_weight > 0) {
    kin_traj_opt.AddFinalCost(FLAGS_tfinal_weight*kin_traj_opt.time()(0));
  }

  // Add spatial velocity cost
  if (FLAGS_spatial_velocity_weight > 0) {
    kin_traj_opt.AddSpatialVelocityCost(FLAGS_velocity_cost_body,
                                        FLAGS_spatial_velocity_weight);
  }

  // Add middle and final pose constraints
  Isometry3<double> X_WF0{Isometry3<double>::Identity()};
  Isometry3<double> X_WFf{Isometry3<double>::Identity()};
  Vector3<double> rpy_WF0;
  Vector3<double> rpy_WFf;
  std::istringstream iss_initial_ee_position{FLAGS_initial_ee_position};
  std::istringstream iss_final_ee_position{FLAGS_final_ee_position};
  std::istringstream iss_initial_ee_orientation{FLAGS_initial_ee_orientation};
  std::istringstream iss_final_ee_orientation{FLAGS_final_ee_orientation};
  for (int i = 0; i < 3; ++i) {
    iss_initial_ee_position >> X_WF0.translation()(i);
    DRAKE_THROW_UNLESS(!iss_initial_ee_position.fail());
    iss_final_ee_position >> X_WFf.translation()(i);
    DRAKE_THROW_UNLESS(!iss_final_ee_position.fail());
    iss_initial_ee_orientation >> rpy_WF0(i);
    DRAKE_THROW_UNLESS(!iss_initial_ee_orientation.fail());
    iss_final_ee_orientation >> rpy_WFf(i);
    DRAKE_THROW_UNLESS(!iss_final_ee_orientation.fail());
  }
  X_WF0.linear() = drake::math::rpy2rotmat(M_PI/180*rpy_WF0);
  X_WFf.linear() = drake::math::rpy2rotmat(M_PI/180*rpy_WFf);

  const double kOrientationTolerance{FLAGS_orientation_tolerance*M_PI/180};

  kin_traj_opt.AddBodyPoseConstraint(0.5, "iiwa_link_ee", X_WF0,
                                     kOrientationTolerance,
                                     FLAGS_position_tolerance);
  kin_traj_opt.AddBodyPoseConstraint(1, "iiwa_link_ee", X_WFf,
                                     kOrientationTolerance,
                                     FLAGS_position_tolerance);

  // Add collision avoidance constraints
  double kCollisionAvoidanceThreshold{FLAGS_collision_avoidance_threshold};
  double kCollisionAvoidanceKnotThreshold{FLAGS_collision_avoidance_knot_threshold};
  DRAKE_THROW_UNLESS(kCollisionAvoidanceThreshold <= kCollisionAvoidanceKnotThreshold);
  if (kCollisionAvoidanceThreshold > 0) {
    kin_traj_opt.AddCollisionAvoidanceConstraint(kCollisionAvoidanceKnotThreshold);
  }

  SolutionResult result{drake::solvers::kUnknownError};
  const int kNumSeedKnots{2};
  std::vector<double> t_seed(kNumSeedKnots);
  std::vector<MatrixX<double>> q_seed{kNumSeedKnots};
  q_seed[0] = kin_traj_opt.tree().getZeroConfiguration();
  t_seed[0] = 0;
  for (int i = 1; i < kNumSeedKnots; ++i) {
    t_seed[i] = kMinDuration * static_cast<double>(i) /
                static_cast<double>(kNumSeedKnots - 1);
    q_seed[i] = q_seed[0];
  }
  kin_traj_opt.SetInitialTrajectory(PiecewisePolynomial<double>::Cubic(
      t_seed, q_seed, VectorX<double>::Zero(kNumPositions),
      VectorX<double>::Zero(kNumPositions)));

  kin_traj_opt.set_system_order(1);
  while (num_knots < kFinalNumKnots) {
    result = kin_traj_opt.Solve();
    drake::log()->info(
        "Solved {}-order model with {} knots: Solver returned {}. Trajectory "
        "Duration = {} s",
        kin_traj_opt.system_order(), kin_traj_opt.num_time_samples(), result,
        kin_traj_opt.GetPositionTrajectory().get_end_time());
    if (result == drake::solvers::kSolutionFound &&
        kin_traj_opt.IsPositionTrajectoryCollisionFree(
            kCollisionAvoidanceThreshold)) {
      break;
    }
    drake::log()->info("Refining trajectory ...");
    kin_traj_opt.SetInitialTrajectory(
        kin_traj_opt.GetPositionTrajectory().get_piecewise_polynomial());
    num_knots = num_knots + (num_knots-1);
    kin_traj_opt.set_num_time_samples(num_knots);
  }

  drake::log()->info("Refining trajectory ...");
  kin_traj_opt.SetInitialTrajectory(
      kin_traj_opt.GetPositionTrajectory().get_piecewise_polynomial());
  kin_traj_opt.set_system_order(3);
  while (num_knots < kFinalNumKnots) {
    result = kin_traj_opt.Solve();
    drake::log()->info(
        "Solved {}-order model with {} knots: Solver returned {}. Trajectory "
        "Duration = {} s",
        kin_traj_opt.system_order(), kin_traj_opt.num_time_samples(), result,
        kin_traj_opt.GetPositionTrajectory().get_end_time());
    if (result == drake::solvers::kSolutionFound &&
        kin_traj_opt.IsPositionTrajectoryCollisionFree(
            kCollisionAvoidanceThreshold)) {
      break;
    }
    drake::log()->info("Refining trajectory ...");
    kin_traj_opt.SetInitialTrajectory(
        kin_traj_opt.GetPositionTrajectory().get_piecewise_polynomial());
    num_knots = num_knots + (num_knots-1);
    kin_traj_opt.set_num_time_samples(num_knots);
  }

  auto q_sol = kin_traj_opt.GetPositionTrajectory();
  std::vector<double> breaks{
      q_sol.get_piecewise_polynomial().getSegmentTimes()};
  std::vector<MatrixX<double>> knots;
  std::vector<MatrixX<double>> knots_dot;
  knots.reserve(breaks.size());
  knots_dot.reserve(breaks.size());
  drake::log()->info("Number of knots in solution: {}", breaks.size());
  drake::log()->info("Duration of solution trajectory: {} s",
                     q_sol.get_end_time() - q_sol.get_start_time());
  for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
    const double t{breaks[i]};
    // The visualizer won't use the velocities, so we only populate the
    // positions here.
    knots.emplace_back(kNumPositions + kNumVelocities, 1);
    knots.back().topRows(kNumPositions) = q_sol.value(t);
    knots_dot.emplace_back(kNumPositions + kNumVelocities, 1);
    knots_dot.back().topRows(kNumPositions) =
        FLAGS_realtime_rate * q_sol.derivative(1)->value(t);
    breaks[i] /= FLAGS_realtime_rate;
  }
  std::unique_ptr<PiecewisePolynomialTrajectory> q_and_v_traj;
  if (FLAGS_animate_with_zoh) {
    q_and_v_traj = std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::ZeroOrderHold(breaks, knots));
  } else {
    q_and_v_traj = std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomial<double>::Cubic(breaks, knots, knots_dot));
  }

  do {
    visualizer.PlaybackTrajectory(q_and_v_traj->get_piecewise_polynomial());
  } while (FLAGS_loop_animation);

  return result;
}
}  // namespace drake
}  // namespace manipulation
}  // namespace planner

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::manipulation::planner::DoMain();
}
