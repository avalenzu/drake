#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/constraint_wrappers.h"
#include "drake/multibody/kinematics_cache_helper.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

DEFINE_double(collision_avoidance_threshold, 0.05,
              "Minimum distance to obstacles at all points.");
DEFINE_string(initial_ee_position, "0.5 0.5 0.5",
              "Initial end-effector position");
DEFINE_string(final_ee_position, "0.5 -0.5 0.5", "Final end-effector position");
DEFINE_string(initial_ee_orientation, "0.0 0.0 0.0",
              "Initial end-effector orientation (RPY in degrees)");
DEFINE_string(final_ee_orientation, "0.0 0.0 0.0",
              "Final end-effector position (RPY in degrees)");
DEFINE_string(obstacle_0_position, "0.1 0.4 0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_0_size, "0.2 0.3 1.5", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_1_position, "0.1 -0.4 0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_1_size, "0.2 0.3 1.5", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_2_position, "0.5 0.0 0.0", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_2_size, "0.5 1.0 0.3", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_3_position, "0.8 0.0 0.7", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_3_size, "0.5 1.0 0.8", "Dimensions of obstacle (m)");
DEFINE_string(obstacle_4_position, "0.6625 0.0 0",
              "Dimensions of obstacle (m)");
DEFINE_string(obstacle_4_size, "0.175 1.0 1.5", "Dimensions of obstacle (m)");
DEFINE_bool(flat_terrain, true, "If true, add flat terrain to the world.");
DEFINE_bool(loop_animation, true, "If true, repeat playback indefinitely");
DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 12, "Number of control points");
DEFINE_int32(num_evaluation_points, -1,
             "Number of points on the spline at which costs and constraints "
             "should be evaluated.");
DEFINE_int32(num_plotting_points, 1000,
             "Number of points to use when plotting.");
DEFINE_int32(derivatives_to_plot, 3, "Order of derivatives to plot.");
DEFINE_double(velocity_weight, -1, "Weight on squared velocity cost.");
DEFINE_double(acceleration_weight, -1, "Weight on squared acceleration cost.");
DEFINE_double(jerk_weight, -1, "Weight on squared jerk cost.");
DEFINE_double(duration_weight, -1, "Weight on squared duration cost.");
DEFINE_double(duration, 10, "Duration of the trajectory.");
DEFINE_double(max_velocity, 1.5, "Maximum allowable velocity.");
DEFINE_double(position_tolerance, 0.0, "Maximum position error.");
DEFINE_double(velocity_tolerance, 0.0, "Maximum velocity error.");
DEFINE_double(acceleration_tolerance, 0.0, "Maximum acceleration error.");
DEFINE_double(jerk_tolerance, -1.0, "Maximum jerk error.");
DEFINE_double(ee_orientation_tolerance, 1.0, "Orientation tolerance (degrees)");
DEFINE_double(ee_position_tolerance, 0.001, "Position tolerance");

namespace drake {
namespace manipulation {
namespace planner {
namespace {

using common::CallPython;
using common::ToPythonKwargs;
using symbolic::Expression;
using systems::DrakeVisualizer;
using systems::plants::KinematicsCacheHelper;
using systems::plants::SingleTimeKinematicConstraintWrapper;
using trajectories::PiecewisePolynomial;

int DoMain() {
  const int kSplineOrder = FLAGS_order;
  const int num_control_points = FLAGS_num_control_points;
  const int num_plotting_points = FLAGS_num_plotting_points;
  const int derivatives_to_plot = FLAGS_derivatives_to_plot;
  const int kNumInternalIntervals{num_control_points - kSplineOrder + 1};
  const int kNumEvaluationPoints = FLAGS_num_evaluation_points > 0
                                       ? FLAGS_num_evaluation_points
                                       : 3 * kNumInternalIntervals + 1;
  const double velocity_weight = FLAGS_velocity_weight;
  const double acceleration_weight = FLAGS_acceleration_weight;
  const double jerk_weight = FLAGS_jerk_weight;
  const double duration_weight = FLAGS_duration_weight;
  // const double kDuration = FLAGS_duration;
  double kCollisionAvoidanceThreshold{FLAGS_collision_avoidance_threshold};
  const double orientation_tolerance{FLAGS_ee_orientation_tolerance * M_PI /
                                     180};
  const double position_tolerance{FLAGS_ee_position_tolerance};

  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_collision.urdf");

  // const std::string kWsgModelPath = FindResourceOrThrow(
  //"drake/manipulation/models/wsg_50_description/urdf/"
  //"schunk_wsg_50_fixed_fingers.urdf");
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());
  // auto frame_ee = iiwa->findFrame("iiwa_frame_ee");
  // auto wsg_frame = std::make_shared<RigidBodyFrame<double>>(*frame_ee);
  // wsg_frame->get_mutable_transform_to_body()->rotate(
  // Eigen::AngleAxisd(-0.39269908, Eigen::Vector3d::UnitY()));
  // wsg_frame->get_mutable_transform_to_body()->translate(
  // 0.04 * Eigen::Vector3d::UnitY());
  // drake::parsers::urdf::AddModelInstanceFromUrdfFile(kWsgModelPath,
  // multibody::joints::kFixed, wsg_frame, iiwa.get());
  if (FLAGS_flat_terrain) {
    drake::multibody::AddFlatTerrainToWorld(iiwa.get());
  }

  // Add obstacles to the world
  // std::istringstream iss_obstacle_0_position{FLAGS_obstacle_0_position};
  // std::istringstream iss_obstacle_0_size{FLAGS_obstacle_0_size};
  // Vector3<double> obstacle_0_size;
  // Isometry3<double> X_WO_0{Isometry3<double>::Identity()};
  // for (int i = 0; i < 3; ++i) {
  // iss_obstacle_0_size >> obstacle_0_size(i);
  // DRAKE_THROW_UNLESS(!iss_obstacle_0_size.fail());
  // iss_obstacle_0_position >> X_WO_0.translation()(i);
  // DRAKE_THROW_UNLESS(!iss_obstacle_0_position.fail());
  //}
  //// Move the obstacle up to sit on the ground plane
  // X_WO_0.translation().z() += obstacle_0_size.z() / 2;

  const int kNumPositions{iiwa->get_num_positions()};
  const VectorX<double> kZeroVector{VectorX<double>::Zero(kNumPositions)};
  const VectorX<double> kOnesVector{VectorX<double>::Ones(kNumPositions)};
  const VectorX<double> kMaxVelocity = FLAGS_max_velocity * kOnesVector;
  const VectorX<double> kPositionTolerance =
      FLAGS_position_tolerance * kOnesVector;
  const VectorX<double> kVelocityTolerance =
      FLAGS_velocity_tolerance * kOnesVector;
  const VectorX<double> kAccelerationTolerance =
      FLAGS_acceleration_tolerance * kOnesVector;
  const VectorX<double> kJerkTolerance = FLAGS_jerk_tolerance * kOnesVector;

  KinematicTrajectoryOptimization program{iiwa->get_num_positions(),
                                          num_control_points, kSplineOrder};

  lcm::DrakeLcm lcm;
  lcm.StartReceiveThread();
  DrakeVisualizer visualizer{*iiwa, &lcm, true};
  visualizer.PublishLoadRobot();

  program.AddLinearConstraint(
      program.duration()(0) >= 1e-3 && program.duration()(0) <= 10, {{0., 0.}});
  // const double kTStart{0.0};
  // const double kTMid{0.5};
  // const double kTEnd{1.0};
  const VectorX<double> kPositionTargetStart{0.0 * kOnesVector};
  const VectorX<double> kPositionTargetMid{0.5 * kOnesVector};
  VectorX<double> kPositionTargetEnd(kNumPositions);
  kPositionTargetEnd << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, -0.7;
  const VectorX<double> kVelocityTargetStart{kZeroVector};
  const VectorX<double> kVelocityTargetMid{kZeroVector};
  const VectorX<double> kVelocityTargetEnd{kZeroVector};
  const VectorX<double> kAccelerationTargetStart{kZeroVector};
  const VectorX<double> kAccelerationTargetMid{kZeroVector};
  const VectorX<double> kAccelerationTargetEnd{kZeroVector};
  const VectorX<double> kJerkTargetStart{kZeroVector};
  const VectorX<double> kJerkTargetMid{kZeroVector};
  const VectorX<double> kJerkTargetEnd{kZeroVector};

  if (kPositionTolerance(0) > 0) {
    // program.AddLinearConstraint(program.position(kTStart) <=
    // kPositionTargetStart + kPositionTolerance);
    // program.AddLinearConstraint(program.position(kTStart) >=
    // kPositionTargetStart - kPositionTolerance);
  } else {
    // program.AddLinearConstraint(program.position(kTStart) ==
    // kPositionTargetStart);
  }

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
  X_WF0.linear() = drake::math::rpy2rotmat(M_PI / 180 * rpy_WF0);
  X_WFf.linear() = drake::math::rpy2rotmat(M_PI / 180 * rpy_WFf);

  std::vector<std::unique_ptr<WorldPositionConstraint>> position_constraints;
  std::vector<std::unique_ptr<WorldQuatConstraint>> orientation_constraints;
  std::vector<std::unique_ptr<KinematicsCacheHelper<double>>> cache_helpers;
  const int end_effector_index =
      iiwa->findFrame("iiwa_link_ee")->get_frame_index();
  const auto end_effector_points = Matrix3X<double>::Zero(3, 1);

  MinDistanceConstraint min_distance_constraint{
      iiwa.get(), kCollisionAvoidanceThreshold, {}, {}};
  if (kCollisionAvoidanceThreshold > 0) {
    drake::log()->info("Adding collision avoidance constraints ...");
    cache_helpers.emplace_back(new KinematicsCacheHelper<double>(*iiwa));
    program.AddGenericPositionConstraint(
        std::make_shared<SingleTimeKinematicConstraintWrapper>(
            &min_distance_constraint, cache_helpers.back().get()),
        {{0., 1.}});
    drake::log()->info("\tDone.");
  }

  drake::log()->info("Adding body-pose constraint to mid-point ...");
  if (position_tolerance > 0) {
    auto lb = X_WF0.translation() -
              MatrixX<double>::Constant(3, 1, position_tolerance);
    auto ub = X_WF0.translation() +
              MatrixX<double>::Constant(3, 1, position_tolerance);
    position_constraints.emplace_back(new WorldPositionConstraint(
        iiwa.get(), end_effector_index, end_effector_points, lb, ub));
    cache_helpers.emplace_back(new KinematicsCacheHelper<double>(*iiwa));
    program.AddGenericPositionConstraint(
        std::make_shared<SingleTimeKinematicConstraintWrapper>(
            position_constraints.back().get(), cache_helpers.back().get()),
        {{0.5, 0.5}});
  }
  if (orientation_tolerance > 0) {
    Quaternion<double> quat_WF{X_WF0.linear()};
    orientation_constraints.emplace_back(new WorldQuatConstraint(
        iiwa.get(), end_effector_index,
        {quat_WF.w(), quat_WF.x(), quat_WF.y(), quat_WF.z()},
        orientation_tolerance));
    cache_helpers.emplace_back(new KinematicsCacheHelper<double>(*iiwa));
    program.AddGenericPositionConstraint(
        std::make_shared<SingleTimeKinematicConstraintWrapper>(
            orientation_constraints.back().get(), cache_helpers.back().get()),
        {{0.5, 0.5}});
  }
  program.AddLinearConstraint(
      program.velocity() == VectorX<double>::Zero(iiwa->get_num_velocities()),
      {{0.5, 0.5}});
  program.AddLinearConstraint(
      program.acceleration() == VectorX<double>::Zero(iiwa->get_num_velocities()),
      {{0.5, 0.5}});
  drake::log()->info("\tDone.");

  drake::log()->info("Adding body-pose constraint to end-point ...");
  if (position_tolerance > 0) {
    auto lb = X_WFf.translation() -
              MatrixX<double>::Constant(3, 1, position_tolerance);
    auto ub = X_WFf.translation() +
              MatrixX<double>::Constant(3, 1, position_tolerance);
    position_constraints.emplace_back(new WorldPositionConstraint(
        iiwa.get(), end_effector_index, end_effector_points, lb, ub));
    cache_helpers.emplace_back(new KinematicsCacheHelper<double>(*iiwa));
    program.AddGenericPositionConstraint(
        std::make_shared<SingleTimeKinematicConstraintWrapper>(
            position_constraints.back().get(), cache_helpers.back().get()),
        {{1., 1.}});
  }
  if (orientation_tolerance > 0) {
    Quaternion<double> quat_WF{X_WFf.linear()};
    orientation_constraints.emplace_back(new WorldQuatConstraint(
        iiwa.get(), end_effector_index,
        {quat_WF.w(), quat_WF.x(), quat_WF.y(), quat_WF.z()},
        orientation_tolerance));
    cache_helpers.emplace_back(new KinematicsCacheHelper<double>(*iiwa));
    program.AddGenericPositionConstraint(
        std::make_shared<SingleTimeKinematicConstraintWrapper>(
            orientation_constraints.back().get(), cache_helpers.back().get()),
        {{1., 1.}});
  }
  program.AddLinearConstraint(
      program.velocity() == VectorX<double>::Zero(iiwa->get_num_velocities()),
      {{1., 1.}});
  program.AddLinearConstraint(
      program.acceleration() == VectorX<double>::Zero(iiwa->get_num_velocities()),
      {{1., 1.}});
  drake::log()->info("\tDone.");

  // Constrain initial position, velocity, and acceleration.
  program.AddLinearConstraint(
      program.position() == iiwa->getZeroConfiguration(), {{0., 0.}});
  program.AddLinearConstraint(
      program.velocity() == VectorX<double>::Zero(iiwa->get_num_velocities()),
      {{0., 0.}});
  program.AddLinearConstraint(
      program.acceleration() == VectorX<double>::Zero(iiwa->get_num_velocities()),
      {{0., 0.}});

  // Add position limits
  program.AddLinearConstraint(program.position() >= iiwa->joint_limit_min &&
                              program.position() <= iiwa->joint_limit_max);

  // Add velocity limits.
  const VectorX<double> max_joint_velocities =
      examples::kuka_iiwa_arm::get_iiwa_max_joint_velocities();
  program.AddLinearConstraint(
      program.velocity() >= -program.duration()(0) * max_joint_velocities &&
      program.velocity() <= program.duration()(0) * max_joint_velocities);

  // Add min-time cost.
  if (duration_weight > 0) {
    program.AddQuadraticCost(
        program.duration().transpose() * program.duration(), {{0., 0.}});
  }
  if (velocity_weight > 0) {
    program.AddQuadraticCost(program.velocity().transpose() *
                             program.velocity());
  }
  if (acceleration_weight > 0) {
    program.AddQuadraticCost(program.acceleration().transpose() *
                             program.acceleration());
  }
  if (jerk_weight > 0) {
    program.AddQuadraticCost(program.jerk().transpose() * program.jerk());
  }

  // if (kVelocityTolerance(0) > 0) {
  // program.AddLinearConstraint(program.velocity(kTStart) <=
  // kVelocityTargetStart + kVelocityTolerance);
  // program.AddLinearConstraint(program.velocity(kTStart) >=
  // kVelocityTargetStart - kVelocityTolerance);
  // program.AddLinearConstraint(program.velocity(kTMid) <=
  // kVelocityTargetMid + kVelocityTolerance);
  // program.AddLinearConstraint(program.velocity(kTMid) >=
  // kVelocityTargetMid - kVelocityTolerance);
  // program.AddLinearConstraint(program.velocity(kTEnd) <=
  // kVelocityTargetEnd + kVelocityTolerance);
  // program.AddLinearConstraint(program.velocity(kTEnd) >=
  // kVelocityTargetEnd - kVelocityTolerance);
  //} else {
  // program.AddLinearConstraint(program.velocity(kTStart) ==
  // kVelocityTargetStart);
  // program.AddLinearConstraint(program.velocity(kTMid) == kVelocityTargetMid);
  // program.AddLinearConstraint(program.velocity(kTEnd) == kVelocityTargetEnd);
  //}

  // if (kAccelerationTolerance(0) > 0) {
  // program.AddLinearConstraint(program.acceleration(kTStart) <=
  // kAccelerationTargetStart +
  // kAccelerationTolerance);
  // program.AddLinearConstraint(program.acceleration(kTStart) >=
  // kAccelerationTargetStart -
  // kAccelerationTolerance);
  // program.AddLinearConstraint(program.acceleration(kTMid) <=
  // kAccelerationTargetMid +
  // kAccelerationTolerance);
  // program.AddLinearConstraint(program.acceleration(kTMid) >=
  // kAccelerationTargetMid -
  // kAccelerationTolerance);
  // program.AddLinearConstraint(program.acceleration(kTEnd) <=
  // kAccelerationTargetEnd +
  // kAccelerationTolerance);
  // program.AddLinearConstraint(program.acceleration(kTEnd) >=
  // kAccelerationTargetEnd -
  // kAccelerationTolerance);
  //} else {
  // program.AddLinearConstraint(program.acceleration(kTStart) ==
  // kAccelerationTargetStart);
  // program.AddLinearConstraint(program.acceleration(kTMid) ==
  // kAccelerationTargetMid);
  // program.AddLinearConstraint(program.acceleration(kTEnd) ==
  // kAccelerationTargetEnd);
  //}

  // if (kJerkTolerance(0) > 0) {
  // program.AddLinearConstraint(program.jerk(kTStart) <=
  // kJerkTargetStart + kJerkTolerance);
  // program.AddLinearConstraint(program.jerk(kTStart) >=
  // kJerkTargetStart - kJerkTolerance);
  // program.AddLinearConstraint(program.jerk(kTMid) <=
  // kJerkTargetMid + kJerkTolerance);
  // program.AddLinearConstraint(program.jerk(kTMid) >=
  // kJerkTargetMid - kJerkTolerance);
  // program.AddLinearConstraint(program.jerk(kTEnd) <=
  // kJerkTargetEnd + kJerkTolerance);
  // program.AddLinearConstraint(program.jerk(kTEnd) >=
  // kJerkTargetEnd - kJerkTolerance);
  //} else if (kJerkTolerance(0) == 0.0) {
  // program.AddLinearConstraint(program.jerk(kTStart) == kJerkTargetStart);
  // program.AddLinearConstraint(program.jerk(kTMid) == kJerkTargetMid);
  // program.AddLinearConstraint(program.jerk(kTEnd) == kJerkTargetEnd);
  //}

  VectorX<double> evaluation_times =
      VectorX<double>::LinSpaced(kNumEvaluationPoints, 0.0, 1.0);
  symbolic::Substitution control_point_substitution;
  // for (int i = 0; i < kNumPositions; ++i) {
  // for (int j = 0; j < num_control_points; ++j) {
  // control_point_substitution.emplace(
  // program.control_points()(i, j),
  // std::sqrt(kJerkWeight) * program.control_points()(i, j));
  //}
  //}
  // Expression jerk_squared_cost{Expression::Zero()};
  // for (int i = 0; i < kNumEvaluationPoints; ++i) {
  // if (kJerkWeight > 0 && i < kNumEvaluationPoints - 1) {
  // VectorX<Expression> jerk0 = program.jerk(evaluation_times(i));
  // VectorX<Expression> jerk1 = program.jerk(evaluation_times(i + 1));
  // jerk_squared_cost +=
  //(evaluation_times(i + 1) - evaluation_times(i)) * 0.5 *
  //(jerk0.transpose() * jerk0 + jerk1.transpose() * jerk1)(0);
  //// drake::log()->info("Cost for t = {}: {}", evaluation_times(i),
  //// jerk_squared_cost);
  //}
  // if (FLAGS_max_velocity > 0) {
  // program.AddLinearConstraint(program.velocity(evaluation_times(i)) <=
  // kMaxVelocity);
  // program.AddLinearConstraint(program.velocity(evaluation_times(i)) >=
  //-kMaxVelocity);
  //}
  //}
  // if (kJerkWeight > 0) {
  // program.AddQuadraticCost(
  // jerk_squared_cost.Substitute(control_point_substitution));
  //}

  drake::log()->info("Calling program.Solve() ...");
  bool done{false};
  while (!done) {
    solvers::SolutionResult result = program.Solve(false);
    drake::log()->info("Solution result: {}", result);
    if (result == solvers::SolutionResult::kSolutionFound) {
      done = !program.UpdateGenericConstraints();
    } else {
      done = !program.AddKnots();
    }
  }
  drake::log()->info("... Done.");

  trajectories::PiecewisePolynomial<double> solution_trajectory{
      program.GetPositionAndVelocitySolution(1)};

  const PiecewisePolynomial<double> curve = program.GetPositionSolution();
  const VectorX<double> t{
      VectorX<double>::LinSpaced(num_plotting_points, 0, curve.end_time())};
  MatrixX<double> curve_values(curve.rows(), t.size());
  CallPython("figure", 1);
  CallPython("clf");
  auto fig_and_axes = CallPython("plt.subplots", derivatives_to_plot + 1, 1,
                                 ToPythonKwargs("squeeze", false, "num", 1));
  auto axes = fig_and_axes[1];
  for (int derivative_order = 0; derivative_order <= derivatives_to_plot;
       ++derivative_order) {
    for (int i = 0; i < curve.rows(); ++i) {
      for (int plotting_point_index = 0;
           plotting_point_index < num_plotting_points; ++plotting_point_index) {
        curve_values(i, plotting_point_index) =
            curve.derivative(derivative_order)
                .value(t(plotting_point_index))(i, 0);
      }
      axes[derivative_order][0].attr("plot")(
          t, 180. / M_PI * curve_values.row(i).transpose());
    }
  }

  // Publish an animation of the trajectory to the visualizer.
  do {
    visualizer.PlaybackTrajectory(solution_trajectory);
  } while (FLAGS_loop_animation);
  return 0;
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

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::manipulation::planner::DoMain();
}
