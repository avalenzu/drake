#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/planner/kinematic_planning_problem.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
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
DEFINE_string(obstacle_4_position, "0.6625 0.0 0", "Dimensions of obstacle (m)");
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
DEFINE_double(jerk_weight, 1, "Weight applied to the squared jerk cost.");
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

using common::CallMatlab;
using symbolic::Expression;
using systems::DrakeVisualizer;

int DoMain() {
  const int kSplineOrder = FLAGS_order;
  const int kNumControlPoints = FLAGS_num_control_points;
  const int kNumPlottingPoints = FLAGS_num_plotting_points;
  const int kNumInternalIntervals{kNumControlPoints - kSplineOrder + 1};
  const int kNumEvaluationPoints = FLAGS_num_evaluation_points > 0
                                       ? FLAGS_num_evaluation_points
                                       : 3 * kNumInternalIntervals + 1;
  const double kJerkWeight = FLAGS_jerk_weight;
  const double kDuration = FLAGS_duration;
  double kCollisionAvoidanceThreshold{FLAGS_collision_avoidance_threshold};
  const double kEndEffectorOrientationTolerance{FLAGS_ee_orientation_tolerance *
                                                M_PI / 180};
  const double kEndEffectorPositionTolerance{FLAGS_ee_position_tolerance};

  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_sphere_collision.urdf");

  const std::string kWsgModelPath = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/urdf/"
      "schunk_wsg_50_fixed_fingers.urdf");
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());
  auto frame_ee = iiwa->findFrame("iiwa_frame_ee");
  auto wsg_frame = frame_ee->Clone(frame_ee->get_mutable_rigid_body());
  wsg_frame->get_mutable_transform_to_body()->rotate(
      Eigen::AngleAxisd(-0.39269908, Eigen::Vector3d::UnitY()));
  wsg_frame->get_mutable_transform_to_body()->translate(
      0.04 * Eigen::Vector3d::UnitY());
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(kWsgModelPath, multibody::joints::kFixed, wsg_frame, iiwa.get());
  if (FLAGS_flat_terrain) {
    drake::multibody::AddFlatTerrainToWorld(iiwa.get());
  }

  KinematicPlanningProblem problem{std::move(iiwa)};

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

  problem.AddFixedBoxToWorld(obstacle_0_size, X_WO_0);

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

  problem.AddFixedBoxToWorld(obstacle_1_size, X_WO_1);

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

  problem.AddFixedBoxToWorld(obstacle_2_size, X_WO_2);

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

  problem.AddFixedBoxToWorld(obstacle_3_size, X_WO_3);

  std::istringstream iss_obstacle_4_position{FLAGS_obstacle_4_position};
  std::istringstream iss_obstacle_4_size{FLAGS_obstacle_4_size};
  Vector3<double> obstacle_4_size;
  Isometry3<double> X_WO_4{Isometry3<double>::Identity()};
  for (int i = 0; i < 3; ++i) {
    iss_obstacle_4_size >> obstacle_4_size(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_4_size.fail());
    iss_obstacle_4_position >> X_WO_4.translation()(i);
    DRAKE_THROW_UNLESS(!iss_obstacle_4_position.fail());
  }
  // Move the obstacle up to sit on the ground plane
  X_WO_4.translation().z() += obstacle_4_size.z() / 2;

  problem.AddFixedBoxToWorld(obstacle_4_size, X_WO_4);

  const int kNumPositions{problem.num_positions()};
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

  KinematicTrajectoryOptimization program{&problem.tree(), kNumControlPoints,
                                          kNumEvaluationPoints, kSplineOrder,
                                          kDuration};

  lcm::DrakeLcm lcm;
  lcm.StartReceiveThread();
  DrakeVisualizer visualizer{problem.tree(), &lcm, true};
  visualizer.PublishLoadRobot();

  const double kTStart{0.0};
  const double kTMid{0.5};
  const double kTEnd{1.0};
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
    program.AddLinearConstraint(program.position(kTStart) <=
                                kPositionTargetStart + kPositionTolerance);
    program.AddLinearConstraint(program.position(kTStart) >=
                                kPositionTargetStart - kPositionTolerance);
  } else {
    program.AddLinearConstraint(program.position(kTStart) ==
                                kPositionTargetStart);
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

  if (kCollisionAvoidanceThreshold > 0) {
    drake::log()->info("Adding collision avoidance constraints ...");
    program.AddCollisionAvoidanceConstraint(kCollisionAvoidanceThreshold);
    drake::log()->info("\tDone.");
  }

  drake::log()->info("Adding body-pose constraint to mid-point ...");
  program.AddBodyPoseConstraint(0.5, "iiwa_link_ee", X_WF0,
                                kEndEffectorOrientationTolerance,
                                kEndEffectorPositionTolerance);
  drake::log()->info("\tDone.");

  drake::log()->info("Adding body-pose constraint to end-point ...");
  program.AddBodyPoseConstraint(1, "iiwa_link_ee", X_WFf, kEndEffectorOrientationTolerance,
                                kEndEffectorPositionTolerance);
  drake::log()->info("\tDone.");

  if (kVelocityTolerance(0) > 0) {
    program.AddLinearConstraint(program.velocity(kTStart) <=
                                kVelocityTargetStart + kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(kTStart) >=
                                kVelocityTargetStart - kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(kTMid) <=
                                kVelocityTargetMid + kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(kTMid) >=
                                kVelocityTargetMid - kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(kTEnd) <=
                                kVelocityTargetEnd + kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(kTEnd) >=
                                kVelocityTargetEnd - kVelocityTolerance);
  } else {
    program.AddLinearConstraint(program.velocity(kTStart) ==
                                kVelocityTargetStart);
    program.AddLinearConstraint(program.velocity(kTMid) == kVelocityTargetMid);
    program.AddLinearConstraint(program.velocity(kTEnd) == kVelocityTargetEnd);
  }

  if (kAccelerationTolerance(0) > 0) {
    program.AddLinearConstraint(program.acceleration(kTStart) <=
                                kAccelerationTargetStart +
                                    kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTStart) >=
                                kAccelerationTargetStart -
                                    kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTMid) <=
                                kAccelerationTargetMid +
                                    kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTMid) >=
                                kAccelerationTargetMid -
                                    kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTEnd) <=
                                kAccelerationTargetEnd +
                                    kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTEnd) >=
                                kAccelerationTargetEnd -
                                    kAccelerationTolerance);
  } else {
    program.AddLinearConstraint(program.acceleration(kTStart) ==
                                kAccelerationTargetStart);
    program.AddLinearConstraint(program.acceleration(kTMid) ==
                                kAccelerationTargetMid);
    program.AddLinearConstraint(program.acceleration(kTEnd) ==
                                kAccelerationTargetEnd);
  }

  if (kJerkTolerance(0) > 0) {
    program.AddLinearConstraint(program.jerk(kTStart) <=
                                kJerkTargetStart + kJerkTolerance);
    program.AddLinearConstraint(program.jerk(kTStart) >=
                                kJerkTargetStart - kJerkTolerance);
    program.AddLinearConstraint(program.jerk(kTMid) <=
                                kJerkTargetMid + kJerkTolerance);
    program.AddLinearConstraint(program.jerk(kTMid) >=
                                kJerkTargetMid - kJerkTolerance);
    program.AddLinearConstraint(program.jerk(kTEnd) <=
                                kJerkTargetEnd + kJerkTolerance);
    program.AddLinearConstraint(program.jerk(kTEnd) >=
                                kJerkTargetEnd - kJerkTolerance);
  } else if (kJerkTolerance(0) == 0.0) {
    program.AddLinearConstraint(program.jerk(kTStart) == kJerkTargetStart);
    program.AddLinearConstraint(program.jerk(kTMid) == kJerkTargetMid);
    program.AddLinearConstraint(program.jerk(kTEnd) == kJerkTargetEnd);
  }

  VectorX<double> evaluation_times =
      VectorX<double>::LinSpaced(kNumEvaluationPoints, 0.0, 1.0);
  symbolic::Substitution control_point_substitution;
  for (int i = 0; i < kNumPositions; ++i) {
    for (int j = 0; j < kNumControlPoints; ++j) {
      control_point_substitution.emplace(
          program.control_points()(i, j),
          std::sqrt(kJerkWeight) * program.control_points()(i, j));
    }
  }
  Expression jerk_squared_cost{Expression::Zero()};
  for (int i = 0; i < kNumEvaluationPoints; ++i) {
    if (kJerkWeight > 0 && i < kNumEvaluationPoints - 1) {
      VectorX<Expression> jerk0 = program.jerk(evaluation_times(i));
      VectorX<Expression> jerk1 = program.jerk(evaluation_times(i + 1));
      jerk_squared_cost +=
          (evaluation_times(i + 1) - evaluation_times(i)) * 0.5 *
          (jerk0.transpose() * jerk0 + jerk1.transpose() * jerk1)(0);
      // drake::log()->info("Cost for t = {}: {}", evaluation_times(i),
      // jerk_squared_cost);
    }
    if (FLAGS_max_velocity > 0) {
      program.AddLinearConstraint(program.velocity(evaluation_times(i)) <=
                                  kMaxVelocity);
      program.AddLinearConstraint(program.velocity(evaluation_times(i)) >=
                                  -kMaxVelocity);
    }
  }
  if (kJerkWeight > 0) {
    program.AddQuadraticCost(
        jerk_squared_cost.Substitute(control_point_substitution));
  }

  drake::log()->info("Calling program.Solve() ...");
  solvers::SolutionResult result = program.Solve();
  drake::log()->info("... Done. Solver returned {}", result);

  PiecewisePolynomialTrajectory solution_trajectory{
      program.ReconstructTrajectory(1)};

  // Plot trajectory using CallMatlab
  const VectorX<double> x{VectorX<double>::LinSpaced(
      kNumPlottingPoints, solution_trajectory.get_start_time(),
      solution_trajectory.get_end_time())};
  MatrixX<double> position_values(x.size(), kNumPositions);
  MatrixX<double> velocity_values(x.size(), kNumPositions);
  MatrixX<double> acceleration_values(x.size(), kNumPositions);
  MatrixX<double> jerk_values(x.size(), kNumPositions);
  for (int i = 0; i < kNumPlottingPoints; ++i) {
    position_values.row(i) = solution_trajectory.value(x(i))
                                 .topLeftCorner(kNumPositions, 1)
                                 .transpose();
    velocity_values.row(i) = solution_trajectory.derivative(1)
                                 ->value(x(i))
                                 .topLeftCorner(kNumPositions, 1)
                                 .transpose();
    ;
    acceleration_values.row(i) = solution_trajectory.derivative(2)
                                     ->value(x(i))
                                     .topLeftCorner(kNumPositions, 1)
                                     .transpose();
    ;
    jerk_values.row(i) = solution_trajectory.derivative(3)
                             ->value(x(i))
                             .topLeftCorner(kNumPositions, 1)
                             .transpose();
    ;
  }
  CallMatlab("subplot", 4, 1, 1);
  CallMatlab("plot", x, position_values, "LineWidth", 2.0);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Position");
  std::string plot_title("Solution Result:" + std::to_string(result) +
                         ", Solver: " + program.GetSolverId()->name());
  CallMatlab("title", plot_title);
  CallMatlab("subplot", 4, 1, 2);
  CallMatlab("plot", x, velocity_values);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Velocity");
  CallMatlab("subplot", 4, 1, 3);
  CallMatlab("plot", x, acceleration_values);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Acceleration");
  CallMatlab("subplot", 4, 1, 4);
  CallMatlab("plot", x, jerk_values);
  CallMatlab("grid", "on");
  CallMatlab("legend", "Jerk");
  drake::log()->info("Control Point Values = \n{}",
                     program.GetSolution(program.control_points()));

  // Publish an animation of the trajectory to the visualizer.
  do {
    visualizer.PlaybackTrajectory(
        solution_trajectory.get_piecewise_polynomial());
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
