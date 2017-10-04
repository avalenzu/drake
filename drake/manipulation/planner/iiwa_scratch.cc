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
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

DEFINE_bool(flat_terrain, true, "If true, add flat terrain to the world.");
DEFINE_bool(loop_animation, true, "If true, repeat playback indefinitely");
DEFINE_int32(order, 6, "Order of the B-splines");
DEFINE_int32(num_control_points, 20, "Number of control points");
DEFINE_int32(num_evaluation_points, -1,
             "Number of points on the spline at which costs and constraints "
             "should be evaluated.");
DEFINE_int32(num_plotting_points, 1000,
             "Number of points to use when plotting.");
DEFINE_double(jerk_weight, -1.0, "Weight applied to the squared jerk cost.");
DEFINE_double(duration, 1, "Duration of the trajectory.");
DEFINE_double(max_velocity, -1.0, "Maximum allowable velocity.");
DEFINE_double(position_tolerance, 0.0, "Maximum position error.");
DEFINE_double(velocity_tolerance, 0.0, "Maximum velocity error.");
DEFINE_double(acceleration_tolerance, 0.0, "Maximum acceleration error.");
DEFINE_double(jerk_tolerance, 0.0, "Maximum jerk error.");

namespace drake {
namespace manipulation {
namespace planner {
namespace {

using common::CallMatlab;
using systems::DrakeVisualizer;
//using symbolic::Expression;

int DoMain() {
  const int kSplineOrder = FLAGS_order;
  const int kNumControlPoints = FLAGS_num_control_points;
  const int kNumPlottingPoints = FLAGS_num_plotting_points;
  const int kNumInternalIntervals{kNumControlPoints - kSplineOrder + 1};
  const int kNumEvaluationPoints = FLAGS_num_evaluation_points > 0
                                       ? FLAGS_num_evaluation_points
                                       : 3 * kNumInternalIntervals + 1;
  const double kJerkWeight = FLAGS_jerk_weight;
  const double kMaxVelocity = FLAGS_max_velocity;
  const double kDuration = FLAGS_duration;

  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  auto iiwa = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      kModelPath, multibody::joints::kFixed, nullptr, iiwa.get());
  if (FLAGS_flat_terrain) {
    drake::multibody::AddFlatTerrainToWorld(iiwa.get());
  }

  KinematicPlanningProblem problem{std::move(iiwa)};

  const int kNumPositions{problem.num_positions()};
  const VectorX<double> kZeroVector{VectorX<double>::Zero(kNumPositions)};
  const VectorX<double> kOnesVector{VectorX<double>::Ones(kNumPositions)};
  const VectorX<double> kPositionTolerance = FLAGS_position_tolerance * kOnesVector;
  const VectorX<double> kVelocityTolerance = FLAGS_velocity_tolerance * kOnesVector;
  const VectorX<double> kAccelerationTolerance =
      FLAGS_acceleration_tolerance * kOnesVector;
  const VectorX<double> kJerkTolerance = FLAGS_jerk_tolerance * kOnesVector;

  KinematicTrajectoryOptimization program{&problem, kNumControlPoints,
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
  const VectorX<double> kPositionTargetEnd{1.0 * kOnesVector};
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
    program.AddLinearConstraint(program.position(kTMid) <=
                                kPositionTargetMid + kPositionTolerance);
    program.AddLinearConstraint(program.position(kTMid) >=
                                kPositionTargetMid - kPositionTolerance);
    program.AddLinearConstraint(program.position(kTEnd) <=
                                kPositionTargetEnd + kPositionTolerance);
    program.AddLinearConstraint(program.position(kTEnd) >=
                                kPositionTargetEnd - kPositionTolerance);
  } else {
    program.AddLinearConstraint(program.position(kTStart) ==
                                kPositionTargetStart);
    program.AddLinearConstraint(program.position(kTMid) == kPositionTargetMid);
    program.AddLinearConstraint(program.position(kTEnd) == kPositionTargetEnd);
  }

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
                                kAccelerationTargetStart + kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTStart) >=
                                kAccelerationTargetStart - kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTMid) <=
                                kAccelerationTargetMid + kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTMid) >=
                                kAccelerationTargetMid - kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTEnd) <=
                                kAccelerationTargetEnd + kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(kTEnd) >=
                                kAccelerationTargetEnd - kAccelerationTolerance);
  } else {
    program.AddLinearConstraint(program.acceleration(kTStart) ==
                                kAccelerationTargetStart);
    program.AddLinearConstraint(program.acceleration(kTMid) == kAccelerationTargetMid);
    program.AddLinearConstraint(program.acceleration(kTEnd) == kAccelerationTargetEnd);
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
  } else {
    program.AddLinearConstraint(program.jerk(kTStart) ==
                                kJerkTargetStart);
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
  for (int i = 0; i < kNumEvaluationPoints; ++i) {
    if (kJerkWeight > 0 && i < kNumEvaluationPoints - 1) {
      auto jerk0 = program.jerk(evaluation_times(i + 1))(0).Substitute(
          control_point_substitution);
      auto jerk1 = program.jerk(evaluation_times(i + 1))(0).Substitute(
          control_point_substitution);
      auto jerk_squared_cost = (evaluation_times(i + 1) - evaluation_times(i)) *
                               0.5 * (jerk0 * jerk0 + jerk1 * jerk1);
      program.AddQuadraticCost(jerk_squared_cost);
      drake::log()->info("Cost for t = {}: {}", evaluation_times(i),
                         jerk_squared_cost);
    }
    if (kMaxVelocity > 0) {
      program.AddLinearConstraint(program.velocity(evaluation_times(i))(0) <=
                                  kMaxVelocity);
      program.AddLinearConstraint(program.velocity(evaluation_times(i))(0) >=
                                  -kMaxVelocity);
    }
  }

  solvers::SolutionResult result = program.Solve();

  PiecewisePolynomialTrajectory solution_trajectory{
      program.ReconstructTrajectory(1)};
  const VectorX<double> x{VectorX<double>::LinSpaced(
      kNumPlottingPoints, solution_trajectory.get_start_time(),
      solution_trajectory.get_end_time())};
  MatrixX<double> position_values(x.size(), kNumPositions);
  MatrixX<double> velocity_values(x.size(), kNumPositions);
  MatrixX<double> acceleration_values(x.size(), kNumPositions);
  MatrixX<double> jerk_values(x.size(), kNumPositions);
  for (int i = 0; i < kNumPlottingPoints; ++i) {
    position_values.row(i) = solution_trajectory.value(x(i)).topLeftCorner(kNumPositions, 1).transpose();
    velocity_values.row(i) =
        solution_trajectory.derivative(1)->value(x(i)).topLeftCorner(kNumPositions, 1).transpose();
    ;
    acceleration_values.row(i) =
        solution_trajectory.derivative(2)->value(x(i)).topLeftCorner(kNumPositions, 1).transpose();
    ;
    jerk_values.row(i) =
        solution_trajectory.derivative(3)->value(x(i)).topLeftCorner(kNumPositions, 1).transpose();
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
  drake::log()->info("Control Point Values = \n{}", program.GetSolution(program.control_points()));

  do {
    visualizer.PlaybackTrajectory(solution_trajectory.get_piecewise_polynomial());
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
