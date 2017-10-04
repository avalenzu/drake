#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/manipulation/planner/kinematic_planning_problem.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

using common::CallMatlab;

DEFINE_int32(order, 4, "Order of the B-splines");
DEFINE_int32(num_control_points, 11, "Number of control points");
DEFINE_int32(num_evaluation_points, -1,
             "Number of points on the spline at which costs and constraints "
             "should be evaluated.");
DEFINE_int32(num_plotting_points, 1000,
             "Number of points to use when plotting.");
DEFINE_double(jerk_weight, 1.0,
            "Weight applied to the squared jerk cost.");
DEFINE_double(max_velocity, -1.0, "Maximum allowable velocity.");
DEFINE_double(position_tolerance, 0.0, "Maximum position error.");
DEFINE_double(velocity_tolerance, 0.0, "Maximum velocity error.");
DEFINE_double(acceleration_tolerance, 0.0, "Maximum acceleration error.");
DEFINE_double(jerk_tolerance, 0.0, "Maximum jerk error.");
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
  const double kPositionTolerance = FLAGS_position_tolerance;
  const double kVelocityTolerance = FLAGS_velocity_tolerance;
  const double kAccelerationTolerance = FLAGS_acceleration_tolerance;
  const double kJerkTolerance = FLAGS_jerk_tolerance;
  const int kNumPositions{1};
  KinematicTrajectoryOptimization program{kNumPositions,
                                          kNumControlPoints,
                                          kNumEvaluationPoints, kSplineOrder};
  if (kPositionTolerance > 0) {
    program.AddLinearConstraint(program.position(0.0)(0) <=
                                0 + kPositionTolerance);
    program.AddLinearConstraint(program.position(0.0)(0) >=
                                0 - kPositionTolerance);
    program.AddLinearConstraint(program.position(0.5)(0) <=
                                0.5 + kPositionTolerance);
    program.AddLinearConstraint(program.position(0.5)(0) >=
                                0.5 - kPositionTolerance);
    program.AddLinearConstraint(program.position(1.0)(0) <=
                                1.0 + kPositionTolerance);
    program.AddLinearConstraint(program.position(1.0)(0) >=
                                1.0 - kPositionTolerance);
  } else {
    program.AddLinearConstraint(program.position(0.0)(0) == 0.0);
    program.AddLinearConstraint(program.position(0.5)(0) == 0.5);
    program.AddLinearConstraint(program.position(1.0)(0) == 1.0);
  }

  if (kVelocityTolerance > 0) {
    program.AddLinearConstraint(program.velocity(0.0)(0) <=
                                0.0 + kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(0.0)(0) >=
                                0.0 - kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(0.5)(0) <=
                                0.0 + kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(0.5)(0) >=
                                0.0 - kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(1.0)(0) <=
                                0.0 + kVelocityTolerance);
    program.AddLinearConstraint(program.velocity(1.0)(0) >=
                                0.0 - kVelocityTolerance);
  } else {
    program.AddLinearConstraint(program.velocity(0.0)(0) == 0.0);
    program.AddLinearConstraint(program.velocity(0.5)(0) == 0.0);
    program.AddLinearConstraint(program.velocity(1.0)(0) == 0.0);
  }

  if (kAccelerationTolerance > 0) {
    program.AddLinearConstraint(program.acceleration(0.0)(0) <=
                                0.0 + kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(0.0)(0) >=
                                0.0 - kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(0.5)(0) <=
                                0.0 + kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(0.5)(0) >=
                                0.0 - kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(1.0)(0) <=
                                0.0 + kAccelerationTolerance);
    program.AddLinearConstraint(program.acceleration(1.0)(0) >=
                                0.0 - kAccelerationTolerance);
  } else {
    program.AddLinearConstraint(program.acceleration(0.0)(0) == 0.0);
    program.AddLinearConstraint(program.acceleration(0.5)(0) == 0.0);
    program.AddLinearConstraint(program.acceleration(1.0)(0) == 0.0);
  }

  if (kJerkTolerance > 0) {
    program.AddLinearConstraint(program.jerk(0.0)(0) <= 0.0 + kJerkTolerance);
    program.AddLinearConstraint(program.jerk(0.0)(0) >= 0.0 - kJerkTolerance);
    program.AddLinearConstraint(program.jerk(0.5)(0) <= 0.0 + kJerkTolerance);
    program.AddLinearConstraint(program.jerk(0.5)(0) >= 0.0 - kJerkTolerance);
    program.AddLinearConstraint(program.jerk(1.0)(0) <= 0.0 + kJerkTolerance);
    program.AddLinearConstraint(program.jerk(1.0)(0) >= 0.0 - kJerkTolerance);
  } else {
    program.AddLinearConstraint(program.jerk(0.0)(0) == 0.0);
    program.AddLinearConstraint(program.jerk(0.5)(0) == 0.0);
    program.AddLinearConstraint(program.jerk(1.0)(0) == 0.0);
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
      auto jerk0 = program.jerk(evaluation_times(i + 1))(0).Substitute(control_point_substitution);
      auto jerk1 = program.jerk(evaluation_times(i + 1))(0).Substitute(control_point_substitution);
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
      program.ReconstructTrajectory()};
  const VectorX<double> x{VectorX<double>::LinSpaced(
      kNumPlottingPoints, solution_trajectory.get_start_time(),
      solution_trajectory.get_end_time())};
  VectorX<double> position_values(x.size());
  VectorX<double> velocity_values(x.size());
  VectorX<double> acceleration_values(x.size());
  VectorX<double> jerk_values(x.size());
  for (int i = 0; i < kNumPlottingPoints; ++i) {
    position_values(i) = solution_trajectory.value(x(i))(0);
    velocity_values(i) = solution_trajectory.derivative(1)->value(x(i))(0);
    acceleration_values(i) = solution_trajectory.derivative(2)->value(x(i))(0);
    jerk_values(i) = solution_trajectory.derivative(3)->value(x(i))(0);
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
