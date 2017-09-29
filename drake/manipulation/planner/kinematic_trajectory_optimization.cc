#include "drake/manipulation/planner/kinematic_trajectory_optimization.h"

namespace drake {
namespace manipulation {
namespace planner {

KinematicTrajectoryOptimization::KinematicTrajectoryOptimization(
    const KinematicPlanningProblem& problem, int num_control_points,
    int num_evaluation_points, int spline_order)
    : kNumControlPoints_(num_control_points),
      kNumEvaluationPoints_(num_evaluation_points > 0 ? num_evaluation_points
                                                      : num_control_points),
      kOrder_(spline_order),
      kNumKnots_(num_control_points + spline_order),
      kNumPositions_(problem.num_positions()),
      control_points_(
          NewContinuousVariables(kNumPositions_, kNumControlPoints_, "q")) {
  for (const auto& cost : problem.costs()) {
  }
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
